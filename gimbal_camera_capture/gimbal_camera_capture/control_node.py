"""ROS 2 topic control for two SIYI A8 mini gimbals."""

from datetime import datetime
import json
import time
from typing import Dict

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float32, String

from .siyi_protocol import normalized_speed, SiyiUdpClient


class GimbalState:
    """Keep one camera client and its dead-man watchdog state."""

    def __init__(
        self,
        client: SiyiUdpClient,
        yaw_direction: int,
        pitch_direction: int,
    ) -> None:
        """Initialize a camera state."""
        self.client = client
        self.yaw_direction = yaw_direction
        self.pitch_direction = pitch_direction
        self.motion_active = False
        self.zoom_active = False
        self.motion_deadline = 0.0
        self.zoom_deadline = 0.0


class GimbalControlNode(Node):
    """Translate typed ROS topics into SIYI UDP gimbal commands."""

    def __init__(self) -> None:
        """Declare camera endpoints and create control subscriptions."""
        super().__init__('gimbal_control')

        self.declare_parameter('left_ip', '192.168.144.25')
        self.declare_parameter('left_port', 37260)
        self.declare_parameter('left_bind_address', '192.168.144.10')
        self.declare_parameter('left_yaw_direction', 1)
        self.declare_parameter('left_pitch_direction', 1)
        self.declare_parameter('right_ip', '192.168.144.26')
        self.declare_parameter('right_port', 37260)
        self.declare_parameter('right_bind_address', '192.168.144.11')
        self.declare_parameter('right_yaw_direction', 1)
        self.declare_parameter('right_pitch_direction', 1)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('step_duration_sec', 0.15)
        self.declare_parameter('step_speed', 40)
        self.declare_parameter('result_topic', '/gimbal/control/result')

        self._timeout = max(
            0.05,
            float(self.get_parameter('command_timeout_sec').value),
        )
        self._step_duration = max(
            0.02,
            float(self.get_parameter('step_duration_sec').value),
        )
        self._step_speed = max(
            1,
            min(100, int(self.get_parameter('step_speed').value)),
        )
        self._states: Dict[str, GimbalState] = {}
        try:
            for name in ('left', 'right'):
                yaw_direction = int(
                    self.get_parameter(f'{name}_yaw_direction').value
                )
                pitch_direction = int(
                    self.get_parameter(f'{name}_pitch_direction').value
                )
                if yaw_direction not in (-1, 1):
                    raise ValueError(
                        f'{name}_yaw_direction must be -1 or 1'
                    )
                if pitch_direction not in (-1, 1):
                    raise ValueError(
                        f'{name}_pitch_direction must be -1 or 1'
                    )
                client = SiyiUdpClient(
                    str(self.get_parameter(f'{name}_ip').value),
                    int(self.get_parameter(f'{name}_port').value),
                    str(self.get_parameter(f'{name}_bind_address').value),
                )
                self._states[name] = GimbalState(
                    client,
                    yaw_direction,
                    pitch_direction,
                )
        except Exception:
            for state in self._states.values():
                state.client.close()
            raise

        result_topic = str(self.get_parameter('result_topic').value)
        self._result_publisher = self.create_publisher(
            String,
            result_topic,
            10,
        )
        self._control_subscriptions = []
        for name in self._states:
            self._control_subscriptions.extend([
                self.create_subscription(
                    String,
                    f'/gimbal/{name}/move',
                    lambda message, camera=name: self._on_move(
                        camera,
                        message,
                    ),
                    10,
                ),
                self.create_subscription(
                    Twist,
                    f'/gimbal/{name}/cmd_vel',
                    lambda message, camera=name: self._on_velocity(
                        camera,
                        message,
                    ),
                    10,
                ),
                self.create_subscription(
                    Float32,
                    f'/gimbal/{name}/zoom',
                    lambda message, camera=name: self._on_zoom(
                        camera,
                        message,
                    ),
                    10,
                ),
                self.create_subscription(
                    Empty,
                    f'/gimbal/{name}/center',
                    lambda message, camera=name: self._on_center(
                        camera,
                        message,
                    ),
                    10,
                ),
            ])

        self._watchdog_timer = self.create_timer(0.05, self._watchdog)
        self.get_logger().info(
            'Gimbal control ready: /gimbal/{left,right}/'
            'move, cmd_vel, zoom, center'
        )

    def _on_move(self, camera: str, message: String) -> None:
        commands = {
            'up': 'up',
            'down': 'down',
            'left': 'left',
            'right': 'right',
            'stop': 'stop',
            '상': 'up',
            '하': 'down',
            '좌': 'left',
            '우': 'right',
            '정지': 'stop',
        }
        requested = message.data.strip().lower()
        command = commands.get(requested)
        if command is None:
            self._publish_result(
                camera,
                'move',
                False,
                f'unsupported direction: {message.data!r}',
            )
            return

        if command == 'stop':
            self._stop_motion(camera)
            self._publish_result(camera, 'move', True, values={
                'direction': command,
            })
            return

        state = self._states[camera]
        yaw = 0
        pitch = 0
        if command == 'up':
            pitch = self._step_speed * state.pitch_direction
        elif command == 'down':
            pitch = -self._step_speed * state.pitch_direction
        elif command == 'left':
            yaw = -self._step_speed * state.yaw_direction
        elif command == 'right':
            yaw = self._step_speed * state.yaw_direction

        try:
            state.client.rotate(yaw, pitch)
        except OSError as error:
            self._stop_motion(camera)
            self._publish_result(camera, 'move', False, str(error))
            return

        state.motion_deadline = time.monotonic() + self._step_duration
        state.motion_active = True
        self._publish_result(
            camera,
            'move',
            True,
            values={
                'direction': command,
                'duration_sec': self._step_duration,
                'pitch': pitch,
                'yaw': yaw,
            },
        )

    def _on_velocity(self, camera: str, message: Twist) -> None:
        state = self._states[camera]
        try:
            yaw = normalized_speed(
                message.angular.z,
                state.yaw_direction,
            )
            pitch = normalized_speed(
                message.angular.y,
                state.pitch_direction,
            )
            state.client.rotate(yaw, pitch)
        except (OSError, ValueError) as error:
            self._stop_motion(camera)
            self._publish_result(camera, 'velocity', False, str(error))
            return

        state.motion_deadline = time.monotonic() + self._timeout
        state.motion_active = yaw != 0 or pitch != 0
        self._publish_result(
            camera,
            'velocity',
            True,
            values={'yaw': yaw, 'pitch': pitch},
        )

    def _on_zoom(self, camera: str, message: Float32) -> None:
        state = self._states[camera]
        if message.data > 0.0:
            direction = 1
        elif message.data < 0.0:
            direction = -1
        else:
            direction = 0

        try:
            state.client.zoom(direction)
        except OSError as error:
            self._stop_zoom(camera)
            self._publish_result(camera, 'zoom', False, str(error))
            return

        state.zoom_deadline = time.monotonic() + self._timeout
        state.zoom_active = direction != 0
        self._publish_result(
            camera,
            'zoom',
            True,
            values={'direction': direction},
        )

    def _on_center(self, camera: str, _message: Empty) -> None:
        self._stop_motion(camera)
        try:
            self._states[camera].client.center()
        except OSError as error:
            self._publish_result(camera, 'center', False, str(error))
            return
        self._publish_result(camera, 'center', True)

    def _stop_motion(self, camera: str) -> None:
        state = self._states[camera]
        try:
            state.client.rotate(0, 0)
        except OSError as error:
            self.get_logger().error(
                f'Failed to stop {camera} gimbal: {error}'
            )
        state.motion_active = False

    def _stop_zoom(self, camera: str) -> None:
        state = self._states[camera]
        try:
            state.client.zoom(0)
        except OSError as error:
            self.get_logger().error(
                f'Failed to stop {camera} zoom: {error}'
            )
        state.zoom_active = False

    def _watchdog(self) -> None:
        now = time.monotonic()
        for camera, state in self._states.items():
            if (
                state.motion_active
                and now >= state.motion_deadline
            ):
                self._stop_motion(camera)
                self._publish_result(camera, 'watchdog_stop', True)
            if (
                state.zoom_active
                and now >= state.zoom_deadline
            ):
                self._stop_zoom(camera)
                self._publish_result(camera, 'zoom_watchdog_stop', True)

    def _publish_result(
        self,
        camera: str,
        action: str,
        success: bool,
        error: str = '',
        values: Dict = None,
    ) -> None:
        result = String()
        result.data = json.dumps(
            {
                'action': action,
                'camera': camera,
                'error': error,
                'success': success,
                'timestamp': datetime.now().astimezone().isoformat(),
                'values': values or {},
            },
            ensure_ascii=False,
            sort_keys=True,
        )
        self._result_publisher.publish(result)

    def destroy_node(self) -> bool:
        """Stop both gimbals and close their UDP sockets."""
        for camera, state in self._states.items():
            try:
                state.client.stop()
            except OSError as error:
                self.get_logger().error(
                    f'Failed to stop {camera} during shutdown: {error}'
                )
            state.client.close()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the dual SIYI gimbal control node."""
    rclpy.init(args=args)
    node = GimbalControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
