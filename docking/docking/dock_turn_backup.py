#!/usr/bin/env python3
"""Start the AprilTag docking stack, dock, turn, back up, and shut it down."""

import math
import os
import shutil
import signal
import subprocess
import sys
import time
from typing import Any

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.action import DockRobot
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class DockTurnBackup(Node):
    def __init__(self) -> None:
        super().__init__('dock_turn_backup')

        self.package_share_dir = get_package_share_directory('docking')
        default_docking_params = os.path.join(
            self.package_share_dir, 'config', 'docking.yaml')

        self.declare_parameter('manage_stack', True)
        self.declare_parameter('start_apriltag', True)
        self.declare_parameter('start_bridge', True)
        self.declare_parameter('start_docking_server', True)
        self.declare_parameter('activate_docking_server', True)
        self.declare_parameter('docking_params_file', default_docking_params)
        self.declare_parameter('stack_startup_delay_sec', 2.0)
        self.declare_parameter('lifecycle_timeout_sec', 20.0)
        self.declare_parameter('docking_server_tf_warmup_sec', 2.0)

        self.declare_parameter('dock_action', '/dock_robot')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('fixed_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tf_wait_timeout_sec', 10.0)
        self.declare_parameter('dock_id', 'home_dock')
        self.declare_parameter('navigate_to_staging_pose', False)
        self.declare_parameter('max_staging_time', 1000.0)
        self.declare_parameter('spin_yaw', math.pi)
        self.declare_parameter('spin_angular_speed', 0.15)
        self.declare_parameter('spin_tolerance', 0.04)
        self.declare_parameter('backup_distance', 0.65)
        self.declare_parameter('backup_speed', 0.05)
        self.declare_parameter('backup_tolerance', 0.02)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('motion_timeout_sec', 45.0)
        self.declare_parameter('server_wait_timeout_sec', 10.0)

        self.managed_processes: list[subprocess.Popen] = []

        self.dock_client = ActionClient(
            self, DockRobot, self.get_parameter('dock_action').value)
        self.cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._odom_callback, 10)
        self.last_odom: Odometry | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def run(self) -> bool:
        if bool(self.get_parameter('manage_stack').value):
            if not self._start_stack():
                return False

        if bool(self.get_parameter('activate_docking_server').value):
            if not self._configure_and_activate_docking_server():
                return False

        if not self._wait_for_dock_server():
            return False

        if not self._wait_for_odom():
            return False

        if not self._wait_for_base_transform():
            return False

        self._warmup_docking_server_tf_buffer()

        if not self._dock_near_tag():
            return False

        if not self._spin_180():
            return False

        return self._backup()

    def _start_stack(self) -> bool:
        ros2 = shutil.which('ros2')
        if ros2 is None:
            self.get_logger().error('Could not find ros2 executable in PATH')
            return False

        if bool(self.get_parameter('start_apriltag').value):
            self._start_process(
                'apriltag',
                [ros2, 'launch', 'docking', 'apriltag_36h11.launch.py'])

        if bool(self.get_parameter('start_bridge').value):
            self._start_process(
                'apriltag_bridge',
                [ros2, 'run', 'docking', 'apriltag_bridge'])

        if bool(self.get_parameter('start_docking_server').value):
            params_file = str(self.get_parameter('docking_params_file').value)
            self._start_process(
                'docking_server',
                [
                    ros2, 'run', 'opennav_docking', 'opennav_docking',
                    '--ros-args', '--params-file', params_file,
                ])

        delay = float(self.get_parameter('stack_startup_delay_sec').value)
        if delay > 0.0:
            time.sleep(delay)

        for process in self.managed_processes:
            if process.poll() is not None:
                self.get_logger().error(
                    f'Managed process exited early: pid={process.pid}, '
                    f'code={process.returncode}')
                return False

        return True

    def _start_process(self, name: str, command: list[str]) -> None:
        self.get_logger().info(f'Starting {name}: {" ".join(command)}')
        env = os.environ.copy()
        env.setdefault('RCUTILS_LOGGING_USE_STDOUT', '1')
        process = subprocess.Popen(command, env=env, start_new_session=True)
        self.managed_processes.append(process)

    def cleanup_managed_processes(self) -> None:
        self._stop_robot()

        for process in reversed(self.managed_processes):
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass

        deadline = time.monotonic() + 5.0
        for process in reversed(self.managed_processes):
            remaining = max(deadline - time.monotonic(), 0.0)
            if process.poll() is None:
                try:
                    process.wait(timeout=remaining)
                except subprocess.TimeoutExpired:
                    process.terminate()

        for process in reversed(self.managed_processes):
            if process.poll() is None:
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    process.kill()

    def _configure_and_activate_docking_server(self) -> bool:
        timeout = float(self.get_parameter('lifecycle_timeout_sec').value)
        get_state_client = self.create_client(GetState, '/docking_server/get_state')
        change_state_client = self.create_client(
            ChangeState, '/docking_server/change_state')

        self.get_logger().info('Waiting for docking_server lifecycle services...')
        if not get_state_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('docking_server get_state service is not available')
            return False
        if not change_state_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('docking_server change_state service is not available')
            return False

        state = self._get_lifecycle_state(get_state_client, timeout)
        if state is None:
            return False
        if state == State.PRIMARY_STATE_ACTIVE:
            self.get_logger().info('docking_server is already active')
            return True

        if state == State.PRIMARY_STATE_UNCONFIGURED:
            if not self._change_lifecycle_state(
                    change_state_client, Transition.TRANSITION_CONFIGURE, timeout):
                return False
            state = self._get_lifecycle_state(get_state_client, timeout)
            if state is None:
                return False

        if state == State.PRIMARY_STATE_INACTIVE:
            return self._change_lifecycle_state(
                change_state_client, Transition.TRANSITION_ACTIVATE, timeout)

        self.get_logger().error(f'docking_server is in unsupported state id={state}')
        return False

    def _get_lifecycle_state(self, client: Any, timeout: float) -> int | None:
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            self.get_logger().error('Failed to get docking_server lifecycle state')
            return None
        state = future.result().current_state
        self.get_logger().info(f'docking_server lifecycle state: {state.label}')
        return state.id

    def _change_lifecycle_state(
            self, client: Any, transition_id: int, timeout: float) -> bool:
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            self.get_logger().error(
                f'Lifecycle transition {transition_id} did not return')
            return False
        if not future.result().success:
            self.get_logger().error(f'Lifecycle transition {transition_id} failed')
            return False
        return True

    def _wait_for_dock_server(self) -> bool:
        timeout = float(self.get_parameter('server_wait_timeout_sec').value)

        self.get_logger().info('Waiting for dock_robot action server...')
        if not self.dock_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('dock_robot action server is not available')
            return False
        return True

    def _wait_for_odom(self) -> bool:
        timeout = float(self.get_parameter('server_wait_timeout_sec').value)
        start = self.get_clock().now()
        self.get_logger().info('Waiting for odom...')

        while rclpy.ok() and self.last_odom is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - start).nanoseconds / 1e9 > timeout:
                self.get_logger().error('odom is not available')
                return False

        return True

    def _wait_for_base_transform(self) -> bool:
        timeout = float(self.get_parameter('tf_wait_timeout_sec').value)
        fixed_frame = str(self.get_parameter('fixed_frame').value)
        base_frame = str(self.get_parameter('base_frame').value)
        start = self.get_clock().now()
        last_log_time = time.monotonic()

        self.get_logger().info(
            f'Waiting for TF transform {fixed_frame} -> {base_frame}...')

        while rclpy.ok():
            try:
                self.tf_buffer.lookup_transform(
                    fixed_frame, base_frame, rclpy.time.Time())
                self.get_logger().info(
                    f'TF transform {fixed_frame} -> {base_frame} is available')
                return True
            except TransformException as exc:
                now = time.monotonic()
                if now - last_log_time >= 1.0:
                    self.get_logger().info(
                        f'Still waiting for TF {fixed_frame} -> {base_frame}: {exc}')
                    last_log_time = now

            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - start).nanoseconds / 1e9 > timeout:
                self.get_logger().error(
                    f'TF transform {fixed_frame} -> {base_frame} is not available')
                return False

        return False

    def _warmup_docking_server_tf_buffer(self) -> None:
        warmup_sec = float(self.get_parameter('docking_server_tf_warmup_sec').value)
        if warmup_sec <= 0.0:
            return

        self.get_logger().info(
            f'Warming up docking_server TF buffer for {warmup_sec:.1f}s...')
        deadline = time.monotonic() + warmup_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _dock_near_tag(self) -> bool:
        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = str(self.get_parameter('dock_id').value)
        goal.navigate_to_staging_pose = bool(
            self.get_parameter('navigate_to_staging_pose').value)
        goal.max_staging_time = float(self.get_parameter('max_staging_time').value)

        self.get_logger().info(
            f'Docking near tag using dock_id="{goal.dock_id}" '
            f'(navigate_to_staging_pose={goal.navigate_to_staging_pose})')
        result = self._send_and_wait(self.dock_client, goal)
        if result is None:
            return False

        dock_result = result.result
        if result.status != GoalStatus.STATUS_SUCCEEDED or not dock_result.success:
            self.get_logger().error(
                'Docking step failed: '
                f'status={result.status}, error_code={dock_result.error_code}, '
                f'error_msg="{dock_result.error_msg}"')
            return False

        self.get_logger().info('Docking step complete; robot is at the tag-front stop point')
        return True

    def _spin_180(self) -> bool:
        target_yaw = float(self.get_parameter('spin_yaw').value)
        speed = abs(float(self.get_parameter('spin_angular_speed').value))
        tolerance = abs(float(self.get_parameter('spin_tolerance').value))
        direction = 1.0 if target_yaw >= 0.0 else -1.0

        self.get_logger().info(
            f'Spinning {target_yaw:.3f} rad at {speed:.3f} rad/s using cmd_vel')

        ok = self._run_motion_until(
            done_cb=self._make_spin_done_cb(abs(target_yaw), tolerance),
            cmd_cb=lambda: self._twist(angular_z=direction * speed),
            timeout_sec=float(self.get_parameter('motion_timeout_sec').value),
        )
        self._stop_robot()

        if ok:
            self.get_logger().info('Spin step complete')
        else:
            self.get_logger().error('Spin step failed or timed out')
        return ok

    def _backup(self) -> bool:
        distance = abs(float(self.get_parameter('backup_distance').value))
        speed = abs(float(self.get_parameter('backup_speed').value))
        tolerance = abs(float(self.get_parameter('backup_tolerance').value))

        self.get_logger().info(
            f'Backing up {distance:.3f} m at {speed:.3f} m/s using cmd_vel')

        ok = self._run_motion_until(
            done_cb=self._make_backup_done_cb(distance, tolerance),
            cmd_cb=lambda: self._twist(linear_x=-speed),
            timeout_sec=float(self.get_parameter('motion_timeout_sec').value),
        )
        self._stop_robot()

        if ok:
            self.get_logger().info('Dock-turn-backup sequence complete')
        else:
            self.get_logger().error('Backup step failed or timed out')
        return ok

    def _send_and_wait(self, client: ActionClient, goal: Any) -> Any:
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Goal was rejected')
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result()

    def _run_motion_until(self, done_cb: Any, cmd_cb: Any, timeout_sec: float) -> bool:
        rate_hz = float(self.get_parameter('control_rate_hz').value)
        sleep_time = 1.0 / max(rate_hz, 1.0)
        start = self.get_clock().now()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            if done_cb():
                return True

            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout_sec:
                return False

            self.cmd_vel_pub.publish(cmd_cb())
            rclpy.spin_once(self, timeout_sec=sleep_time)

        return False

    def _make_spin_done_cb(self, target_yaw: float, tolerance: float) -> Any:
        last_yaw = self._current_yaw()
        accumulated = 0.0

        def done() -> bool:
            nonlocal last_yaw, accumulated
            current_yaw = self._current_yaw()
            delta = self._normalize_angle(current_yaw - last_yaw)
            accumulated += abs(delta)
            last_yaw = current_yaw
            return accumulated >= max(target_yaw - tolerance, 0.0)

        return done

    def _make_backup_done_cb(self, distance: float, tolerance: float) -> Any:
        start_x, start_y = self._current_xy()

        def done() -> bool:
            current_x, current_y = self._current_xy()
            traveled = math.hypot(current_x - start_x, current_y - start_y)
            return traveled >= max(distance - tolerance, 0.0)

        return done

    def _odom_callback(self, msg: Odometry) -> None:
        self.last_odom = msg

    def _current_yaw(self) -> float:
        assert self.last_odom is not None
        orientation = self.last_odom.pose.pose.orientation
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _current_xy(self) -> tuple[float, float]:
        assert self.last_odom is not None
        position = self.last_odom.pose.pose.position
        return position.x, position.y

    def _stop_robot(self) -> None:
        stop = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.02)

    @staticmethod
    def _twist(linear_x: float = 0.0, angular_z: float = 0.0) -> Twist:
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        return twist

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None) -> int:
    rclpy.init(args=args)
    node = DockTurnBackup()

    try:
        ok = node.run()
    except KeyboardInterrupt:
        node.get_logger().warn('Interrupted')
        ok = False
    finally:
        node.cleanup_managed_processes()
        node.destroy_node()
        rclpy.shutdown()

    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
