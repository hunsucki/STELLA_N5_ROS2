#!/usr/bin/env python3

"""Convert Xbox game-controller input into safe mobile-base velocity commands."""

from collections.abc import Sequence
import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy


def clamp_axis(value: float) -> float:
    """Return a finite joystick axis value limited to [-1.0, 1.0]."""
    value = float(value)
    if not math.isfinite(value):
        return 0.0
    return max(-1.0, min(1.0, value))


def trigger_value(value: float) -> float:
    """Convert an SDL trigger axis from released=0 to fully pressed=1."""
    return max(0.0, -clamp_axis(value))


def calculate_velocity(
    axes: Sequence[float],
    forward_axis: int,
    reverse_axis: int,
    steering_axis: int,
    max_linear_speed: float,
    max_angular_speed: float,
) -> tuple[float, float]:
    """Calculate velocity from RT, LT, and the left stick's horizontal axis."""
    required_axes = (forward_axis, reverse_axis, steering_axis)
    if any(index < 0 or index >= len(axes) for index in required_axes):
        raise IndexError(
            f'configured axes {required_axes} are not present in Joy.axes')

    forward = trigger_value(axes[forward_axis])
    reverse = trigger_value(axes[reverse_axis])
    linear_x = (forward - reverse) * max_linear_speed
    angular_z = clamp_axis(axes[steering_axis]) * max_angular_speed
    return linear_x, angular_z


def button_pressed(buttons: Sequence[int], index: int) -> bool:
    """Return whether a mapped button is pressed; a negative index disables it."""
    return 0 <= index < len(buttons) and buttons[index] != 0


class TeleopJoyNode(Node):
    """Publish ``cmd_vel`` while the configured dead-man button is held."""

    def __init__(self) -> None:
        super().__init__('teleop_joy_node')

        self.declare_parameter('forward_axis', 5)
        self.declare_parameter('reverse_axis', 4)
        self.declare_parameter('steering_axis', 0)
        self.declare_parameter('enable_button', -1)
        self.declare_parameter('max_linear_speed', 0.70)
        self.declare_parameter('max_angular_speed', 1.8)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('joy_timeout_sec', 0.5)

        self._forward_axis = self.get_parameter('forward_axis').value
        self._reverse_axis = self.get_parameter('reverse_axis').value
        self._steering_axis = self.get_parameter('steering_axis').value
        self._enable_button = self.get_parameter('enable_button').value
        self._max_linear_speed = self.get_parameter(
            'max_linear_speed').value
        self._max_angular_speed = self.get_parameter(
            'max_angular_speed').value
        publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self._joy_timeout_sec = self.get_parameter('joy_timeout_sec').value

        self._validate_parameters(publish_rate_hz)

        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._joy_sub = self.create_subscription(
            Joy, 'joy', self._joy_callback, 10)
        self._timer = self.create_timer(
            1.0 / publish_rate_hz, self._publish_timer_callback)

        self._enabled = False
        self._target_linear_x = 0.0
        self._target_angular_z = 0.0
        self._last_joy_time = 0.0
        self._last_invalid_message_log = 0.0

        control_description = (
            'RT drives forward, LT drives in reverse, and the left stick '
            'steers')
        if self._enable_button >= 0:
            control_description += (
                f'; hold button {self._enable_button} as the dead-man switch')
        self.get_logger().info(f'Ready: {control_description}')

    def _validate_parameters(self, publish_rate_hz: float) -> None:
        axes = (self._forward_axis, self._reverse_axis, self._steering_axis)
        if any(axis < 0 for axis in axes):
            raise ValueError('all configured axes must be non-negative')
        if publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be greater than zero')
        if self._joy_timeout_sec <= 0.0:
            raise ValueError('joy_timeout_sec must be greater than zero')

        speeds = (self._max_linear_speed, self._max_angular_speed)
        if not all(math.isfinite(speed) and speed >= 0.0 for speed in speeds):
            raise ValueError('maximum speeds must be finite and non-negative')

    def _joy_callback(self, msg: Joy) -> None:
        now = time.monotonic()
        self._last_joy_time = now

        required_axes = (
            self._forward_axis,
            self._reverse_axis,
            self._steering_axis,
        )
        axes_available = all(axis < len(msg.axes) for axis in required_axes)
        buttons_available = (
            self._enable_button < 0
            or self._enable_button < len(msg.buttons)
        )

        if not axes_available or not buttons_available:
            self._deactivate()
            if now - self._last_invalid_message_log >= 2.0:
                self.get_logger().warning(
                    'Joy message does not contain the configured axes/buttons '
                    f'(axes={len(msg.axes)}, buttons={len(msg.buttons)})')
                self._last_invalid_message_log = now
            return

        if (
            self._enable_button >= 0
            and not button_pressed(msg.buttons, self._enable_button)
        ):
            self._deactivate()
            return

        linear_x, angular_z = calculate_velocity(
            msg.axes,
            self._forward_axis,
            self._reverse_axis,
            self._steering_axis,
            self._max_linear_speed,
            self._max_angular_speed,
        )

        target_changed = (
            linear_x != self._target_linear_x
            or angular_z != self._target_angular_z
        )
        was_enabled = self._enabled
        self._enabled = True
        self._target_linear_x = linear_x
        self._target_angular_z = angular_z

        # Publish changes immediately; the timer keeps the base watchdog fed.
        if not was_enabled or target_changed:
            self._publish_target()

    def _publish_timer_callback(self) -> None:
        if not self._enabled:
            return

        if time.monotonic() - self._last_joy_time > self._joy_timeout_sec:
            self.get_logger().warning('Joystick input timed out; stopping robot')
            self._deactivate()
            return

        self._publish_target()

    def _publish_target(self) -> None:
        command = Twist()
        command.linear.x = self._target_linear_x
        command.angular.z = self._target_angular_z
        self._cmd_vel_pub.publish(command)

    def _deactivate(self) -> None:
        was_enabled = self._enabled
        self._enabled = False
        self._target_linear_x = 0.0
        self._target_angular_z = 0.0
        if was_enabled:
            self.publish_stop()

    def publish_stop(self) -> None:
        """Publish one explicit zero-velocity command."""
        self._cmd_vel_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopJoyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
