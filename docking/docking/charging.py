"""Wireless charger command and physical charging verification."""

import time
from typing import Callable

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool


class ChargingVerifier:
    @staticmethod
    def declare_parameters(node: Node) -> None:
        node.declare_parameter('charger_available_topic', '/sk120/available')
        node.declare_parameter('charger_command_topic', '/sk120/cmd_output')
        node.declare_parameter('battery_state_topic', '/battery_state')
        node.declare_parameter('charger_wait_timeout_sec', 18.0)
        node.declare_parameter('charging_stable_sec', 3.0)
        node.declare_parameter('charging_min_current', 0.05)
        node.declare_parameter('charging_status_max_age_sec', 3.0)
        node.declare_parameter('charging_command_rate_hz', 2.0)
        node.declare_parameter('existing_charging_wait_sec', 2.5)

    def __init__(self, node: Node, should_stop: Callable[[], bool]) -> None:
        self.node = node
        self.should_stop = should_stop
        self.last_available: BatteryState | None = None
        self.last_available_at = 0.0
        self.last_battery: BatteryState | None = None
        self.last_battery_at = 0.0
        self.charging_since: float | None = None
        self.command_sent = False
        self.confirmed = False

        self.command_pub = node.create_publisher(
            Bool, node.get_parameter('charger_command_topic').value, 10)
        self.available_sub = node.create_subscription(
            BatteryState,
            node.get_parameter('charger_available_topic').value,
            self._available_callback,
            10)
        self.battery_sub = node.create_subscription(
            BatteryState,
            node.get_parameter('battery_state_topic').value,
            self._battery_callback,
            10)

    def already_charging(self) -> bool:
        return self._battery_is_charging(time.monotonic())

    def wait_for_existing_charge(self) -> bool:
        timeout = float(
            self.node.get_parameter('existing_charging_wait_sec').value)
        deadline = time.monotonic() + max(timeout, 0.0)
        while rclpy.ok() and not self.should_stop():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.already_charging():
                self.confirmed = True
                self.node.get_logger().info(
                    'Robot is already docked and charging; no motion is required')
                return True
            if time.monotonic() >= deadline:
                return False
        return False

    def verify(self) -> bool:
        timeout = float(self.node.get_parameter('charger_wait_timeout_sec').value)
        command_rate = float(
            self.node.get_parameter('charging_command_rate_hz').value)
        command_period = 1.0 / max(command_rate, 0.2)
        deadline = time.monotonic() + timeout
        next_command_at = 0.0
        last_log_at = 0.0

        self.node.get_logger().info(
            'Waiting for charger contact and stable charging current...')

        while rclpy.ok() and not self.should_stop():
            now = time.monotonic()
            if now >= deadline:
                self.node.get_logger().error(
                    'Charger contact or stable charging current was not confirmed')
                return False

            rclpy.spin_once(self.node, timeout_sec=0.1)
            now = time.monotonic()

            if self._charger_is_available(now) and now >= next_command_at:
                self.command_pub.publish(Bool(data=True))
                self.command_sent = True
                next_command_at = now + command_period

            if self._charging_is_stable(now):
                self.confirmed = True
                battery = self.last_battery
                assert battery is not None
                self.node.get_logger().info(
                    'Physical docking confirmed: '
                    f'charging current={battery.current:.3f}A')
                return True

            if now - last_log_at >= 1.0:
                if not self._charger_is_available(now):
                    detail = 'waiting for /sk120/available contact'
                elif not self._battery_is_charging(now):
                    detail = 'charger detected; waiting for positive battery current'
                else:
                    detail = 'charging detected; checking stability'
                self.node.get_logger().info(detail)
                last_log_at = now

        return False

    def cancel_unconfirmed_charging(self) -> None:
        if not self.command_sent or self.confirmed or not rclpy.ok():
            return
        for _ in range(5):
            self.command_pub.publish(Bool(data=False))
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _charger_is_available(self, now: float) -> bool:
        msg = self.last_available
        if msg is None or not self._is_fresh(self.last_available_at, now):
            return False
        return (
            msg.present
            and msg.power_supply_status
            == BatteryState.POWER_SUPPLY_STATUS_CHARGING)

    def _battery_is_charging(self, now: float) -> bool:
        msg = self.last_battery
        if msg is None or not self._is_fresh(self.last_battery_at, now):
            return False
        min_current = float(
            self.node.get_parameter('charging_min_current').value)
        return (
            msg.present
            and msg.power_supply_status
            == BatteryState.POWER_SUPPLY_STATUS_CHARGING
            and msg.current >= min_current)

    def _charging_is_stable(self, now: float) -> bool:
        if not self._charger_is_available(now) or not self._battery_is_charging(now):
            return False
        if self.charging_since is None:
            return False
        stable_sec = float(self.node.get_parameter('charging_stable_sec').value)
        return now - self.charging_since >= stable_sec

    def _is_fresh(self, received_at: float, now: float) -> bool:
        max_age = float(
            self.node.get_parameter('charging_status_max_age_sec').value)
        return now - received_at <= max_age

    def _available_callback(self, msg: BatteryState) -> None:
        self.last_available = msg
        self.last_available_at = time.monotonic()

    def _battery_callback(self, msg: BatteryState) -> None:
        now = time.monotonic()
        self.last_battery = msg
        self.last_battery_at = now
        if self._battery_is_charging(now):
            if self.charging_since is None:
                self.charging_since = now
        else:
            self.charging_since = None
