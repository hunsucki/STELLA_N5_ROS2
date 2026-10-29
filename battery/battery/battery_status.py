#!/usr/bin/env python3
"""Battery monitoring and acknowledged, measurement-gated SK120 charging."""

import math
import time

from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32

from .monitor import StellaN5Monitor
from .sk120_driver import SK120Driver


class BatteryNode(Node):
    """Keep charger commands, observed output, and measured current distinct."""

    FAIL_THRESHOLD = 3
    LOW_CURRENT_THRESHOLD = 0.05
    LOW_CURRENT_MAX_COUNT = 3
    GOOD_CURRENT_MIN_COUNT = 2

    def __init__(self):
        super().__init__('battery_node')
        defaults = {
            'port': '/dev/SK120', 'baudrate': 115200, 'slave_id': 1,
            'voltage_set': 25.2, 'start_current': 0.1, 'target_current': 1.8,
            'ramp_step': 0.1, 'ramp_interval': 5.0, 'current_offset': 0.0,
            'status_interval': 2.0, 'poll_interval': 5.0,
            'settle_interval': 5.0, 'sensor_type': 'auto',
            'shunt_resistor': 0.01,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = {name: self.get_parameter(name).value for name in defaults}
        self._validate_parameters(p)
        self._configuration_parameters = set(defaults)
        self.add_on_set_parameters_callback(self._reject_live_config_changes)
        self._port, self._baudrate, self._slave_id = p['port'], p['baudrate'], p['slave_id']
        self._v_set = p['voltage_set']
        self._i_start, self._i_target = p['start_current'], p['target_current']
        self._i_offset = p['current_offset']
        self._i_start_raw = round(self._i_start + self._i_offset, 3)
        self._i_target_raw = round(self._i_target + self._i_offset, 3)
        self._ramp_step, self._ramp_interval = p['ramp_step'], p['ramp_interval']
        self._settle_interval = p['settle_interval']
        self._status_interval = p['status_interval']
        self._monitor_options = {'sensor_type': p['sensor_type'],
                                 'shunt_resistor': p['shunt_resistor']}
        self._monitor = None
        self._sk120 = None
        self._sk120_ready = False
        self._charging = False  # Requested session, not evidence of battery charging.
        self._ramp_active = False
        self._fault_latched = False
        self._shutting_down = False
        self._output_on = None  # None means no fresh readback.
        self._current_set = self._i_start_raw
        self._sk120_current = float('nan')
        self._sk120_voltage = float('nan')
        self._last_status_at = 0.0
        self._changed_at = 0.0
        self._fail_count = 0
        self._write_fail_count = 0
        self._ramp_limit = None
        self._i_target_eff = self._i_target_raw
        self._last_good_current_set = None
        self._good_current_candidate = None
        self._good_current_count = 0
        self._low_current_count = 0
        self._pub_battery = self.create_publisher(BatteryState, '/battery_state', 10)
        self._pub_available = self.create_publisher(BatteryState, '/sk120/available', 10)
        self._pub_on = self.create_publisher(Bool, '/sk120/output_on', 10)
        self._pub_iset = self.create_publisher(Float32, '/sk120/current_set', 10)
        self._pub_iout = self.create_publisher(Float32, '/sk120/current_out', 10)
        self._pub_vout = self.create_publisher(Float32, '/sk120/voltage_out', 10)
        self._pub_vin = self.create_publisher(Float32, '/sk120/voltage_in', 10)
        self._pub_power = self.create_publisher(Float32, '/sk120/power_out', 10)
        self.create_subscription(Bool, '/sk120/cmd_output', self._cb_cmd, 10)
        self.create_timer(p['poll_interval'], self._cb_poll)
        self.create_timer(p['status_interval'], self._cb_status)
        self.create_timer(min(0.5, self._ramp_interval), self._cb_ramp)
        self.get_logger().info(
            f'배터리 노드 시작 | 충전 설정 {self._i_start}A → {self._i_target}A | '
            '실측 확인 후 램프업')

    @staticmethod
    def _validate_parameters(p):
        numeric = ('voltage_set', 'start_current', 'target_current', 'ramp_step',
                   'ramp_interval', 'current_offset', 'status_interval',
                   'poll_interval', 'settle_interval', 'shunt_resistor')
        if any(not math.isfinite(p[k]) for k in numeric):
            raise ValueError('Battery parameters must be finite')
        if not 0 < p['voltage_set'] <= 25.2:
            raise ValueError('6S charging voltage must be in (0, 25.2] V')
        if not 0 < p['start_current'] <= p['target_current'] <= 1.8:
            raise ValueError('Require 0 < start_current <= target_current <= 1.8 A')
        if not 0 <= p['current_offset'] or p['target_current'] + p['current_offset'] > 1.8:
            raise ValueError('Offset-adjusted current must not exceed 1.8 A')
        if any(p[k] <= 0 for k in ('ramp_step', 'ramp_interval', 'status_interval',
                                   'poll_interval', 'shunt_resistor')):
            raise ValueError('Intervals, ramp step and shunt resistance must be positive')
        if p['settle_interval'] < 0 or p['sensor_type'] not in ('auto', 'ina226', 'ina219'):
            raise ValueError('Invalid settle interval or sensor type')
        if not 1 <= p['slave_id'] <= 247 or p['baudrate'] <= 0:
            raise ValueError('Invalid Modbus address or baudrate')

    def _reject_live_config_changes(self, parameters):
        if any(p.name in self._configuration_parameters for p in parameters):
            return SetParametersResult(
                successful=False, reason='Battery configuration requires node restart')
        return SetParametersResult(successful=True)

    def _read_status(self):
        try:
            return self._sk120.get_status()
        except Exception as exc:
            self.get_logger().warn(f'SK120 읽기 오류: {exc}')
            return None

    def _write(self, method, value):
        try:
            return self._sk120 is not None and bool(getattr(self._sk120, method)(value))
        except Exception as exc:
            self.get_logger().error(f'SK120 {method} 오류: {exc}')
            return False

    def _cb_poll(self):
        if self._sk120_ready:
            return
        if self._sk120 is None:
            try:
                self._sk120 = SK120Driver(
                    self._port, baudrate=self._baudrate, slave_id=self._slave_id)
            except Exception:
                return
        status = self._read_status()
        if status is None:
            self._close_sk120()  # Reopen after unplug/replug or serial errors.
            return
        # A restarted node never adopts an output left ON by an earlier process.
        if status.output_on:
            self._write('set_output', False)
            status = self._read_status()
            if status is None or status.output_on:
                self.get_logger().error('SK120 초기 출력 OFF 확인 실패')
                return
        self._sk120_ready = True
        self._fail_count = 0
        self._accept_status(status)
        self._publish_sk120_available()
        self.get_logger().info('[충전 모듈 감지] 통신 및 출력 OFF 확인')

    def _close_sk120(self):
        if self._sk120 is not None:
            try:
                self._sk120.close()
            except Exception:
                pass
        self._sk120 = None

    def _on_undocked(self):
        # Communication loss alone cannot prove physical undocking. Preserve limit.
        self._fault_latched = True
        self._stop_charging()
        self._close_sk120()
        self._sk120_ready = False
        self._output_on = None
        self._invalidate_status()
        self._publish_sk120_available()

    def _publish_sk120_available(self):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sk120'
        msg.present = self._sk120_ready
        msg.percentage = 1.0 if self._sk120_ready else 0.0
        # Legacy docking contact signal; this topic is NOT a charging measurement.
        msg.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING if self._sk120_ready
            else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        self._pub_available.publish(msg)

    def _cb_cmd(self, msg):
        if not msg.data:
            if self._stop_charging():
                self._fault_latched = False
            return
        if self._fault_latched:
            return  # Repeated ON commands must not bypass a protective stop.
        if not self._sk120_ready:
            self.get_logger().warn('[충전 명령 무시] SK120 미연결')
        elif not self._charging:
            self._start_charging()

    def _configure_output(self, current):
        # Never change voltage/current while a previous output may still be active.
        if not self._write('set_output', False):
            return False
        status = self._read_status()
        if status is None or status.output_on:
            return False
        for method, value in (('set_voltage', self._v_set), ('set_current', current)):
            if not self._write(method, value):
                return False
        status = self._read_status()
        if (status is None or status.output_on
                or abs(status.current_set - current) > 0.002
                or abs(status.voltage_set - self._v_set) > 0.02):
            return False
        if not self._write('set_output', True):
            return False
        status = self._read_status()
        if (status is None or not status.output_on
                or abs(status.current_set - current) > 0.002
                or abs(status.voltage_set - self._v_set) > 0.02):
            return False
        self._current_set = current
        self._accept_status(status)
        self._changed_at = time.monotonic()
        self._low_current_count = 0
        self._good_current_count = 0
        self._good_current_candidate = None
        self._write_fail_count = 0
        self.get_logger().info(
            f'[SK120 출력 확인] 입력 {status.voltage_in:.2f}V, '
            f'출력 {status.voltage_out:.2f}V / {status.current_out:.3f}A / '
            f'{status.power_out:.2f}W')
        return True

    def _start_charging(self):
        # Require a readable battery voltage before enabling the charger.
        try:
            voltage = self._monitor.read_voltage()
            if not math.isfinite(voltage) or not 17.5 <= voltage <= 25.2:
                raise ValueError(f'Battery voltage outside charge range: {voltage}')
        except Exception as exc:
            self.get_logger().error(f'[충전 시작 거부] 배터리 계측 오류: {exc}')
            return
        self._i_target_eff = min(self._i_target_raw, self._ramp_limit or self._i_target_raw)
        if not self._configure_output(min(self._i_start_raw, self._i_target_eff)):
            self.get_logger().error('[충전 시작 실패] 설정/출력 readback 불일치')
            self._fault_latched = True
            self._stop_charging()
            return
        self._charging = True
        self._ramp_active = self._current_set < self._i_target_eff
        self._fail_count = 0
        self._last_good_current_set = None
        self.get_logger().info(
            f'[충전 시작] 설정 {self._current_set:.3f}A → {self._i_target_eff:.3f}A')

    def _stop_charging(self, reset_ramp_limit=False):
        self._charging = False
        self._ramp_active = False
        self._low_current_count = 0
        self._good_current_count = 0
        self._good_current_candidate = None
        self._last_good_current_set = None
        if reset_ramp_limit:
            self._ramp_limit = None
        if self._sk120 is None:
            return False
        acknowledged = self._write('set_output', False)
        status = self._read_status()
        if status is not None:
            self._accept_status(status)
        else:
            self._invalidate_status()
        confirmed = acknowledged and status is not None and not status.output_on
        if not confirmed:
            self.get_logger().error('[충전 중지] 출력 OFF 미확인; 통신 복구 후 재시도')
        return confirmed

    def _cb_ramp(self):
        now = time.monotonic()
        if (not self._charging or not self._ramp_active
                or self._output_on is not True or self._fail_count
                or now - self._last_status_at > self._status_interval * 1.5
                or now - self._changed_at < max(self._ramp_interval, self._settle_interval)
                or self._good_current_count < self.GOOD_CURRENT_MIN_COUNT
                or self._low_current_count):
            return
        next_i = round(min(self._current_set + self._ramp_step, self._i_target_eff), 3)
        if not self._write('set_current', next_i):
            self._write_fail_count += 1
            if self._write_fail_count >= self.FAIL_THRESHOLD:
                self._fault_latched = True
                self._stop_charging()
            return
        self._write_fail_count = 0
        self._current_set = next_i
        self._changed_at = now
        self._good_current_count = 0
        self._good_current_candidate = None
        self._low_current_count = 0
        self._ramp_active = next_i < self._i_target_eff
        self.get_logger().info(f'[램프업] 설정 {next_i:.3f}A (실측값과 구분)')

    def _invalidate_status(self):
        self._last_status_at = 0.0
        self._output_on = None
        self._sk120_current = float('nan')
        self._sk120_voltage = float('nan')
        self._good_current_count = 0
        self._good_current_candidate = None
        self._low_current_count = 0
        if self._shutting_down:
            return
        self._pub_iout.publish(Float32(data=float('nan')))
        self._pub_vout.publish(Float32(data=float('nan')))
        self._pub_iset.publish(Float32(data=float('nan')))
        self._pub_vin.publish(Float32(data=float('nan')))
        self._pub_power.publish(Float32(data=float('nan')))

    def _accept_status(self, status):
        if time.monotonic() - self._last_status_at > self._status_interval * 1.5:
            self._good_current_count = 0
            self._good_current_candidate = None
            self._low_current_count = 0
        self._last_status_at = time.monotonic()
        self._output_on = status.output_on
        self._sk120_current = status.current_out
        self._sk120_voltage = status.voltage_out
        if self._shutting_down:
            return
        self._pub_on.publish(Bool(data=status.output_on))
        self._pub_iout.publish(Float32(data=status.current_out))
        self._pub_vout.publish(Float32(data=status.voltage_out))
        self._pub_iset.publish(Float32(data=status.current_set))
        self._pub_vin.publish(Float32(data=status.voltage_in))
        self._pub_power.publish(Float32(data=status.power_out))

    def _cb_status(self):
        # Charger protection must run even if I2C fails.
        if self._sk120_ready:
            status = self._read_status()
            if status is None:
                self._fail_count += 1
                self._invalidate_status()
                if self._fail_count >= self.FAIL_THRESHOLD:
                    self.get_logger().error('[SK120 통신 상실] 충전 중지, 상한 유지')
                    self._on_undocked()
            else:
                self._fail_count = 0
                self._accept_status(status)
                if not self._charging and status.output_on:
                    self._stop_charging()  # Retry an unconfirmed OFF command.
                elif self._charging:
                    if not status.output_on:
                        self.get_logger().warn('[충전 중단] SK120 실측 출력 OFF')
                        self._fault_latched = True
                        self._stop_charging()
                    elif (abs(status.current_set - self._current_set) > 0.002
                          or abs(status.voltage_set - self._v_set) > 0.02):
                        self.get_logger().error('[충전 중단] 설정 readback 불일치')
                        self._fault_latched = True
                        self._stop_charging()
                    elif time.monotonic() - self._changed_at >= self._settle_interval:
                        if status.current_out < self.LOW_CURRENT_THRESHOLD:
                            self._low_current_count += 1
                            self._good_current_count = 0
                            self._good_current_candidate = None
                            if self._low_current_count >= self.LOW_CURRENT_MAX_COUNT:
                                self.get_logger().warn(
                                    f'[SK120 저전류 계측] 입력 {status.voltage_in:.2f}V, '
                                    f'출력 {status.voltage_out:.2f}V / '
                                    f'{status.current_out:.3f}A / {status.power_out:.2f}W')
                                self._handle_low_charge_current(status.current_out)
                        else:
                            self._low_current_count = 0
                            self._remember_good_current(status.current_set)
        self._publish_sk120_available()
        voltage = current = float('nan')
        try:
            if self._monitor is None:
                self._monitor = StellaN5Monitor(**self._monitor_options)
                self.get_logger().info(f'배터리 센서: {self._monitor.sensor_type}')
            voltage = self._monitor.read_voltage()
            current = self._monitor.read_current()
        except Exception as exc:
            voltage = current = float('nan')
            self.get_logger().error(f'배터리 센서 읽기 오류: {exc}')
            if self._charging:
                self._fault_latched = True
                self._stop_charging()
            if self._monitor is not None:
                try:
                    self._monitor.close()
                except OSError:
                    pass
                self._monitor = None
        if self._charging and (not math.isfinite(voltage) or voltage > 25.2 or voltage < 17.5):
            self.get_logger().error(f'[충전 중단] 배터리 전압 범위 초과: {voltage}')
            self._fault_latched = True
            self._stop_charging()
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'battery'
        msg.voltage = float(voltage)  # Charger terminal voltage is a different circuit.
        msg.temperature = msg.charge = float('nan')
        msg.capacity = msg.design_capacity = StellaN5Monitor.CAPACITY_AH
        msg.percentage = (self._monitor.get_soc(voltage) / 100.
                          if self._monitor and math.isfinite(voltage) else float('nan'))
        msg.present = math.isfinite(voltage)
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        if self._charging:
            # Compatibility proxy: SK120 output current, NOT proven net battery current.
            msg.current = float(self._sk120_current)
            msg.power_supply_status = (
                BatteryState.POWER_SUPPLY_STATUS_CHARGING
                if self._output_on and self._sk120_current >= self.LOW_CURRENT_THRESHOLD
                else BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING)
            if self._output_on is None:
                msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        else:
            msg.current = -float(current)
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            if (self._output_on is True or not msg.present
                    or self._output_on is None and self._fault_latched):
                msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        self._pub_battery.publish(msg)

    def _remember_good_current(self, current_set):
        if abs(current_set - self._current_set) > 0.002:
            return
        if self._good_current_candidate != current_set:
            self._good_current_candidate = current_set
            self._good_current_count = 0
        self._good_current_count += 1
        if self._good_current_count >= self.GOOD_CURRENT_MIN_COUNT:
            self._last_good_current_set = current_set
            self._ramp_limit = current_set

    def _handle_low_charge_current(self, measured_current):
        fallback = self._last_good_current_set
        if fallback is None or fallback >= self._current_set - 0.001:
            self.get_logger().warn(
                f'[저전류 보호] 실측 {measured_current:.3f}A, 복구 가능한 하위 전류 없음 → OFF')
            self._fault_latched = True
            self._stop_charging()
            return
        self._ramp_limit = fallback
        self._i_target_eff = fallback
        self.get_logger().warn(
            f'[저전류 복구] 실측 {measured_current:.3f}A → 설정 상한 {fallback:.3f}A; '
            '원인은 전류만으로 확정할 수 없음')
        if not self._configure_output(min(self._i_start_raw, fallback)):
            self._fault_latched = True
            self._stop_charging()
        else:
            self._ramp_active = self._current_set < fallback

    def destroy_node(self):
        self._shutting_down = True
        try:
            self._stop_charging()
        finally:
            self._close_sk120()
            try:
                if self._monitor is not None:
                    self._monitor.close()
            finally:
                super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BatteryNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
