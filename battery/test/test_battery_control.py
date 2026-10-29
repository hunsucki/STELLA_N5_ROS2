"""Regression tests for real sensor decoding and charger fault handling."""

from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock

from battery import battery_status, monitor
from battery.sk120_driver import SK120Status
import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Bool


@pytest.mark.parametrize('raw, expected', [
    (0x0010, .004), (0xfff0, -.004), (0x0100, .064), (0x8000, -8.192)])
def test_ina226_current_byte_order_and_sign(monkeypatch, raw, expected):
    registers = {0xfe: 0x5449, 0xff: 0x2260, 1: raw, 2: 19600}
    bus = Mock()
    bus.read_word_data.side_effect = lambda addr, reg: (
        ((registers[reg] & 255) << 8) | (registers[reg] >> 8))
    monkeypatch.setattr(monitor, 'SMBus', lambda _: bus)
    sensor = monitor.StellaN5Monitor()
    assert sensor.sensor_type == 'ina226'
    assert sensor.read_current() == pytest.approx(expected)
    assert sensor.read_voltage() == pytest.approx(24.5)


def test_ina219_explicit_scaling(monkeypatch):
    bus = Mock()
    bus.read_word_data.side_effect = [0x80bb, 0xf0ff]  # 0xbb80 bus, 0xfff0 shunt
    monkeypatch.setattr(monitor, 'SMBus', lambda _: bus)
    sensor = monitor.StellaN5Monitor(sensor_type='ina219')
    assert sensor.read_voltage() == pytest.approx(24.)
    assert sensor.read_current() == pytest.approx(-.016)


class FakeMonitor:
    MAX_V = 25.2
    MIN_V = 17.5
    CAPACITY_AH = 13.0
    sensor_type = 'ina226'
    voltage = 24.5
    fail = False

    def __init__(self, **kwargs):
        pass

    def read_voltage(self):
        if self.fail:
            raise OSError('I2C fault')
        return self.voltage

    def read_current(self):
        return .004

    def get_soc(self, voltage):
        return 90.

    def close(self):
        pass


class FakeDriver:
    def __init__(self, *args, **kwargs):
        self.status = SK120Status(25.2, .1, 25.2, .1, 2.52, 30., False)
        self.fail_method = None
        self.read_fail = False
        self.calls = []

    def get_status(self):
        if self.read_fail:
            raise OSError('Serial disconnected')
        return self.status

    def write(self, method, value, field):
        self.calls.append((method, value))
        if self.fail_method == method:
            return False
        self.status = replace(self.status, **{field: value})
        return True

    def set_voltage(self, value):
        return self.write('set_voltage', value, 'voltage_set')

    def set_current(self, value):
        return self.write('set_current', value, 'current_set')

    def set_output(self, value):
        return self.write('set_output', value, 'output_on')

    def close(self):
        pass


@pytest.fixture
def node(monkeypatch):
    clock = SimpleNamespace(now=100.)
    monkeypatch.setattr(battery_status, 'time', SimpleNamespace(monotonic=lambda: clock.now))
    monkeypatch.setattr(battery_status, 'StellaN5Monitor', FakeMonitor)
    monkeypatch.setattr(battery_status, 'SK120Driver', FakeDriver)
    rclpy.init()
    n = battery_status.BatteryNode()
    n.test_clock = clock
    n._cb_status()
    n._cb_poll()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def sample(node, amps=.1):
    node.test_clock.now += 2.
    node._sk120.status = replace(node._sk120.status, current_out=amps)
    node._cb_status()


def start(node):
    node._cb_cmd(Bool(data=True))
    assert node._charging


@pytest.mark.parametrize('method', ['set_output', 'set_voltage', 'set_current'])
def test_failed_start_is_not_reported_as_charging(node, method):
    node._sk120.fail_method = method
    node._cb_cmd(Bool(data=True))
    assert not node._charging
    assert node._fault_latched
    assert ('set_output', True) not in node._sk120.calls


def test_no_ramp_without_measured_current_and_repeated_on_cannot_restart(node):
    start(node)
    for _ in range(8):
        sample(node, 0.)
        node._cb_ramp()
    assert node._current_set == .1
    assert not node._charging
    assert node._fault_latched
    node._cb_cmd(Bool(data=True))
    assert not node._charging
    node._cb_cmd(Bool(data=False))
    assert not node._fault_latched


def test_each_step_requires_settle_and_two_good_samples(node):
    start(node)
    for _ in range(3):
        sample(node)
        node._cb_ramp()
    assert node._current_set == .1
    sample(node)
    node._cb_ramp()
    assert node._current_set == .2
    assert node._ramp_limit == .1
    sample(node)
    node._cb_ramp()
    assert node._current_set == .2


def test_low_current_backs_off_to_measured_good_step(node):
    start(node)
    for _ in range(4):
        sample(node)
    node._cb_ramp()
    assert node._current_set == .2
    for _ in range(5):
        sample(node, 0.)
    assert node._charging
    assert node._current_set == .1
    assert node._i_target_eff == .1
    for _ in range(5):
        sample(node, 0.)
    assert not node._charging
    assert node._fault_latched


def test_disconnect_stales_readings_freezes_ramp_and_preserves_limit(node):
    start(node)
    for _ in range(4):
        sample(node)
    node._ramp_limit = .1
    driver = node._sk120
    driver.read_fail = True
    node._cb_status()
    assert node._output_on is None
    node._cb_ramp()
    assert node._current_set == .1
    node._cb_status()
    node._cb_status()
    assert not node._charging
    assert not node._sk120_ready
    assert node._sk120 is None
    assert node._ramp_limit == .1


def test_ina_failure_does_not_skip_charger_protection(node):
    start(node)
    node._monitor.fail = True
    node._cb_status()
    assert not node._charging
    assert not node._output_on
    assert node._fault_latched


def test_actual_output_off_ends_session(node):
    start(node)
    node._sk120.status = replace(node._sk120.status, output_on=False)
    node._cb_status()
    assert not node._charging
    assert not node._output_on


def test_failed_off_is_reported_on_and_retried(node):
    start(node)
    node._sk120.fail_method = 'set_output'
    assert not node._stop_charging()
    assert node._output_on is True
    node._sk120.fail_method = None
    node._cb_status()
    assert node._output_on is False


def test_external_setpoint_change_stops_charge(node):
    start(node)
    node._sk120.status = replace(node._sk120.status, current_set=.8)
    node._cb_status()
    assert not node._charging
    assert node._fault_latched


def test_ramp_write_failures_accumulate_despite_good_reads(node):
    start(node)
    for _ in range(4):
        sample(node)
    node._sk120.fail_method = 'set_current'
    for _ in range(3):
        node._cb_ramp()
        sample(node)
    assert not node._charging
    assert node._fault_latched


def test_acknowledged_but_unchanged_setting_never_enables_output(node):
    driver = node._sk120
    driver.status = replace(driver.status, voltage_set=20.)
    driver.set_voltage = lambda value: True
    node._cb_cmd(Bool(data=True))
    assert not node._charging
    assert not driver.status.output_on
    assert ('set_output', True) not in driver.calls


def test_full_ramp_reaches_limit_without_overshoot(node):
    start(node)
    for _ in range(100):
        sample(node)
        node._cb_ramp()
    assert node._charging
    assert node._current_set == 1.8
    assert not node._ramp_active


def test_shutdown_turns_output_off_without_publishing_to_closed_context(node):
    start(node)
    node._shutting_down = True
    node._pub_on = Mock()
    node._stop_charging()
    assert node._output_on is False
    node._pub_on.publish.assert_not_called()


def test_partial_ina_read_does_not_publish_present_or_discharge_status(node):
    node._monitor.read_current = Mock(side_effect=OSError('Shunt read failure'))
    node._pub_battery = Mock()
    node._cb_status()
    message = node._pub_battery.publish.call_args.args[0]
    assert not message.present
    assert message.power_supply_status == message.POWER_SUPPLY_STATUS_UNKNOWN


def test_live_parameter_change_cannot_silently_diverge_from_active_config(node):
    result = node.set_parameters([Parameter('target_current', value=.5)])[0]
    assert not result.successful
    assert node.get_parameter('target_current').value == 1.8


def test_restart_disables_preexisting_hardware_output(node):
    node._sk120_ready = False
    node._sk120.status = replace(node._sk120.status, output_on=True)
    node._cb_poll()
    assert node._sk120_ready
    assert node._output_on is False
    assert not node._charging


def test_gap_does_not_count_as_continuous_good_current(node):
    start(node)
    for _ in range(3):
        sample(node)
    assert node._good_current_count == 1
    node.test_clock.now += 20.
    sample(node)
    assert node._good_current_count == 1
    node._cb_ramp()
    assert node._current_set == .1
