import math
from types import SimpleNamespace
from unittest.mock import Mock

from docking.control import PrecisionYawController, spin_for
from docking.motion import MotionController
import pytest
import rclpy
from rclpy.time import Time


def test_busy_executor_does_not_turn_20hz_motor_loop_into_callback_rate(monkeypatch):
    wall = [10.0]
    callbacks = [0]
    sent = []
    monkeypatch.setattr('time.monotonic', lambda: wall[0])
    monkeypatch.setattr(rclpy, 'ok', lambda: True)

    def spin_once(node, timeout_sec):
        # One immediately ready callback every millisecond; spin_once's
        # timeout is a maximum wait, not a rate limit.
        wall[0] += min(timeout_sec, 0.001)
        callbacks[0] += 1

    monkeypatch.setattr(rclpy, 'spin_once', spin_once)
    motion = MotionController.__new__(MotionController)
    motion.node = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(value=20.0),
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=wall[0])),
        get_logger=Mock(),
    )
    motion.should_stop = lambda: False
    motion.odom_is_fresh = lambda: True
    motion.cmd_vel_pub = SimpleNamespace(publish=lambda msg: sent.append(wall[0]))
    assert motion._run_until(
        lambda: wall[0] >= 10.5, lambda: MotionController.twist(angular_z=0.08), 1.0)
    assert 9 <= len(sent) <= 11
    assert min(b - a for a, b in zip(sent, sent[1:])) >= 0.05 - 1e-9
    assert callbacks[0] >= 400


def test_control_wait_can_be_interrupted_without_another_motion_command(monkeypatch):
    wall = [0.0]
    monkeypatch.setattr('time.monotonic', lambda: wall[0])
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    monkeypatch.setattr(rclpy, 'spin_once', lambda *a, **kw: wall.__setitem__(0, wall[0] + .01))
    spin_for(None, 1.0, lambda: wall[0] >= .03)
    assert wall[0] == pytest.approx(.03)


@pytest.mark.parametrize('direction', [-1, 1])
def test_stationary_stiction_recovery_converges_with_bounded_speed(direction):
    control = PrecisionYawController()
    error = direction * math.radians(22.0)
    rate = 0.0
    commands = []
    for i in range(900):
        if abs(error) <= math.radians(2.0):
            break
        command = control.update(error, rate, rate, i * .05)
        assert control.fault is None
        commands.append(command)
        rate = 0.0 if abs(command) < .1 else .8 * command
        error -= rate * .05
    assert abs(error) <= math.radians(2.0)
    assert max(abs(v) for v in commands) >= .1
    assert max(abs(v) for v in commands) <= .15
    assert max(abs(b - a) for a, b in zip(commands, commands[1:])) <= .015 + 1e-9


@pytest.mark.parametrize('gyro,wheel', [(0.0, 0.0), (.05, 0.0), (0.0, -.05)])
def test_jammed_or_inconsistent_heading_stops_instead_of_continuing(gyro, wheel):
    control = PrecisionYawController()
    for i in range(100):
        command = control.update(math.radians(22), gyro, wheel, i * .05)
        if control.fault:
            break
    assert control.fault
    assert command == 0.0
    if gyro or wheel:
        assert control.recovery_floor == 0.0
        assert i * .05 <= 2.1
    else:
        assert i * .05 <= 4.1


@pytest.mark.parametrize('normal,recovery', [(0, .15), (.08, .2), (.08, .05), (.08, math.nan)])
def test_invalid_recovery_cannot_exceed_existing_spin_speed(normal, recovery):
    with pytest.raises(ValueError):
        PrecisionYawController(normal, recovery)


def test_stale_gyro_does_not_allow_a_stronger_recovery_command():
    control = PrecisionYawController()
    commands = [control.update(.3, 0.0, 0.0, i * .05, allow_recovery=False) for i in range(60)]
    assert control.fault == 'cannot recover yaw stall without fresh independent gyro'
    assert max(commands) <= .08
    assert commands[-1] == 0.0
