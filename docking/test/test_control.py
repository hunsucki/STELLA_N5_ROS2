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


@pytest.mark.parametrize('direction', [-1, 1])
def test_final_yaw_floor_crosses_trial30_wheel_only_low_speed_region(direction):
    control = PrecisionYawController(minimum_speed=.04)
    error = direction * math.radians(2.64)
    gyro = wheel = 0.0
    commands = []
    for i in range(120):
        if abs(error) <= math.radians(1):
            break
        command = control.update(error, gyro, wheel, i * .05)
        assert control.fault is None
        commands.append(command)
        wheel = command
        gyro = .4 * command if abs(command) >= .025 else 0.0
        error -= gyro * .05
    assert abs(error) <= math.radians(1)
    assert i * .05 < 3
    assert max(abs(c) for c in commands) <= .08
    assert all(abs(b-a) <= .015 + 1e-9 for a, b in zip(commands, commands[1:]))


def test_final_yaw_floor_still_stops_when_only_wheels_turn():
    control = PrecisionYawController(minimum_speed=.04)
    command = 0.0
    for i in range(60):
        command = control.update(math.radians(1.05), 0.0, command, i * .05)
        if control.fault:
            break
    assert control.fault == 'heading is not converging despite measured rotation'
    assert command == 0.0
    assert control.recovery_floor == 0.0
    assert i * .05 <= 2.1


@pytest.mark.parametrize('minimum', [-.01, .09, math.nan, math.inf])
def test_invalid_precision_yaw_minimum_is_rejected(minimum):
    with pytest.raises(ValueError):
        PrecisionYawController(minimum_speed=minimum)


def test_centerline_stage_needs_distinct_observations_and_ignores_boundary_jitter():
    from docking.control import CenterlineLatch
    latch = CenterlineLatch(.025)
    for _ in range(20):
        assert not latch.update(1, .02, False)
        assert latch.pending
    assert not latch.update(2, .02, False)
    assert latch.update(3, .02, False)
    for sequence,lateral in enumerate([.024,.030,.026,.028,.024],4):
        assert latch.update(sequence,lateral,False)
    # Returning to straight drive still requires the original tolerance.
    assert latch.update(9,.030,True) and latch.pending
    assert latch.update(10,.030,True) and latch.pending
    assert not latch.update(11,.030,True)


def test_persistent_center_drift_and_missing_observations_reset_the_stage():
    from docking.control import CenterlineLatch
    latch = CenterlineLatch(.025)
    for seq in range(3):
        latch.update(seq,.01,False)
    assert latch.centered
    assert latch.update(3,.04,False)
    assert latch.update(4,.04,False)
    assert not latch.update(5,.04,False)
    latch.update(6,.01,False)
    latch.reset()
    assert not latch.update(7,.01,False)
    assert not latch.update(8,math.nan,False)
    assert not latch.centered


def test_axle_centering_keeps_yaw_stage_but_checks_body_before_straight_drive():
    from docking.control import CenterlineLatch
    latch = CenterlineLatch(.025, entering_tolerance=.012)
    for seq in range(3):
        latch.update(seq,.01,False,settled_lateral=.035)
    assert latch.centered
    # Small axle residual during a turn must not restart point steering.
    assert latch.update(3,.020,False,settled_lateral=.035)
    assert not latch.pending
    # Settled heading requires the original body tolerance, regardless of axle.
    assert latch.update(4,.010,True,settled_lateral=.030) and latch.pending
    assert latch.update(5,.010,True,settled_lateral=.030) and latch.pending
    assert not latch.update(6,.010,True,settled_lateral=.030)
