"""Physical response limits for in-place backup correction."""
import math

import pytest
from docking.control import BackupStationaryYawController


@pytest.mark.parametrize('direction', [-1, 1])
def test_caster_deadband_converges_within_existing_speed_bound(direction):
    controller = BackupStationaryYawController()
    yaw = wheel = 0.0
    target = direction * math.radians(4)
    for i in range(160):
        command = controller.update(target-yaw, yaw, wheel, i*.05, math.radians(1))
        assert controller.fault is None
        if abs(target-yaw) <= math.radians(1):
            break
        # Model low-speed resistance; not a fitted model of the real caster.
        yaw += (.55 * command if abs(command) >= .025 else 0.0) * .05
        wheel += command * .05
    assert abs(target-yaw) <= math.radians(1)
    assert i*.05 < 6.0


def test_wheel_motion_with_frozen_chassis_fails_even_if_plane_drifts():
    controller = BackupStationaryYawController()
    for i in range(60):
        t = i*.05
        command = controller.update(math.radians(4-.5*t), 0, .06*t, t, math.radians(1))
        if controller.fault:
            break
    assert command == 0
    assert 'wheels rotate' in controller.fault
    assert t <= 2.1


def test_no_progress_stops_without_spending_entire_backup_timeout():
    controller = BackupStationaryYawController()
    for i in range(81):
        command = controller.update(math.radians(4), 0, 0, i*.05, math.radians(1))
        if controller.fault:
            break
    assert command == 0
    assert 'did not converge' in controller.fault
    assert i*.05 == pytest.approx(3.0)


def test_oscillation_cannot_reset_total_rotation_budget():
    controller = BackupStationaryYawController()
    for i in range(100):
        yaw = .02 if i % 2 else 0
        command = controller.update(.07, yaw, yaw, i*.05, .017)
        if controller.fault:
            break
    assert command == 0
    assert 'travel budget' in controller.fault


def test_noise_does_not_count_as_chassis_rotation():
    controller = BackupStationaryYawController()
    for i in range(60):
        command = controller.update(.07, .001*(-1)**i, .06*i*.05, i*.05, .017)
        if controller.fault:
            break
    assert command == 0
    assert 'wheels rotate' in controller.fault


def test_settled_then_small_overshoot_gets_new_progress_window():
    controller = BackupStationaryYawController()
    tolerance = math.radians(1)
    for t, degrees in [(0, 4), (1, 2), (2, .8), (3, .5), (4, -1.2), (5, -.8)]:
        yaw = math.radians(4 - degrees)
        controller.update(math.radians(degrees), yaw, yaw, t, tolerance)
        assert controller.fault is None


def test_repeated_settling_cannot_extend_absolute_timeout():
    controller = BackupStationaryYawController()
    for i in range(81):
        error = math.radians(.9 if i % 2 else 1.1)
        controller.update(error, 0, 0, i * .1, math.radians(1))
    assert '8 seconds' in controller.fault


@pytest.mark.parametrize('value', [math.nan, math.inf, -.01, .07])
def test_invalid_speed_limits_rejected(value):
    with pytest.raises(ValueError):
        BackupStationaryYawController(max_speed=value)
