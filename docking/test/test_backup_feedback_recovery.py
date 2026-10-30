"""Stop and recover encoder delivery without changing a verified corridor."""

from types import SimpleNamespace

from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
import pytest

from test_backup_entry_completion import simulation


def add_feedback_fault(motion, state, monkeypatch, sensor, duration,
                       repeats=1, freeze=False, yaw_drift=False):
    # The existing simulation supplies actual odom/wheel callbacks. Supply
    # accepted stamped IMU callbacks too, rather than just a freshness stub.
    import rclpy
    original_spin = rclpy.spin_once
    original_odom, original_wheel = motion._odom_callback, motion._wheel_yaw_callback
    state.dropouts = []
    def dropping():
        if not state.dropouts and state.gap+state.x < .14:
            state.dropouts.append(state.now)
        elif (len(state.dropouts) < repeats and state.dropouts
              and state.now-state.dropouts[-1] > duration+.8):
            state.dropouts.append(state.now)
        return bool(state.dropouts and state.now-state.dropouts[-1] < duration)
    def deliver_odom(msg):
        if not (dropping() and sensor == 'odom'):
            original_odom(msg)
    def deliver_wheel(msg):
        if not (dropping() and sensor == 'wheel'):
            if freeze and state.dropouts:
                msg.header.stamp.sec = 10
                msg.header.stamp.nanosec = 0
            original_wheel(msg)
    motion._odom_callback, motion._wheel_yaw_callback = deliver_odom, deliver_wheel
    def spin(node, timeout_sec):
        result = original_spin(node, timeout_sec)
        imu = Imu()
        imu.header.stamp = motion.node.get_clock().now().to_msg()
        motion._imu_callback(imu)
        if yaw_drift and state.dropouts and state.now-state.dropouts[0] > duration:
            # Exceed the unchanged 3deg ceiling. A smaller late change may
            # fit the measured rectangle; recovery must never reset the anchor
            # and erase an unsafe change.
            state.yaw = .06
        return result
    monkeypatch.setattr(rclpy, 'spin_once', spin)


def configure_corridor(motion, state):
    state.gap = .48
    motion.node.parameters.update(
        use_lidar_guide_centering=True, backup_guide_center_tolerance=.015,
        backup_guide_visibility_handoff_clearance=.50, backup_guide_known_width=.54)
    motion.backup_lidar_guide_estimator = lambda _: (
        SimpleNamespace(center_offset=.002, separation=.54)
        if state.gap+state.x > .40 else None)


@pytest.mark.parametrize('sensor', ['wheel', 'odom'])
@pytest.mark.parametrize('duration', [.61, 1.4])
def test_brief_actual_delivery_gap_recovers_and_completes_straight_entry(
        simulation, monkeypatch, sensor, duration):
    motion, state = simulation
    configure_corridor(motion, state)
    add_feedback_fault(motion, state, monkeypatch, sensor, duration)
    assert motion.backup_with_lidar()
    assert state.gap+state.x <= .0245
    assert state.linear == state.angular == 0
    logs = motion.node.get_logger().info.call_args_list
    assert any('motion feedback recovered while stopped' in str(c) for c in logs)
    detection = state.dropouts[0]+.5
    # No movement commands while the stale delivery is being recovered.
    assert all(v == w == 0 for at, _, _, v, w in state.sent
               if detection+.05 < at < state.dropouts[0]+duration)
    assert sum('Guide visibility handoff:' in str(c) for c in logs) == 1


@pytest.mark.parametrize('fault', ['persistent', 'duplicate', 'heading_drift', 'total_budget', 'deadline', 'imu_loss'])
def test_recovery_keeps_absolute_feedback_heading_and_time_guards(simulation, monkeypatch, fault):
    motion, state = simulation
    configure_corridor(motion, state)
    duration = 3. if fault == 'persistent' else .61
    add_feedback_fault(motion, state, monkeypatch, 'wheel', duration,
                       repeats=8 if fault == 'total_budget' else 1,
                       freeze=fault == 'duplicate', yaw_drift=fault == 'heading_drift')
    if fault == 'total_budget':
        motion.node.parameters.update(backup_motion_feedback_recovery_timeout_sec=.5,
                                      backup_motion_feedback_recovery_total_sec=.5)
    if fault == 'deadline':
        motion.node.parameters['motion_timeout_sec'] = 10.7
    if fault == 'imu_loss':
        motion.imu_is_fresh = lambda: not state.dropouts or state.now-state.dropouts[0] < .52
    assert not motion.backup_with_lidar()
    assert state.linear == state.angular == 0
    if fault != 'deadline':
        assert state.dropouts
    if fault in ('persistent', 'duplicate'):
        assert state.now-state.dropouts[0] < 2.7
        assert all(v == w == 0 for at, _, _, v, w in state.sent
                   if at > state.dropouts[0]+.55)
    if fault == 'total_budget':
        assert len(state.dropouts) >= 2
    if fault == 'deadline':
        assert state.now < 20.9


@pytest.mark.parametrize('per,total', [(0.,4.), (4.,4.), (2.,1.), (2.,7.), (float('nan'),4.)])
def test_invalid_recovery_budget_cannot_move(simulation, per, total):
    motion, state = simulation
    motion.node.parameters.update(backup_motion_feedback_recovery_timeout_sec=per,
                                  backup_motion_feedback_recovery_total_sec=total)
    assert not motion.backup_with_lidar()
    assert all(v == w == 0 for _, _, _, v, w in state.sent)
