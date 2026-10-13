"""Exercise the rear aligner through real odom callback freshness failures."""

import math

from docking.lidar_alignment import LidarPlaneAligner
import pytest
from test_backup_entry_completion import simulation  # noqa: F401


@pytest.fixture
def rear_alignment(simulation):
    motion, state = simulation
    LidarPlaneAligner.declare_parameters(motion.node)
    motion.node.parameters['odom_max_age_sec'] = .10
    aligner = LidarPlaneAligner(motion.node, motion, motion.lidar)
    aligner.estimate_error = lambda _: (
        -state.yaw, math.pi / 2 - state.yaw, 100, 117, .51)
    return aligner, motion, state


def interrupt_odom(motion, state, end, fault='missing', start=10.35):
    original = motion._odom_callback
    cached = motion.last_odom

    def deliver(msg):
        if start <= state.now < end:
            if fault == 'duplicate':
                original(cached)
            elif fault == 'old_header':
                msg.header.stamp.sec -= 2
                original(msg)
            return
        original(msg)

    motion._odom_callback = deliver


def logs(motion, level):
    return [call.args[0] for call in
            getattr(motion.node.get_logger(), level).call_args_list]


def test_healthy_alignment_has_no_recovery_delay(rear_alignment):
    aligner, motion, state = rear_alignment
    assert aligner.align()
    assert state.now < 10.9
    assert not logs(motion, 'warning')


@pytest.mark.parametrize('fault', ['missing', 'duplicate', 'old_header'])
def test_brief_odom_loss_stops_and_reacquires_new_plane(rear_alignment, fault):
    aligner, motion, state = rear_alignment
    interrupt_odom(motion, state, end=10.95, fault=fault)
    estimated = []
    original = aligner.estimate_error

    def estimate(snapshot):
        estimated.append((state.now, snapshot.sequence))
        return original(snapshot)

    aligner.estimate_error = estimate

    assert aligner.align()
    assert all(v == w == 0 for _, _, _, v, w in state.sent)
    assert any('Pausing LiDAR alignment' in line for line in logs(motion, 'warning'))
    assert any('Odometry recovered' in line for line in logs(motion, 'info'))
    assert sum('rear plane acquired' in line for line in logs(motion, 'info')) == 2
    # Eight distinct post-recovery scans: three acquisition, five settled.
    recovered_scans = [seq for t, seq in estimated if t >= 10.95]
    assert len(recovered_scans) == len(set(recovered_scans)) == 8
    assert min(recovered_scans) > 9
    assert 'unique_scans=5' in logs(motion, 'info')[-1]


@pytest.mark.parametrize('fault', ['missing', 'duplicate', 'old_header'])
def test_persistent_odom_loss_cannot_resume_or_complete(rear_alignment, fault):
    aligner, motion, state = rear_alignment
    interrupt_odom(motion, state, end=100, fault=fault)

    assert not aligner.align()
    assert 11.4 <= state.now <= 11.55
    assert all(v == w == 0 for _, _, _, v, w in state.sent)
    message = logs(motion, 'error')[-1]
    assert 'Odometry did not recover' in message
    assert 'header_age=' in message and 'receipt_age=' in message
    if fault != 'missing':
        assert 'last_rejection=invalid, repeated' in message


def test_active_rotation_stops_during_odom_loss(rear_alignment):
    aligner, motion, state = rear_alignment
    state.yaw = math.radians(4)
    interrupt_odom(motion, state, end=11.3, start=10.5)

    assert aligner.align()
    assert any(w != 0 for t, _, _, _, w in state.sent if t < 10.6)
    waiting = [cmd for cmd in state.sent if 10.65 <= cmd[0] < 11.3]
    assert waiting and all(v == w == 0 for _, _, _, v, w in waiting)
    assert state.linear == state.angular == 0


def test_rotation_across_odom_loss_still_counts_against_limit(rear_alignment):
    aligner, motion, state = rear_alignment
    original = motion._odom_callback

    def deliver(msg):
        if 10.35 <= state.now < 10.95:
            return
        if state.now >= 10.95:
            state.yaw = math.radians(19)
            msg.pose.pose.orientation.z = math.sin(state.yaw / 2)
            msg.pose.pose.orientation.w = math.cos(state.yaw / 2)
        original(msg)

    motion._odom_callback = deliver
    assert not aligner.align()
    assert 'rotation safety limit' in logs(motion, 'error')[-1]
    assert state.linear == state.angular == 0


def test_no_completion_from_scans_cached_during_odom_loss(rear_alignment):
    aligner, motion, state = rear_alignment
    interrupt_odom(motion, state, end=10.95)
    original = motion.lidar.snapshot
    motion.lidar.snapshot = lambda: (
        original() if state.now < 10.95
        else type(original())(sequence=9))
    motion.node.parameters['lidar_align_timeout_sec'] = 2.0

    assert not aligner.align()
    assert 'alignment timed out' in logs(motion, 'error')[-1]
    assert all(v == w == 0 for _, _, _, v, w in state.sent)


def test_odom_recovery_does_not_extend_alignment_deadline(rear_alignment):
    aligner, motion, state = rear_alignment
    motion.node.parameters['lidar_align_timeout_sec'] = 1.1
    interrupt_odom(motion, state, end=10.95)

    assert not aligner.align()
    assert 11.1 < state.now < 11.2
    assert 'alignment timed out' in logs(motion, 'error')[-1]


@pytest.mark.parametrize('value', [-.1, math.nan, math.inf, 18.1])
def test_invalid_recovery_budget_is_rejected(rear_alignment, value):
    aligner, motion, state = rear_alignment
    motion.node.parameters['lidar_align_odom_recovery_timeout_sec'] = value
    assert not aligner.align()
    assert not state.sent
