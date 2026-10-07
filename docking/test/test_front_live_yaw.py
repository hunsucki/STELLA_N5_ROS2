"""Use physical panel feedback when fused odom has the opposite yaw error."""
import math
from types import SimpleNamespace

from docking.tag_centering import LidarTagCentering
from geometry_msgs.msg import PoseStamped
import pytest
from test_backup_entry_completion import simulation  # noqa: F401


@pytest.mark.parametrize('direction', [-1, 1])
@pytest.mark.parametrize('fault', [None, 'stale_imu', 'lost_panel', 'stalled',
                                  'lateral_drift', 'timing_gap', 'timing_lost',
                                  'tag_gap', 'tag_lost', 'scan_gap', 'scan_lost'])
@pytest.mark.parametrize('initial_profile', [False, True])
def test_live_front_yaw_uses_panel_with_bounded_rotation(
        simulation, direction, fault, initial_profile):
    motion, state = simulation
    initial = direction * math.radians(16.22 if initial_profile else 2.5)
    state.yaw = initial
    node = motion.node
    node.motion = motion
    node.should_stop = lambda: False
    node._pose_is_fresh = lambda *args: not (
        (fault == 'tag_gap' and 10.4 <= state.now < 10.65)
        or (fault == 'tag_lost' and state.now >= 10.4))
    node.last_dock_pose_received_at = state.now
    # Fused heading reports the wrong direction. The correction must never
    # steer from it; gyro, wheel and panel remain independent observations.
    motion.current_yaw = lambda: -state.yaw
    helper = LidarTagCentering.__new__(LidarTagCentering)
    helper.node = node
    helper.panel = lambda *args: None if fault == 'lost_panel' else -state.yaw

    def snapshot():
        tag = PoseStamped()
        tag.header.frame_id = 'base_link'
        tag.header.stamp.sec = int(state.now)
        tag.header.stamp.nanosec = int((state.now - int(state.now))*1e9)
        lateral = direction*.015 + .2*(state.yaw-initial)
        if fault == 'lateral_drift':
            lateral = direction*(.079 if initial_profile else .024) - .2*(state.yaw-initial)
        angle = -state.yaw
        tag.pose.position.x = math.cos(angle) - math.sin(angle)*lateral
        tag.pose.position.y = math.sin(angle) + math.cos(angle)*lateral
        node.last_dock_pose = tag
        scan_header = SimpleNamespace(stamp=SimpleNamespace(
            sec=tag.header.stamp.sec, nanosec=tag.header.stamp.nanosec))
        if ((fault == 'timing_gap' and 10.4 <= state.now < 10.65)
                or (fault == 'timing_lost' and state.now >= 10.4)):
            scan_header.stamp.sec -= 1
        if ((fault == 'scan_gap' and 10.4 <= state.now < 10.65)
                or (fault == 'scan_lost' and state.now >= 10.4)):
            return None
        return SimpleNamespace(sequence=int(state.now*10),
                               scan=SimpleNamespace(header=scan_header))

    helper.lidar = SimpleNamespace(snapshot=snapshot)
    if fault == 'stale_imu':
        motion.imu_is_fresh = lambda: False
    if fault == 'stalled':
        state.yaw_response = lambda _: 0
    else:
        state.yaw_response = lambda _: 1.0 if initial_profile else .7
    snapshot()
    success = helper._correct_live_heading(initial=initial_profile)
    assert success is (fault in (None, 'timing_gap', 'tag_gap', 'scan_gap'))
    assert state.now - 10 < (8.2 if initial_profile else 6.2)
    assert all(v == 0 and abs(w) <= (.06 if initial_profile else .04)
               for _, _, _, v, w in state.sent)
    commands = [w for _, _, _, _, w in state.sent if w]
    assert all(w * direction < 0 for w in commands)
    assert state.linear == state.angular == 0
    if success:
        assert abs(state.yaw) <= math.radians(.75)
    if fault in ('stale_imu', 'lost_panel'):
        assert not commands
    if fault in ('timing_gap', 'timing_lost', 'tag_gap', 'tag_lost', 'scan_gap', 'scan_lost'):
        during_gap = [w for t, _, _, _, w in state.sent
                      if 10.4 <= t < (10.65 if fault.endswith('_gap') else 100)]
        assert during_gap and all(w == 0 for w in during_gap)
    if fault in ('timing_lost', 'tag_lost', 'scan_lost'):
        assert state.now < 11.1
