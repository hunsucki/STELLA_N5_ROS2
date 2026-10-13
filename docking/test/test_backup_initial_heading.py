"""Replay the 8.99-degree panel rejection after a completed odom spin."""

import math
from pathlib import Path

import pytest
import yaml

from test_backup_entry_completion import simulation


def configure(motion, state, degrees):
    config = yaml.safe_load((Path(__file__).parents[1] / 'config/docking.yaml').read_text())
    motion.node.parameters.update(config['dock_turn_backup']['ros__parameters'])
    state.gap, state.yaw = .48, math.radians(degrees)
    # Replay the measured panel support rather than perfect synthetic support.
    motion.backup_lidar_heading_estimator = lambda _: (
        -state.yaw, math.pi / 2 - state.yaw, 89, 115, .60)
    sync_initial_pose(motion, state)


def sync_initial_pose(motion, state):
    # Set the already-observed post-spin pose before the backup stage takes
    # its yaw anchors; this is an initial tilt, not a sudden wheel jump.
    motion.last_odom.pose.pose.orientation.z = math.sin(state.yaw / 2)
    motion.last_odom.pose.pose.orientation.w = math.cos(state.yaw / 2)
    motion.last_wheel_yaw = state.yaw


@pytest.mark.parametrize('degrees', [-11.9, -8.99, 8.99, 11.9])
@pytest.mark.parametrize('legacy_limit', [False, True])
def test_initial_panel_angle_is_corrected_before_reverse(simulation, degrees, legacy_limit):
    motion, state = simulation
    configure(motion, state, degrees)
    if legacy_limit:
        motion.node.parameters['backup_lidar_heading_max_error'] = math.radians(8)
    headings_at_reverse = []
    publish = motion.cmd_vel_pub.publish

    def observe_command(msg):
        if msg.linear.x < 0:
            headings_at_reverse.append(state.yaw)
        publish(msg)

    motion.cmd_vel_pub.publish = observe_command
    assert motion.backup_with_lidar() is (not legacy_limit)
    if legacy_limit:
        assert all(row[3] == row[4] == 0 for row in state.sent)
        assert 'heading_out_of_range' in motion.node.get_logger().error.call_args.args[0]
        return
    reverse = [row for row in state.sent if row[3] < 0]
    assert reverse
    assert abs(headings_at_reverse[0]) <= math.radians(1)
    assert abs(state.yaw) < math.radians(2)
    # In-place correction leaves x exactly zero. Never reverse with the
    # initial tilt; complete the stationary settling phase first.
    correction = [row for row in state.sent if row[4] != 0 and row[1] == 0]
    assert correction and all(row[3] == 0 for row in correction)
    assert reverse[0][0] > correction[-1][0]
    assert all(abs(row[4]) <= .060001 for row in state.sent)
    assert .022 <= state.gap + state.x <= .034
    assert state.linear == state.angular == 0


@pytest.mark.parametrize('fault', [
    'angle', 'weak_support', 'short_line', 'imu_loss',
    'no_chassis_response', 'no_guides', 'unsafe_side',
])
def test_correctable_angle_does_not_bypass_other_guards(simulation, fault):
    motion, state = simulation
    configure(motion, state, 8.99)
    if fault == 'angle':
        state.yaw = math.radians(12.2)
        sync_initial_pose(motion, state)
    if fault in ('weak_support', 'short_line'):
        motion.backup_lidar_heading_estimator = lambda _: (
            -state.yaw, 0., 60 if fault == 'weak_support' else 89,
            115, .10 if fault == 'short_line' else .60)
    if fault == 'imu_loss':
        motion.imu_is_fresh = lambda: False
    if fault == 'no_chassis_response':
        state.yaw_response = lambda _: 0.
    if fault == 'no_guides':
        motion.backup_lidar_guide_estimator = lambda _: None
    if fault == 'unsafe_side':
        state.offset = .04
    assert not motion.backup_with_lidar()
    assert all(row[3] == 0 for row in state.sent)
    assert state.linear == state.angular == 0
