"""Regressions for stale observations and premature docking completion."""

import math
import time
from types import SimpleNamespace
from unittest.mock import Mock

from docking.apriltag_bridge import AprilTagToPoseBridge
from docking.dock_turn_backup import DockTurnBackup
from docking.motion import MotionController
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped
from nav_msgs.msg import Odometry
import pytest
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros.buffer import Buffer


class FakeNode:
    def __init__(self):
        self.now_ns = 10_000_000_000
        self.tf_buffer = Buffer()
        self.parameters = {'fixed_frame': 'odom', 'base_frame': 'base_link'}
        MotionController.declare_parameters(self)
        DockTurnBackup._declare_docking_parameters(self)

    def declare_parameter(self, name, default):
        self.parameters[name] = default

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def get_clock(self):
        return SimpleNamespace(now=lambda: Time(nanoseconds=self.now_ns))


def make_motion():
    motion = MotionController.__new__(MotionController)
    motion.node = FakeNode()
    motion.last_odom = None
    motion.last_odom_received_at = 0.0
    motion.last_odom_stamp_nanoseconds = 0
    motion.odom_sequence = 0
    motion.last_imu_yaw_rate = None
    motion.last_imu_received_at = 0.0
    motion.last_imu_stamp_nanoseconds = 0
    motion.integrated_imu_yaw = 0.0
    motion.last_wheel_yaw = None
    motion.last_wheel_yaw_received_at = 0.0
    motion.last_wheel_yaw_stamp_nanoseconds = 0
    return motion


def odom(stamp_ns=9_900_000_000):
    msg = Odometry()
    msg.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'
    msg.pose.pose.orientation.w = 1.0
    return msg


def make_bridge():
    clock = FakeNode()
    bridge = SimpleNamespace(
        tf_buffer=Buffer(), source_frame='base_link', target_frame='tag36h11:0',
        transform_max_age_sec=0.5, transform_future_tolerance_sec=0.05,
        last_published_stamp_nanoseconds=0, publisher_=Mock(),
        get_clock=clock.get_clock,
    )
    return bridge, clock


def tag_transform(stamp_ns):
    transform = TransformStamped()
    transform.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
    transform.header.frame_id = 'base_link'
    transform.child_frame_id = 'tag36h11:0'
    transform.transform.translation.x = 1.0
    transform.transform.rotation.w = 1.0
    return transform


def test_tf_cache_is_not_republished_as_new_tag_detections():
    bridge, clock = make_bridge()
    transform = tag_transform(9_900_000_000)
    bridge.tf_buffer.set_transform(transform, 'test')
    AprilTagToPoseBridge.on_timer(bridge)
    message = bridge.publisher_.publish.call_args.args[0]
    assert message.header.stamp == transform.header.stamp
    assert message.pose.position.x == 1.0

    # TF still returns this transform after the camera loses the tag.
    for _ in range(20):
        clock.now_ns += 100_000_000
        AprilTagToPoseBridge.on_timer(bridge)
    bridge.publisher_.publish.assert_called_once()

    bridge.tf_buffer.set_transform(tag_transform(clock.now_ns), 'test')
    AprilTagToPoseBridge.on_timer(bridge)
    assert bridge.publisher_.publish.call_count == 2


@pytest.mark.parametrize('stamp_ns', [0, 9_400_000_000, 10_100_000_000])
def test_bridge_rejects_zero_stale_and_future_transforms(stamp_ns):
    bridge, _ = make_bridge()
    bridge.tf_buffer.set_transform(tag_transform(stamp_ns), 'test')
    AprilTagToPoseBridge.on_timer(bridge)
    bridge.publisher_.publish.assert_not_called()


def test_original_tag_stamp_uses_matching_robot_pose_in_tf():
    bridge, _ = make_bridge()
    bridge.tf_buffer.set_transform(tag_transform(9_950_000_000), 'test')
    for stamp, x in [(9_900_000_000, 0.0), (10_000_000_000, 0.01)]:
        transform = tag_transform(stamp)
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = x
        bridge.tf_buffer.set_transform(transform, 'test')
    AprilTagToPoseBridge.on_timer(bridge)
    message = bridge.publisher_.publish.call_args.args[0]
    robot = bridge.tf_buffer.lookup_transform(
        'odom', 'base_link', Time.from_msg(message.header.stamp))
    assert robot.transform.translation.x + message.pose.position.x == pytest.approx(1.005)


def test_repeated_odom_does_not_refresh_age_or_complete_spin():
    motion = make_motion()
    motion._odom_callback(odom())
    received_at = motion.last_odom_received_at
    done = motion._make_absolute_spin_done_cb(0.0, 0.01, 3, 1.0, 0.01)
    for _ in range(20):
        motion._odom_callback(odom())
        assert not done()
    assert motion.last_odom_received_at == received_at
    assert motion.odom_sequence == 1
    for index in range(1, 4):
        motion._odom_callback(odom(9_900_000_000 + index * 10_000_000))
        assert done() == (index == 3)


@pytest.mark.parametrize('invalid', [
    'stale', 'future', 'zero_stamp', 'wrong_parent', 'wrong_child',
    'nan_position', 'zero_quaternion', 'nan_velocity',
])
def test_invalid_odometry_is_not_used_for_precision_control(invalid):
    motion = make_motion()
    msg = odom()
    if invalid in ('stale', 'future', 'zero_stamp'):
        msg.header.stamp = Time(nanoseconds={
            'stale': 9_000_000_000, 'future': 10_100_000_000, 'zero_stamp': 0,
        }[invalid]).to_msg()
    elif invalid == 'wrong_parent':
        msg.header.frame_id = 'map'
    elif invalid == 'wrong_child':
        msg.child_frame_id = 'camera_link'
    elif invalid == 'nan_position':
        msg.pose.pose.position.x = math.nan
    elif invalid == 'zero_quaternion':
        msg.pose.pose.orientation.w = 0.0
    elif invalid == 'nan_velocity':
        msg.twist.twist.linear.x = math.nan
    motion._odom_callback(msg)
    assert not motion.odom_is_fresh()
    assert motion.odom_sequence == 0


def test_odom_age_includes_sensor_transport_delay():
    motion = make_motion()
    motion._odom_callback(odom(9_510_000_000))
    assert motion.odom_is_fresh()
    motion.node.now_ns += 20_000_000
    assert not motion.odom_is_fresh()


@pytest.mark.parametrize('x_offset,accepted', [(0.0, True), (0.05, False)])
def test_odom_footprint_is_accepted_only_when_tf_confirms_same_planar_origin(
        x_offset, accepted):
    motion = make_motion()
    joint = TransformStamped()
    joint.header.frame_id = 'base_footprint'
    joint.child_frame_id = 'base_link'
    joint.transform.translation.x = x_offset
    joint.transform.translation.z = 0.071
    joint.transform.rotation.w = 1.0
    motion.node.tf_buffer.set_transform_static(joint, 'test')
    msg = odom()
    msg.child_frame_id = 'base_footprint'
    motion._odom_callback(msg)
    assert motion.odom_is_fresh() == accepted


@pytest.mark.parametrize('kind', ['imu', 'wheel'])
def test_duplicate_and_out_of_order_heading_samples_do_not_refresh_state(kind):
    motion = make_motion()
    msg = Imu() if kind == 'imu' else Vector3Stamped()
    callback = motion._imu_callback if kind == 'imu' else motion._wheel_yaw_callback
    age_field = 'last_imu_received_at' if kind == 'imu' else 'last_wheel_yaw_received_at'
    msg.header.stamp = Time(nanoseconds=9_900_000_000).to_msg()
    callback(msg)
    received_at = getattr(motion, age_field)
    callback(msg)
    msg.header.stamp = Time(nanoseconds=9_800_000_000).to_msg()
    callback(msg)
    assert getattr(motion, age_field) == received_at
    assert motion.integrated_imu_yaw == 0.0


def test_recent_delivery_does_not_make_old_refinement_target_fresh():
    node = FakeNode()
    pose = PoseStamped()
    pose.header.stamp = Time(nanoseconds=8_000_000_000).to_msg()
    assert not DockTurnBackup._pose_is_fresh(node, pose, time.monotonic(), 1.5)
    pose.header.stamp = Time(nanoseconds=9_900_000_000).to_msg()
    assert DockTurnBackup._pose_is_fresh(node, pose, time.monotonic(), 1.5)


@pytest.mark.parametrize('linear,angular,stationary', [
    (0.02, 0.0, False), (0.0, 0.02, False), (0.0, 0.0, True),
])
def test_refinement_stationary_gate_checks_translation_and_rotation(
        linear, angular, stationary):
    motion = make_motion()
    msg = odom()
    msg.twist.twist.linear.x = linear
    msg.twist.twist.angular.z = angular
    motion._odom_callback(msg)
    assert motion.is_stationary(0.005) == stationary








@pytest.mark.parametrize('delivery,expected', [('continuous', True), ('single', False), ('gaps', False)])
def test_approach_requires_continuous_distinct_detections(monkeypatch, delivery, expected):
    import rclpy

    clock = FakeNode()
    clock.parameters['dock_pose_wait_timeout_sec'] = 2.0
    node = DockTurnBackup.__new__(DockTurnBackup)
    node.get_parameter = clock.get_parameter
    node.get_clock = clock.get_clock
    node.get_logger = Mock(return_value=Mock())
    node.stack = SimpleNamespace(detected_pose_topic='/test/detected_dock_pose')
    node.should_stop = lambda: False
    node.last_dock_pose = None
    node.last_dock_pose_received_at = 0.0
    wall = [10.0]
    ticks = [0]
    monkeypatch.setattr(time, 'monotonic', lambda: wall[0])
    monkeypatch.setattr(rclpy, 'ok', lambda: True)

    def spin_once(*args, **kwargs):
        ticks[0] += 1
        wall[0] += 0.1
        clock.now_ns += 100_000_000
        if delivery == 'continuous' or ticks[0] == 1 or (delivery == 'gaps' and ticks[0] % 6 == 0):
            pose = PoseStamped()
            pose.header.stamp = Time(nanoseconds=clock.now_ns).to_msg()
            node.last_dock_pose = pose
            node.last_dock_pose_received_at = wall[0]

    monkeypatch.setattr(rclpy, 'spin_once', spin_once)
    assert node._wait_for_detected_dock_pose() == expected
    if expected:
        assert ticks[0] >= 3
