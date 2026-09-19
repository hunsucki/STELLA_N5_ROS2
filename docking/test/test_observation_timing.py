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


def test_refinement_rejects_zero_quaternion_and_normalizes_valid_rotation():
    assert math.isnan(DockTurnBackup._quaternion_yaw(0.0, 0.0, 0.0, 0.0))
    assert DockTurnBackup._quaternion_yaw(0.0, 0.0, 2.0, 2.0) == pytest.approx(math.pi / 2)


@pytest.mark.parametrize('scenario', [
    'duplicate_odom', 'coasting', 'settles', 'stale_target', 'zero_rotation',
])
def test_refinement_loop_requires_fresh_target_and_distinct_settled_odom(
        monkeypatch, scenario):
    import rclpy

    wall_time = [10.0]
    monkeypatch.setattr(time, 'monotonic', lambda: wall_time[0])
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    motion = make_motion()
    motion.node.parameters['tag_refinement_timeout_sec'] = 0.8
    motion._odom_callback(odom(10_000_000_000))
    motion.cmd_vel_pub = Mock()
    motion.stop_robot = Mock()

    node = DockTurnBackup.__new__(DockTurnBackup)
    node.motion = motion
    node.get_parameter = motion.node.get_parameter
    node.get_clock = motion.node.get_clock
    node.get_logger = Mock(return_value=Mock())
    node.should_stop = lambda: False
    target = PoseStamped()
    target.header.frame_id = 'odom'
    target.header.stamp = Time(seconds=8 if scenario == 'stale_target' else 10).to_msg()
    target.pose.orientation.w = 0.0 if scenario == 'zero_rotation' else 1.0
    node.last_refinement_target_pose = target
    node.last_refinement_target_received_at = wall_time[0]
    ticks = 0

    def spin_once(*args, **kwargs):
        nonlocal ticks
        ticks += 1
        wall_time[0] += 0.02
        motion.node.now_ns += 20_000_000
        stamp = 10_000_000_000 if scenario == 'duplicate_odom' else motion.node.now_ns
        sample = odom(stamp)
        if scenario == 'coasting' or (scenario == 'settles' and ticks <= 8):
            sample.twist.twist.linear.x = 0.02
        motion._odom_callback(sample)

    monkeypatch.setattr(rclpy, 'spin_once', spin_once)
    assert node._refine_tag_front_pose() == (scenario == 'settles')
    if scenario == 'settles':
        assert ticks >= 13  # Eight moving samples, then five stopped samples.
    for call in motion.cmd_vel_pub.publish.call_args_list:
        assert call.args[0].linear.x == 0.0
        assert call.args[0].angular.z == 0.0


@pytest.mark.parametrize('timeout,expected', [(18.0, False), (45.0, True)])
@pytest.mark.parametrize('mirror', [1.0, -1.0])
@pytest.mark.parametrize(
    'motor_model', [
        'lag', 'turn_dominant', 'stiction', 'wrong_direction', 'jam'])
@pytest.mark.parametrize('logged_error', [
    (0.122, 0.078, -13.67),  # September 9
    (0.096, 0.107, -22.11),  # September 15 11:26
    (0.113, 0.099, -18.74),  # September 15 14:47
    (0.086, 0.121, -26.18),  # September 15 15:42
])
def test_logged_refinement_with_slow_final_rotation(
        monkeypatch, timeout, expected, mirror, motor_model, logged_error):
    """Run the real loop from logged target-axis errors, with motor lag.

    The old test assumed perfect instantaneous actuators and a different pose.
    A 0.6 final-rotation response models the logged ~3 deg/s at 0.08 rad/s.
    """
    import rclpy

    wall_time = [10.0]
    monkeypatch.setattr(time, 'monotonic', lambda: wall_time[0])
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    motion = make_motion()
    motion.node.parameters['tag_refinement_timeout_sec'] = timeout
    motion._odom_callback(odom(10_000_000_000))
    command = [0.0, 0.0]
    command_history = []
    position = [0.0, 0.0, 0.0]
    measured = [0.0, 0.0]
    motion.imu_is_fresh = lambda: True
    motion.current_imu_yaw_rate = lambda: measured[1]
    traveled = [0.0]
    peak_yaw = [0.0]

    def publish(msg):
        command[:] = [msg.linear.x, msg.angular.z]
        command_history.append(tuple(command))

    motion.cmd_vel_pub = SimpleNamespace(publish=publish)
    motion.stop_robot = lambda: command.__setitem__(slice(None), [0.0, 0.0])
    node = DockTurnBackup.__new__(DockTurnBackup)
    node.motion = motion
    node.get_parameter = motion.node.get_parameter
    node.get_clock = motion.node.get_clock
    node.get_logger = Mock(return_value=Mock())
    node.should_stop = lambda: False
    target = PoseStamped()
    target.header.frame_id = 'odom'
    target.header.stamp = Time(seconds=10).to_msg()
    longitudinal, lateral, yaw_degrees = logged_error
    lateral *= mirror
    yaw = mirror * math.radians(yaw_degrees)
    target.pose.position.x = (
        math.cos(yaw) * longitudinal - math.sin(yaw) * lateral)
    target.pose.position.y = (
        math.sin(yaw) * longitudinal + math.cos(yaw) * lateral)
    target.pose.orientation.z = math.sin(yaw / 2)
    target.pose.orientation.w = math.cos(yaw / 2)
    node.last_refinement_target_pose = target
    node.last_refinement_target_received_at = wall_time[0]

    def spin_once(*args, **kwargs):
        dt = 0.05
        wall_time[0] += dt
        motion.node.now_ns += 50_000_000
        # Response differs between translation and stationary final alignment.
        angular_response = 0.6 if command[0] == 0.0 else 1.0
        requested = [command[0], command[1] * angular_response]
        if motor_model == 'turn_dominant' and command[0] != 0.0:
            requested = [command[0] * 0.70, command[1] * 1.25]
        if motor_model == 'wrong_direction':
            requested[1] *= -1.0
        if motor_model != 'lag' and command[0] == 0.0:
            if motor_model == 'jam' or abs(command[1]) < 0.10:
                requested[1] = 0.0
        for i in (0, 1):
            measured[i] += (requested[i] - measured[i]) * 0.2
        position[0] += measured[0] * math.cos(position[2]) * dt
        position[1] += measured[0] * math.sin(position[2]) * dt
        position[2] += measured[1] * dt
        traveled[0] += abs(measured[0]) * dt
        peak_yaw[0] = max(peak_yaw[0], abs(position[2]))
        sample = odom(motion.node.now_ns)
        sample.pose.pose.position.x, sample.pose.pose.position.y = position[:2]
        sample.pose.pose.orientation.z = math.sin(position[2] / 2)
        sample.pose.pose.orientation.w = math.cos(position[2] / 2)
        sample.twist.twist.linear.x, sample.twist.twist.angular.z = measured
        motion._odom_callback(sample)

    monkeypatch.setattr(rclpy, 'spin_once', spin_once)
    expected = expected and motor_model not in ('jam', 'wrong_direction')
    assert node._refine_tag_front_pose() == expected
    assert command == [0.0, 0.0]
    # Every recorded field run started with a point bearing above 8 degrees.
    # Translation must remain stopped until that bearing has been reduced.
    assert command_history[0][0] == 0.0
    assert math.copysign(1.0, command_history[0][1]) == mirror
    assert traveled[0] <= motion.node.parameters['tag_refinement_max_travel']
    assert peak_yaw[0] <= motion.node.parameters[
        'tag_refinement_max_yaw_excursion']
    if expected:
        errors = node._fixed_goal_errors_in_target(
            target.pose.position.x, target.pose.position.y, yaw, *position)
        assert abs(errors[0]) <= 0.04
        assert abs(errors[1]) <= 0.025
        assert abs(errors[2]) <= math.radians(2.0)
        assert motion.is_stationary(0.005)
    else:
        failure = node.get_logger().error.call_args.args[0]
        assert (
            'timed out' in failure
            or motor_model == 'jam'
            and 'bounded stationary recovery' in failure
            or motor_model == 'wrong_direction'
            and 'not converging despite measured rotation' in failure)


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
        assert ticks[0] >= 5
