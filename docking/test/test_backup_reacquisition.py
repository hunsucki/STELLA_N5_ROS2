"""Exercise the real backup loop through temporary and permanent plane loss."""

import math
from types import SimpleNamespace
from unittest.mock import Mock

from docking.motion import MotionController
from geometry_msgs.msg import Vector3Stamped
import pytest
import rclpy
from sensor_msgs.msg import Imu
from test_observation_timing import make_motion, odom


@pytest.mark.parametrize('permanent', [False, True])
@pytest.mark.parametrize('logged_disagreement', [False, True])
def test_plane_loss_stops_translation_then_recovers_or_fails(
        monkeypatch, permanent, logged_disagreement):
    wall = [10.0]
    monkeypatch.setattr('time.monotonic', lambda: wall[0])
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    motion = make_motion()
    motion.node.get_logger = Mock(return_value=Mock())
    motion.node.parameters['backup_target_rear_clearance'] = .0145
    motion.node.parameters['backup_plane_reacquire_timeout_sec'] = 3.0
    motion.should_stop = lambda: False
    motion.validate_lidar_configuration = lambda: True
    motion.backup_lidar_guide_estimator = None
    command = [0.0]
    position = [0.0]
    sent = []

    def publish(msg):
        command[0] = msg.linear.x
        sent.append((wall[0] - 10.0, msg.linear.x, msg.angular.z))

    motion.cmd_vel_pub = SimpleNamespace(publish=publish)
    motion.stop_robot = lambda: publish(MotionController.twist())
    motion.lidar = SimpleNamespace(sequence=0)
    motion.lidar.snapshot = lambda: SimpleNamespace(
        sequence=motion.lidar.sequence, scan=SimpleNamespace(angle_increment=.01))
    motion._rear_clearances = lambda _: [(i, .25 + position[0]) for i in range(12)]
    motion._rear_safety_clearances = motion._rear_clearances

    def disagreement():
        elapsed = wall[0] - 10.0
        return elapsed >= 2.0 and (permanent or elapsed <= 2.5)

    if logged_disagreement:
        # The 16:56 log reached +9.22deg gyro drift with approximately
        # -0.5deg wheel drift. An invalid plane must stop, then get the full
        # bounded recovery window instead of aborting on the first scan.
        motion.imu_is_fresh = lambda: True
        motion.current_integrated_imu_yaw = lambda: math.radians(
            9.22 if disagreement() else 0.0)
        motion.current_stationary_yaw_rate = lambda: (0.0, 'imu')

    def estimate(_):
        if disagreement():
            if logged_disagreement:
                return (math.radians(-4.34), 0.0, 21, 30, .3)
            return None
        return (math.radians(-.52), 0.0, 30, 30, .3)

    motion.backup_lidar_heading_estimator = estimate

    def sensor_tick(dt):
        wall[0] += dt
        position[0] += command[0] * dt
        motion.node.now_ns = int(wall[0] * 1e9)
        sample = odom(motion.node.now_ns)
        sample.pose.pose.position.x = position[0]
        sample.twist.twist.linear.x = command[0]
        if logged_disagreement and disagreement():
            sample.pose.pose.orientation.z = math.sin(math.radians(8.4) / 2)
            sample.pose.pose.orientation.w = math.cos(math.radians(8.4) / 2)
        motion._odom_callback(sample)
        yaw = Vector3Stamped()
        yaw.header.stamp = sample.header.stamp
        if logged_disagreement and disagreement():
            yaw.vector.x = math.radians(-.5)
        motion._wheel_yaw_callback(yaw)
        motion.lidar.sequence = int((wall[0] - 10.0) * 10)

    sensor_tick(0.0)
    monkeypatch.setattr(rclpy, 'spin_once', lambda node, timeout_sec: sensor_tick(min(timeout_sec, .01)))
    assert motion.backup_with_lidar() is not permanent
    assert any(v < 0 for t, v, _ in sent if t < 2.0)
    assert all(v == 0 and w == 0 for t, v, w in sent if 2.12 < t < 2.5)
    assert command[0] == 0.0
    if permanent:
        assert 15.0 <= wall[0] < 15.5
        assert all(v == 0 and w == 0 for t, v, w in sent if t > 2.12)
        assert 'did not recover' in motion.node.get_logger().error.call_args.args[0]
    else:
        assert any(v < 0 for t, v, _ in sent if t > 2.8)
        assert .0045 <= .25 + position[0] <= .0245


def test_correlated_imu_odom_bias_does_not_stop_straight_backup(monkeypatch):
    wall = [20.0]
    monkeypatch.setattr('time.monotonic', lambda: wall[0])
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    motion = make_motion()
    logger = Mock()
    motion.node.get_logger = Mock(return_value=logger)
    motion.node.parameters['backup_target_rear_clearance'] = .0145
    motion.should_stop = lambda: False
    motion.validate_lidar_configuration = lambda: True
    motion.backup_lidar_guide_estimator = None
    command = [MotionController.twist()]
    position = [0.0]

    motion.cmd_vel_pub = SimpleNamespace(
        publish=lambda msg: command.__setitem__(0, msg))
    motion.stop_robot = lambda: command.__setitem__(0, MotionController.twist())
    motion.lidar = SimpleNamespace(sequence=0)
    motion.lidar.snapshot = lambda: SimpleNamespace(
        sequence=motion.lidar.sequence,
        scan=SimpleNamespace(angle_increment=.01))
    motion._rear_clearances = lambda _: [
        (i, .20 + position[0]) for i in range(12)]
    motion._rear_safety_clearances = motion._rear_clearances

    def drift_fraction():
        return min(max((wall[0] - 20.4) / 2.0, 0.0), 1.0)

    def estimate(_):
        candidate = math.radians(.67 + .65 * drift_fraction())
        return (candidate, 0.0, 30, 30, .3)

    motion.backup_lidar_heading_estimator = estimate

    def sensor_tick(dt):
        wall[0] += dt
        position[0] += command[0].linear.x * dt
        motion.node.now_ns = int(wall[0] * 1e9)
        fraction = drift_fraction()
        odom_yaw = math.radians(1.44 * fraction)
        sample = odom(motion.node.now_ns)
        sample.pose.pose.position.x = position[0]
        sample.pose.pose.orientation.z = math.sin(odom_yaw / 2.0)
        sample.pose.pose.orientation.w = math.cos(odom_yaw / 2.0)
        sample.twist.twist.linear.x = command[0].linear.x
        motion._odom_callback(sample)

        wheel = Vector3Stamped()
        wheel.header.stamp = sample.header.stamp
        wheel.vector.x = math.radians(-.05 * fraction)
        motion._wheel_yaw_callback(wheel)

        imu = Imu()
        imu.header.stamp = sample.header.stamp
        imu.angular_velocity.z = 0.0
        motion._imu_callback(imu)
        motion.integrated_imu_yaw = math.radians(1.52 * fraction)
        motion.lidar.sequence = int((wall[0] - 20.0) * 10)

    sensor_tick(0.0)
    monkeypatch.setattr(
        rclpy, 'spin_once',
        lambda node, timeout_sec: sensor_tick(min(timeout_sec, .01)))

    assert motion.backup_with_lidar()
    warnings = ' '.join(call.args[0] for call in logger.warning.call_args_list)
    assert 'inconsistent with robot motion' not in warnings
    assert .0045 <= .20 + position[0] <= .0245


def test_latest_stable_plane_bias_uses_bounded_wheel_heading_guard(monkeypatch):
    wall = [30.0]
    monkeypatch.setattr('time.monotonic', lambda: wall[0])
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    motion = make_motion()
    logger = Mock()
    motion.node.get_logger = Mock(return_value=logger)
    motion.should_stop = lambda: False
    motion.validate_lidar_configuration = lambda: True
    motion.backup_lidar_guide_estimator = None
    command = [MotionController.twist()]
    position = [0.0]
    sent = []

    def publish(msg):
        command[0] = msg
        sent.append((wall[0] - 30.0, msg.linear.x, msg.angular.z))

    motion.cmd_vel_pub = SimpleNamespace(publish=publish)
    motion.stop_robot = lambda: publish(MotionController.twist())
    motion.lidar = SimpleNamespace(sequence=0)
    motion.lidar.snapshot = lambda: SimpleNamespace(
        sequence=motion.lidar.sequence,
        scan=SimpleNamespace(angle_increment=.01))
    motion._rear_clearances = lambda _: [
        (i, .20 + position[0]) for i in range(12)]
    motion._rear_safety_clearances = motion._rear_clearances

    def after_bias():
        return wall[0] - 30.0 >= .8

    motion.backup_lidar_heading_estimator = lambda _: (
        math.radians(2.90 if after_bias() else .75),
        0.0, 96, 114, .60)

    def sensor_tick(dt):
        wall[0] += dt
        position[0] += command[0].linear.x * dt
        motion.node.now_ns = int(wall[0] * 1e9)
        imu_odom_yaw = math.radians(2.10 if after_bias() else 0.0)
        wheel_yaw = math.radians(.13 if after_bias() else 0.0)
        sample = odom(motion.node.now_ns)
        sample.pose.pose.position.x = position[0]
        sample.pose.pose.orientation.z = math.sin(imu_odom_yaw / 2.0)
        sample.pose.pose.orientation.w = math.cos(imu_odom_yaw / 2.0)
        sample.twist.twist.linear.x = command[0].linear.x
        motion._odom_callback(sample)

        wheel = Vector3Stamped()
        wheel.header.stamp = sample.header.stamp
        wheel.vector.x = wheel_yaw
        motion._wheel_yaw_callback(wheel)

        imu = Imu()
        imu.header.stamp = sample.header.stamp
        imu.angular_velocity.z = 0.0
        motion._imu_callback(imu)
        motion.integrated_imu_yaw = imu_odom_yaw
        motion.lidar.sequence = int((wall[0] - 30.0) * 10)

    sensor_tick(0.0)
    monkeypatch.setattr(
        rclpy, 'spin_once',
        lambda node, timeout_sec: sensor_tick(min(timeout_sec, .01)))

    assert motion.backup_with_lidar()
    warnings = ' '.join(call.args[0] for call in logger.warning.call_args_list)
    assert 'encoder-only heading guard' in warnings
    assert not logger.error.called
    assert any(v == 0.0 for t, v, _ in sent if .80 < t < 1.20)
    assert any(v < 0.0 for t, v, _ in sent if t > 1.20)
    assert .0045 <= .20 + position[0] <= .0245


@pytest.mark.parametrize('response', [1.0, -1.0, 0.0])
@pytest.mark.parametrize('guard_interruption', [False, True])
def test_stationary_recovery_aligns_before_reversing(
        monkeypatch, response, guard_interruption):
    wall = [30.0]
    monkeypatch.setattr('time.monotonic', lambda: wall[0])
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    motion = make_motion()
    logger = Mock()
    motion.node.get_logger = Mock(return_value=logger)
    motion.should_stop = lambda: False
    motion.validate_lidar_configuration = lambda: True
    motion.backup_lidar_guide_estimator = None
    command = [MotionController.twist()]
    position = [0.0]
    actual_yaw = [0.0]
    sent = []
    interruption_at = [None]

    def publish(msg):
        command[0] = msg
        sent.append((wall[0] - 30.0, msg.linear.x, msg.angular.z))

    motion.cmd_vel_pub = SimpleNamespace(publish=publish)
    motion.stop_robot = lambda: publish(MotionController.twist())
    motion.lidar = SimpleNamespace(sequence=0)
    motion.lidar.snapshot = lambda: SimpleNamespace(
        sequence=motion.lidar.sequence,
        scan=SimpleNamespace(angle_increment=.01))
    motion._rear_clearances = lambda _: [
        (i, .20 + position[0]) for i in range(12)]
    motion._rear_safety_clearances = motion._rear_clearances

    def after_bias():
        return wall[0] - 30.0 >= .8

    def estimate(_):
        if (guard_interruption and interruption_at[0] is None
                and command[0].angular.z < -.005):
            interruption_at[0] = wall[0]
        if (interruption_at[0] is not None
                and wall[0] - interruption_at[0] < .4):
            # Three plausible wheel-guard scans must not cancel an unfinished
            # correction and resume reversing toward a still tilted panel.
            return (math.radians(-.8), 0.0, 105, 115, .52)
        return (math.radians(-4.34 if after_bias() else -.52)
                - actual_yaw[0], 0.0, 105, 115, .52)

    motion.backup_lidar_heading_estimator = estimate

    def sensor_tick(dt):
        wall[0] += dt
        position[0] += command[0].linear.x * dt
        actual_yaw[0] += response * command[0].angular.z * dt
        motion.node.now_ns = int(wall[0] * 1e9)
        imu_odom_yaw = math.radians(9.22 if after_bias() else 0.0) + actual_yaw[0]
        wheel_yaw = math.radians(-.50 if after_bias() else 0.0) + actual_yaw[0]
        sample = odom(motion.node.now_ns)
        sample.pose.pose.position.x = position[0]
        sample.pose.pose.orientation.z = math.sin(imu_odom_yaw / 2.0)
        sample.pose.pose.orientation.w = math.cos(imu_odom_yaw / 2.0)
        sample.twist.twist.linear.x = command[0].linear.x
        sample.twist.twist.angular.z = response * command[0].angular.z
        motion._odom_callback(sample)

        wheel = Vector3Stamped()
        wheel.header.stamp = sample.header.stamp
        wheel.vector.x = wheel_yaw
        motion._wheel_yaw_callback(wheel)

        imu = Imu()
        imu.header.stamp = sample.header.stamp
        imu.angular_velocity.z = response * command[0].angular.z
        motion._imu_callback(imu)
        motion.integrated_imu_yaw = imu_odom_yaw
        motion.lidar.sequence = int((wall[0] - 30.0) * 10)

    sensor_tick(0.0)
    monkeypatch.setattr(
        rclpy, 'spin_once',
        lambda node, timeout_sec: sensor_tick(min(timeout_sec, .01)))

    assert motion.backup_with_lidar() is (response == 1.0)
    warnings = ' '.join(call.args[0] for call in logger.warning.call_args_list)
    assert 'Stationary rear plane recovered; aligning in place' in warnings
    assert warnings.count('Stationary rear plane recovered') == 1
    turning = [(t, v, w) for t, v, w in sent if t > .8 and abs(w) > 0]
    assert turning
    if guard_interruption:
        assert interruption_at[0] is not None
        assert all(v == 0 for t, v, w in sent
                   if interruption_at[0] - 30.0 <= t <
                   interruption_at[0] - 30.0 + .4)
    assert all(v == 0 for t, v, w in turning)
    if response == 1.0:
        assert abs(math.radians(-4.34) - actual_yaw[0]) <= math.radians(1.0)
        assert any(v < 0 for t, v, w in sent if t > turning[-1][0])
        assert .0045 <= .20 + position[0] <= .0245
        assert not logger.error.called
    else:
        assert all(v == 0 for t, v, w in sent if t > 1.0)
    assert command[0].linear.x == 0
    assert command[0].angular.z == 0


def test_two_verified_corrections_do_not_consume_straight_drift_budget(monkeypatch):
    response = 1.0
    wall = [30.0]
    monkeypatch.setattr('time.monotonic', lambda: wall[0])
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    motion = make_motion()
    logger = Mock()
    motion.node.get_logger = Mock(return_value=logger)
    motion.should_stop = lambda: False
    motion.validate_lidar_configuration = lambda: True
    motion.backup_lidar_guide_estimator = None
    command = [MotionController.twist()]
    position = [0.0]
    actual_yaw = [0.0]
    sent = []
    slip = [0.0]
    dropout_at = [None]

    def publish(msg):
        command[0] = msg
        sent.append((wall[0] - 30.0, msg.linear.x, msg.angular.z))

    motion.cmd_vel_pub = SimpleNamespace(publish=publish)
    motion.stop_robot = lambda: publish(MotionController.twist())
    motion.lidar = SimpleNamespace(sequence=0)
    motion.lidar.snapshot = lambda: SimpleNamespace(
        sequence=motion.lidar.sequence,
        scan=SimpleNamespace(angle_increment=.01))
    motion._rear_clearances = lambda _: [
        (i, .30 + position[0]) for i in range(12)]
    motion._rear_safety_clearances = motion._rear_clearances

    def after_bias():
        return wall[0] - 30.0 >= .8

    def estimate(_):
        settled = sum('correction settled' in c.args[0]
                      for c in logger.info.call_args_list)
        if settled >= 2:
            if dropout_at[0] is None:
                dropout_at[0] = wall[0]
            if wall[0] - dropout_at[0] < .3:
                return None
        return (math.radians(-4.34 if after_bias() else -.52)
                - actual_yaw[0], 0.0, 105, 115, .52)

    motion.backup_lidar_heading_estimator = estimate

    def sensor_tick(dt):
        wall[0] += dt
        position[0] += command[0].linear.x * dt
        actual_yaw[0] += response * command[0].angular.z * dt
        if position[0] < -.08 and slip[0] == 0.0:
            # Chassis rotation measured by LiDAR and IMU, but not wheels.
            slip[0] = math.radians(5.0)
            actual_yaw[0] += slip[0]
        motion.node.now_ns = int(wall[0] * 1e9)
        imu_odom_yaw = math.radians(9.22 if after_bias() else 0.0) + actual_yaw[0]
        wheel_yaw = (math.radians(-.50 if after_bias() else 0.0)
                     + actual_yaw[0] - slip[0])
        sample = odom(motion.node.now_ns)
        sample.pose.pose.position.x = position[0]
        sample.pose.pose.orientation.z = math.sin(imu_odom_yaw / 2.0)
        sample.pose.pose.orientation.w = math.cos(imu_odom_yaw / 2.0)
        sample.twist.twist.linear.x = command[0].linear.x
        sample.twist.twist.angular.z = response * command[0].angular.z
        motion._odom_callback(sample)

        wheel = Vector3Stamped()
        wheel.header.stamp = sample.header.stamp
        wheel.vector.x = wheel_yaw
        motion._wheel_yaw_callback(wheel)

        imu = Imu()
        imu.header.stamp = sample.header.stamp
        imu.angular_velocity.z = response * command[0].angular.z
        motion._imu_callback(imu)
        motion.integrated_imu_yaw = imu_odom_yaw
        motion.lidar.sequence = int((wall[0] - 30.0) * 10)

    sensor_tick(0.0)
    monkeypatch.setattr(
        rclpy, 'spin_once',
        lambda node, timeout_sec: sensor_tick(min(timeout_sec, .01)))

    assert motion.backup_with_lidar()
    assert dropout_at[0] is not None
    # More than eight degrees of commanded correction over two episodes is
    # legitimate; drift is measured from the last physically verified yaw.
    assert math.degrees(motion.current_wheel_yaw()) < -8.0
    assert all(v == 0 and w == 0 for t, v, w in sent
               if dropout_at[0] - 30.0 + .12 < t < dropout_at[0] - 30.0 + .3)
    assert not logger.error.called
    assert .0045 <= .30 + position[0] <= .0245
    assert command[0].linear.x == 0
