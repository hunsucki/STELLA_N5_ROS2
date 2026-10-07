"""Run the actual refinement loop with the logged pose error and slow yaw response."""

import math
from types import SimpleNamespace
from unittest.mock import Mock

from docking.dock_turn_backup import DockTurnBackup
from geometry_msgs.msg import PoseStamped, Vector3Stamped
import pytest
import rclpy
from sensor_msgs.msg import Imu
from test_observation_timing import make_motion, odom


@pytest.mark.parametrize('side', [-1.0, 1.0])
@pytest.mark.parametrize('front_verified', [True, False])
@pytest.mark.parametrize('near_goal', [False, True, 'short_lateral'])
@pytest.mark.parametrize('initial_panel_deg', [0, 16.22])
@pytest.mark.parametrize('wheel_only_below_threshold', [False, True])
def test_one_centimeter_lateral_refinement_finishes_within_existing_limits(
        monkeypatch, side, front_verified, near_goal, initial_panel_deg,
        wheel_only_below_threshold, early_handoff_lateral=None, live_control=False,
        observation_gap=False, stale_feedback=None, apriltag_control=False,
        handoff_longitudinal=.30, target_heading_degrees=2., yaw_gain=.45,
        camera_limit_degrees=None, camera_heading_noise_degrees=0., drive_pivot_x=0.,
        estimated_drive_pivot_x=None):
    state = SimpleNamespace(now=10.0, x=0.0, y=0.0, yaw=0.0, v=0.0, w=0.0, travel=0.0)
    monkeypatch.setattr('time.monotonic', lambda: state.now)
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    motion = make_motion()
    node = DockTurnBackup.__new__(DockTurnBackup)
    node.motion = motion
    node.get_parameter = motion.node.get_parameter
    node.get_clock = motion.node.get_clock
    logger = Mock()
    node.get_logger = lambda: logger
    node.should_stop = lambda: False
    target = PoseStamped()
    target.header.frame_id = 'odom'
    target.header.stamp.sec = 10
    target_yaw = side * math.radians(-14.14)
    c, s = math.cos(target_yaw), math.sin(target_yaw)
    target.pose.position.x = c * .120 - s * side * .078
    target.pose.position.y = s * .120 + c * side * .078
    target.pose.orientation.z = math.sin(target_yaw / 2)
    target.pose.orientation.w = math.cos(target_yaw / 2)
    if near_goal:
        target_yaw = side * math.radians(2.15)
        c, s = math.cos(target_yaw), math.sin(target_yaw)
        target.pose.position.x = c * .009 - s * side * .008
        target.pose.position.y = s * .009 + c * side * .008
        target.pose.orientation.z = math.sin(target_yaw / 2)
        target.pose.orientation.w = math.cos(target_yaw / 2)
    if near_goal == 'short_lateral':
        target_yaw = side * math.radians(-7.56)
        c, s = math.cos(target_yaw), math.sin(target_yaw)
        target.pose.position.x = c * .018 - s * side * .013
        target.pose.position.y = s * .018 + c * side * .013
        target.pose.orientation.z = math.sin(target_yaw / 2)
        target.pose.orientation.w = math.cos(target_yaw / 2)
    if early_handoff_lateral is not None:
        from pathlib import Path
        import yaml
        config = yaml.safe_load((Path(__file__).parents[1] / 'config/docking.yaml').read_text())
        motion.node.parameters.update(config['dock_turn_backup']['ros__parameters'])
        # A base-centered model must configure a base-centered steering pivot.
        motion.node.parameters['apriltag_front_axle_x'] = (
            drive_pivot_x if estimated_drive_pivot_x is None else estimated_drive_pivot_x)
        target_yaw = side * math.radians(target_heading_degrees)
        c, s = math.cos(target_yaw), math.sin(target_yaw)
        target.pose.position.x = c * handoff_longitudinal - s * side * early_handoff_lateral
        target.pose.position.y = s * handoff_longitudinal + c * side * early_handoff_lateral
        target.pose.orientation.z = math.sin(target_yaw / 2)
        target.pose.orientation.w = math.cos(target_yaw / 2)
    node.last_refinement_target_pose = target
    node.last_refinement_target_received_at = state.now

    def publish(command):
        state.v = command.linear.x if abs(command.linear.x) >= .006 else 0.0
        state.w = command.angular.z * yaw_gain
        state.wheel_rate = command.angular.z
        if wheel_only_below_threshold and abs(command.angular.z) < .025:
            state.w = 0.0

    motion.cmd_vel_pub = SimpleNamespace(publish=publish)
    motion.stop_robot = lambda: publish(motion.twist())
    if (estimated_drive_pivot_x is not None
            and (not math.isfinite(estimated_drive_pivot_x)
                 or not 0 <= estimated_drive_pivot_x <= .20)):
        assert not node._refine_tag_front_pose()
        assert 'parameters are outside safe bounds' in logger.error.call_args.args[0]
        assert state.travel == state.yaw == state.v == state.w == 0
        return

    def tick(dt):
        state.now += dt
        # Front driven wheels plus rear casters can rotate about an axle ahead
        # of base_link, moving the body even with zero commanded linear speed.
        state.x += (state.v*math.cos(state.yaw) + drive_pivot_x*state.w*math.sin(state.yaw))*dt
        state.y += (state.v*math.sin(state.yaw) - drive_pivot_x*state.w*math.cos(state.yaw))*dt
        state.yaw += state.w * dt
        state.travel += abs(state.v) * dt
        motion.node.now_ns = int(state.now * 1e9)
        msg = odom(motion.node.now_ns)
        # The trial failed when wheel-derived pose claimed a centered chassis.
        # Keep physical pose separate from biased odometry in the live tests.
        scale = .65 if live_control else 1.
        yaw_scale = .8 if live_control else 1.
        msg.pose.pose.position.x, msg.pose.pose.position.y = scale*state.x, scale*state.y
        msg.pose.pose.orientation.z = math.sin(yaw_scale*state.yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw_scale*state.yaw / 2)
        msg.twist.twist.linear.x = state.v
        msg.twist.twist.angular.z = getattr(state, 'wheel_rate', 0.)
        motion._odom_callback(msg)
        imu = Imu()
        imu.header.stamp = msg.header.stamp
        imu.angular_velocity.z = state.w
        motion._imu_callback(imu)
        wheel = Vector3Stamped()
        wheel.header.stamp = msg.header.stamp
        wheel.vector.x = state.yaw
        motion._wheel_yaw_callback(wheel)
        if apriltag_control and int(state.now*10) != getattr(state, 'last_camera_cycle', None):
            state.last_camera_cycle = int(state.now*10)
            camera = PoseStamped()
            camera.header.frame_id = 'base_link'
            camera.header.stamp = msg.header.stamp
            dx = target.pose.position.x + .95*math.cos(target_yaw) - state.x
            dy = target.pose.position.y + .95*math.sin(target_yaw) - state.y
            camera.pose.position.x = math.cos(state.yaw)*dx + math.sin(state.yaw)*dy
            camera.pose.position.y = -math.sin(state.yaw)*dx + math.cos(state.yaw)*dy
            angle = target_yaw - state.yaw
            if state.travel > .01:
                angle += math.radians(camera_heading_noise_degrees)*math.sin(2*math.pi*4*state.now)
            s, c = math.sin(angle/2), math.cos(angle/2)
            camera.pose.orientation.x = s/math.sqrt(2)
            camera.pose.orientation.y = -c/math.sqrt(2)
            camera.pose.orientation.z = s/math.sqrt(2)
            camera.pose.orientation.w = c/math.sqrt(2)
            optical_bearing = math.degrees(math.atan2(
                camera.pose.position.y-.01471558, camera.pose.position.x-.2190114))
            visible = (camera_limit_degrees is None
                       or abs(optical_bearing) <= camera_limit_degrees)
            node.last_dock_pose = camera if visible else None
            node.last_dock_pose_received_at = state.now
            if hasattr(node, 'tag_centering') and hasattr(node.tag_centering, 'lidar'):
                node.tag_centering.lidar.sequence = int(state.now*10)

    tick(0.0)
    monkeypatch.setattr(rclpy, 'spin_once', lambda node, timeout_sec: tick(min(timeout_sec, .01)))
    if not front_verified:
        node.tag_centering = SimpleNamespace(
            refine=lambda target, **kwargs: target, last_verified_alignment=None)
        assert not node._refine_tag_front_pose()
        assert state.travel == 0
        assert state.v == state.w == 0
        assert 'uncorrected tag orientation' in logger.error.call_args.args[0]
        return
    events = []

    def observe(target, **kwargs):
        events.append('observe')
        return target

    def align(initial=False):
        assert initial and state.travel == 0
        events.append('align')
        node.tag_centering.last_verified_alignment = [(0., 0.)] * 5
        return True

    node.tag_centering = SimpleNamespace(
        refine=observe, _correct_live_heading=align,
        last_verified_alignment=[(0., side * math.radians(initial_panel_deg))] * 5)
    if live_control:
        node.tag_centering.last_verified_standoff = .95
        node.tag_centering.control_observation_sequence = 0

        def observe_live(standoff):
            assert standoff == .95
            if observation_gap and state.now >= 10.5:
                return None
            node.tag_centering.control_observation_sequence += 1
            return node._fixed_goal_errors_in_target(
                target.pose.position.x, target.pose.position.y, target_yaw,
                state.x, state.y, state.yaw)

        node.tag_centering.control_observation = observe_live
        if apriltag_control:
            from docking.tag_centering import AprilTagCentering
            centerer = AprilTagCentering.__new__(AprilTagCentering)
            centerer.node = node
            centerer.lidar = SimpleNamespace(
                sequence=int(state.now*10), validate_parameters=lambda: True,
                last_error='', project=lambda *args: [
                    SimpleNamespace(x=1.,y=y) for y in [-.04,0.,.04]])
            centerer.lidar.snapshot = lambda: SimpleNamespace(
                sequence=centerer.lidar.sequence,
                scan=SimpleNamespace(header=SimpleNamespace(
                    stamp=node.get_clock().now().to_msg())))
            node.tag_centering = centerer
        if stale_feedback:
            setattr(motion, stale_feedback + '_is_fresh', lambda: state.now < 10.5)
        if observation_gap or stale_feedback:
            assert not node._refine_tag_front_pose()
            assert state.v == state.w == 0
            assert state.now - 10 < 2
            reason = logger.error.call_args.args[0]
            assert ('observations did not recover' if observation_gap else
                    'Independent motion feedback became stale') in reason
            return
    if near_goal == 'short_lateral':
        assert not node._refine_tag_front_pose()
        assert state.travel == state.yaw == state.v == state.w == 0
        assert 'Insufficient approach distance' in logger.error.call_args.args[0]
        return
    assert node._refine_tag_front_pose(), (
        logger.error.call_args_list, logger.info.call_args_list[-5:])
    assert events == ([] if apriltag_control else ['observe', 'align', 'observe']
                      if initial_panel_deg and not near_goal else ['observe'])
    longitudinal, lateral, yaw = node._fixed_goal_errors_in_target(
        target.pose.position.x, target.pose.position.y, target_yaw,
        state.x, state.y, state.yaw)
    assert abs(longitudinal) <= .02
    assert abs(lateral) <= motion.node.parameters['tag_refinement_lateral_tolerance']
    assert abs(yaw) <= motion.node.parameters['tag_refinement_yaw_tolerance']
    assert state.now - 10 < 45
    assert state.travel <= motion.node.parameters['tag_refinement_max_travel']
    if near_goal:
        assert state.travel == 0
    assert state.v == state.w == 0


@pytest.mark.parametrize('side', [-1., 1.])
@pytest.mark.parametrize('lateral', [.03, .075, .12])
def test_early_handover_uses_forward_distance_for_centering_with_runtime_config(
        monkeypatch, side, lateral):
    test_one_centimeter_lateral_refinement_finishes_within_existing_limits(
        monkeypatch, side, True, False, 0, True, early_handoff_lateral=lateral)


@pytest.mark.parametrize('side', [-1., 1.])
@pytest.mark.parametrize('lateral', [.03, .075])
def test_live_front_centers_physical_pose_despite_biased_wheel_odometry(
        monkeypatch, side, lateral):
    test_one_centimeter_lateral_refinement_finishes_within_existing_limits(
        monkeypatch, side, True, False, 0, True,
        early_handoff_lateral=lateral, live_control=True)


@pytest.mark.parametrize('fault', ['observation_gap', 'imu', 'wheel_yaw'])
def test_live_front_stops_when_independent_observations_are_lost(monkeypatch, fault):
    test_one_centimeter_lateral_refinement_finishes_within_existing_limits(
        monkeypatch, 1., True, False, 0, True, early_handoff_lateral=.075,
        live_control=True, observation_gap=fault == 'observation_gap',
        stale_feedback=None if fault == 'observation_gap' else fault)


@pytest.mark.parametrize('side', [-1., 1.])
def test_actual_apriltag_pose_controller_centers_with_biased_odometry(monkeypatch, side):
    test_one_centimeter_lateral_refinement_finishes_within_existing_limits(
        monkeypatch, side, True, False, 0, True, early_handoff_lateral=.075,
        live_control=True, apriltag_control=True)


@pytest.mark.parametrize('side', [-1., 1.])
def test_trial40_offset_aligns_heading_first_and_preserves_camera_view(monkeypatch, side):
    test_one_centimeter_lateral_refinement_finishes_within_existing_limits(
        monkeypatch, side, True, False, 0, True, early_handoff_lateral=.119,
        live_control=True, apriltag_control=True, handoff_longitudinal=.43,
        target_heading_degrees=-15.74, yaw_gain=.9, camera_limit_degrees=23.)


@pytest.mark.parametrize('side', [-1., 1.])
def test_accepted_lateral_residual_finishes_standoff_without_a_large_point_turn(
        monkeypatch, side):
    test_one_centimeter_lateral_refinement_finishes_within_existing_limits(
        monkeypatch, side, True, False, 0, True, early_handoff_lateral=.024,
        live_control=True, apriltag_control=True, handoff_longitudinal=.035,
        camera_limit_degrees=23.)


@pytest.mark.parametrize('side', [-1., 1.])
def test_tag_heading_jitter_does_not_reverse_control_stages_repeatedly(monkeypatch, side):
    test_one_centimeter_lateral_refinement_finishes_within_existing_limits(
        monkeypatch, side, True, False, 0, True, early_handoff_lateral=.119,
        live_control=True, apriltag_control=True, handoff_longitudinal=.43,
        target_heading_degrees=-15.74, yaw_gain=.9, camera_limit_degrees=23.,
        camera_heading_noise_degrees=2.)


@pytest.mark.parametrize('side', [-1., 1.])
@pytest.mark.parametrize('pivot', [.08, .10, .12])
def test_front_axle_pivot_and_camera_jitter_finish_the_virtual_pose(monkeypatch, side, pivot):
    test_one_centimeter_lateral_refinement_finishes_within_existing_limits(
        monkeypatch, side, True, False, 0, True, early_handoff_lateral=.119,
        live_control=True, apriltag_control=True, handoff_longitudinal=.43,
        target_heading_degrees=-15.74, yaw_gain=.9, camera_limit_degrees=23.,
        camera_heading_noise_degrees=2., drive_pivot_x=pivot,
        estimated_drive_pivot_x=.10)


@pytest.mark.parametrize('offset', [math.nan, math.inf, -.01, .21])
def test_invalid_axle_estimate_stops_before_initial_heading_motion(monkeypatch, offset):
    test_one_centimeter_lateral_refinement_finishes_within_existing_limits(
        monkeypatch, 1., True, False, 16.22, True, early_handoff_lateral=.119,
        live_control=True, apriltag_control=True, handoff_longitudinal=.43,
        target_heading_degrees=-15.74, estimated_drive_pivot_x=offset)
