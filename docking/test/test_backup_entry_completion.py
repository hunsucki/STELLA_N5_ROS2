"""Replay the final-stop deadlock and simulate measured 54 cm guide entry."""

import math
from types import SimpleNamespace
from unittest.mock import Mock

from docking.motion import MotionController
from geometry_msgs.msg import Vector3Stamped
import pytest
import rclpy
from test_observation_timing import make_motion, odom


@pytest.fixture
def simulation(monkeypatch):
    state = SimpleNamespace(
        now=10.0, x=0.0, y=0.0, yaw=0.0, linear=0.0, angular=0.0,
        gap=0.05, offset=0.0, sent=[], unique_scans=0,
        yaw_response=lambda distance: 1.0, actual_angular=0.0)
    monkeypatch.setattr('time.monotonic', lambda: state.now)
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    motion = make_motion()
    motion.node.get_logger = Mock(return_value=Mock())
    motion.node.parameters.update({
        'backup_speed': .035, 'backup_min_speed': .012,
        'backup_slowdown_clearance': .18,
        'backup_heading_kp': .8,
        'backup_guide_center_kp': 1.0,
        'backup_guide_center_tolerance': .01,
        'backup_guide_center_max_heading': math.radians(4),
    })
    motion.should_stop = lambda: False
    motion.validate_lidar_configuration = lambda: True
    motion.imu_is_fresh = lambda: True
    motion.current_integrated_imu_yaw = lambda: state.yaw
    motion.current_imu_yaw_rate = lambda: state.actual_angular

    def publish(msg):
        state.linear, state.angular = msg.linear.x, msg.angular.z
        state.sent.append((state.now, state.x, state.y, state.linear, state.angular))

    motion.cmd_vel_pub = SimpleNamespace(publish=publish)
    motion.stop_robot = lambda: publish(MotionController.twist())
    motion.lidar = SimpleNamespace(sequence=0)
    motion.lidar.snapshot = lambda: SimpleNamespace(
        sequence=motion.lidar.sequence, scan=SimpleNamespace(angle_increment=.01))
    motion._rear_clearances = lambda _: [
        (i, state.gap + state.x) for i in range(12)]
    motion._rear_safety_clearances = motion._rear_clearances
    motion.backup_lidar_heading_estimator = lambda _: (
        -state.yaw, math.pi / 2 - state.yaw, 30, 30, .5)
    motion.backup_lidar_guide_estimator = lambda _: SimpleNamespace(
        center_offset=state.offset - state.y, separation=.54)

    def tick(dt):
        state.now += dt
        state.actual_angular = state.angular * state.yaw_response(abs(state.x))
        state.yaw += state.actual_angular * dt
        state.x += state.linear * math.cos(state.yaw) * dt
        state.y += state.linear * math.sin(state.yaw) * dt
        motion.node.now_ns = int(state.now * 1e9)
        sample = odom(motion.node.now_ns)
        sample.pose.pose.position.x = state.x
        sample.pose.pose.position.y = state.y
        sample.pose.pose.orientation.z = math.sin(state.yaw / 2)
        sample.pose.pose.orientation.w = math.cos(state.yaw / 2)
        sample.twist.twist.linear.x = state.linear
        sample.twist.twist.angular.z = state.actual_angular
        motion._odom_callback(sample)
        wheel = Vector3Stamped()
        wheel.header.stamp = sample.header.stamp
        wheel.vector.x = state.yaw
        motion._wheel_yaw_callback(wheel)
        motion.lidar.sequence = int((state.now - 10) * 10)

    tick(0.0)
    monkeypatch.setattr(
        rclpy, 'spin_once', lambda node, timeout_sec: tick(min(timeout_sec, .01)))
    return motion, state


def test_nearest_beam_does_not_deadlock_before_panel_cluster_enters_tolerance(simulation):
    motion, state = simulation
    # At the old stop threshold the first return is 24 mm, the supported
    # panel 28 mm and the protective return 22 mm, matching the log pattern.
    motion._rear_clearances = lambda _: [
        (0, state.gap + state.x),
        *[(i, state.gap + state.x + .004) for i in range(1, 12)],
    ]
    motion._rear_safety_clearances = lambda _: [(0, state.gap + state.x - .002)]
    assert motion.backup_with_lidar()
    assert .0145 <= state.gap + state.x - .002
    assert state.gap + state.x + .004 <= .0245
    assert state.linear == state.angular == 0
    assert 'sequence complete' in motion.node.get_logger().info.call_args.args[0]


@pytest.mark.parametrize('fault', [None, 'far_loss', 'off_center', 'never_seen', 'imu_loss', 'yaw_drift'])
def test_verified_guide_visibility_handoff(simulation, fault):
    motion, state = simulation
    state.gap = .38
    motion.node.parameters.update(use_lidar_guide_centering=True,
                                  backup_guide_visibility_handoff_clearance=.22)
    original_heading = motion.backup_lidar_heading_estimator

    def guide(_):
        gap = state.gap + state.x
        if fault == 'never_seen' or gap < (.30 if fault == 'far_loss' else .20):
            return None
        return SimpleNamespace(center_offset=.008 if fault == 'off_center' else .001,
                               separation=.54)

    def heading(snapshot):
        return original_heading(snapshot)

    motion.backup_lidar_guide_estimator = guide
    motion.backup_lidar_heading_estimator = heading
    if fault == 'imu_loss':
        motion.imu_is_fresh = lambda: state.gap + state.x >= .17
    if fault == 'yaw_drift':
        # The panel estimator is deliberately unused after the handoff.
        motion.current_integrated_imu_yaw = lambda: (
            math.radians(2) if state.gap + state.x < .17 else state.yaw)
    assert motion.backup_with_lidar() is (fault is None)
    assert state.linear == state.angular == 0
    if fault is None:
        assert state.gap + state.x <= .0245
        assert any('Guide visibility handoff:' in str(call)
                   for call in motion.node.get_logger().info.call_args_list)
        assert all(abs(w) < 1e-9 for _, x, _, v, w in state.sent
                   if state.gap + x < .195 and v < 0)


@pytest.mark.parametrize('fault', [None, 'imu_loss', 'yaw_drift', 'overrun', 'odom_loss', 'wheel_loss'])
@pytest.mark.parametrize('known_width', [0., .54])
def test_proactive_straight_entry_ignores_degraded_fits_but_keeps_motion_guards(
        simulation, fault, known_width):
    motion, state = simulation
    state.gap = .40
    motion.node.parameters.update(use_lidar_guide_centering=True,
                                  backup_guide_visibility_handoff_clearance=.32,
                                  backup_guide_known_width=known_width)
    original_heading = motion.backup_lidar_heading_estimator
    original_odom_fresh = motion.odom_is_fresh
    original_wheel_fresh = motion.wheel_yaw_is_fresh
    calls_after_commit = []

    def heading(snapshot):
        if state.gap + state.x < .28:
            calls_after_commit.append('panel')
            return (math.radians(4), 0, 30, 30, .23)
        return original_heading(snapshot)

    def guide(_):
        if state.gap + state.x < .28:
            calls_after_commit.append('guide')
            return SimpleNamespace(center_offset=.02, separation=.50)
        return SimpleNamespace(center_offset=.001, separation=.504)

    motion.backup_lidar_heading_estimator = heading
    motion.backup_lidar_guide_estimator = guide
    motion.imu_is_fresh = lambda: not (fault == 'imu_loss' and state.gap + state.x < .20)
    motion.current_integrated_imu_yaw = lambda: (
        math.radians(2) if fault == 'yaw_drift' and state.gap + state.x < .20 else state.yaw)
    motion.odom_is_fresh = lambda: original_odom_fresh() and not (
        fault == 'odom_loss' and state.gap + state.x < .20)
    motion.wheel_yaw_is_fresh = lambda: original_wheel_fresh() and not (
        fault == 'wheel_loss' and state.gap + state.x < .20)
    if fault == 'overrun':
        motion._rear_clearances = lambda _: [
            (i, .002 if state.gap + state.x < .20 else state.gap + state.x)
            for i in range(12)]
    assert motion.backup_with_lidar() is (fault is None)
    assert not calls_after_commit
    assert state.linear == state.angular == 0
    if fault is None:
        assert state.gap + state.x <= .0245
        # No pause to switch strategies; remain in continuous reverse until
        # normal distance completion asks for stationary confirmation.
        assert all(v < 0 and w == 0 for _, x, _, v, w in state.sent
                   if .04 < state.gap + x < .33)


@pytest.mark.parametrize('handoff_clearance,fault', [
    (.32, None), (.50, None), (.50, 'imu_loss'), (.50, 'yaw_drift'),
    (.50, 'odom_loss'), (.50, 'wheel_loss'), (.50, 'overrun'),
])
def test_long_measured_corridor_commits_before_near_field_fit_degrades(
        simulation, handoff_clearance, fault):
    motion, state = simulation
    state.gap = .48
    state.offset = .005
    motion.node.parameters.update(
        use_lidar_guide_centering=True,
        backup_guide_center_tolerance=.015,
        backup_guide_visibility_handoff_clearance=handoff_clearance,
        backup_guide_known_width=.54)
    original_guide = motion.backup_lidar_guide_estimator
    original_heading = motion.backup_lidar_heading_estimator
    original_odom = motion.odom_is_fresh
    original_wheel = motion.wheel_yaw_is_fresh
    bad_fits = []

    def guide(snapshot):
        if state.gap + state.x < .40:
            bad_fits.append(state.now)
            return None
        return original_guide(snapshot)

    motion.backup_lidar_guide_estimator = guide
    motion.backup_lidar_heading_estimator = lambda s: (
        (math.radians(3), 0, 30, 30, .5)
        if state.gap + state.x < .40 else original_heading(s))
    motion.imu_is_fresh = lambda: not (
        fault == 'imu_loss' and state.gap + state.x < .35)
    motion.current_integrated_imu_yaw = lambda: (
        math.radians(2) if fault == 'yaw_drift' and state.gap + state.x < .35
        else state.yaw)
    motion.odom_is_fresh = lambda: original_odom() and not (
        fault == 'odom_loss' and state.gap + state.x < .35)
    motion.wheel_yaw_is_fresh = lambda: original_wheel() and not (
        fault == 'wheel_loss' and state.gap + state.x < .35)
    if fault == 'overrun':
        motion._rear_clearances = lambda _: [
            (i, .002 if state.gap + state.x < .35 else state.gap + state.x)
            for i in range(12)]

    assert motion.backup_with_lidar() is (handoff_clearance == .50 and fault is None)
    assert state.linear == state.angular == 0
    if handoff_clearance == .50:
        assert not bad_fits
        assert any('Guide visibility handoff:' in str(call)
                   for call in motion.node.get_logger().info.call_args_list)
    else:
        assert bad_fits


@pytest.mark.parametrize('known_width,clearance', [(0., .50), (.54, .501)])
def test_long_handoff_requires_measured_profile_and_bounded_distance(
        simulation, known_width, clearance):
    motion, state = simulation
    motion.node.parameters.update(
        backup_guide_known_width=known_width,
        backup_guide_visibility_handoff_clearance=clearance)
    assert not motion.backup_with_lidar()
    assert all(v == w == 0 for _, _, _, v, w in state.sent)


@pytest.mark.parametrize('offset', [-.017, .017])
def test_measured_entry_remembers_safe_corridor_when_guides_leave_view(simulation, offset):
    motion, state = simulation
    state.gap, state.offset = .48, offset
    motion.node.parameters.update(
        use_lidar_guide_centering=True,
        backup_guide_center_tolerance=.015,
        backup_guide_visibility_handoff_clearance=.50,
        backup_guide_known_width=.54)
    guide = motion.backup_lidar_guide_estimator
    motion.backup_lidar_guide_estimator = lambda snapshot: (
        None if state.gap+state.x < .45 else guide(snapshot))
    assert motion.backup_with_lidar()
    assert any('Guide visibility handoff:' in str(call)
               for call in motion.node.get_logger().info.call_args_list)
    # Initial visible-guide correction is permitted; after the rails leave
    # view the committed entry must continue without steering or reacquisition.
    assert all(w == 0 for _, x, _, v, w in state.sent
               if v < 0 and state.gap+x < .45)
    assert .0145 <= state.gap+state.x <= .0245
    assert state.linear == state.angular == 0


@pytest.mark.parametrize('bad_initial_plane', [False, True])
def test_verified_warmup_scans_count_before_guide_tips_leave_lidar_view(
        simulation, bad_initial_plane):
    motion, state = simulation
    state.gap, state.offset = .48, .017
    motion.node.parameters.update(
        use_lidar_guide_centering=True,
        backup_guide_center_tolerance=.015,
        backup_guide_visibility_handoff_clearance=.50,
        backup_guide_known_width=.54)
    guide = motion.backup_lidar_guide_estimator
    # Sequence zero belongs to the preceding stage; exactly five new scans
    # (1..5) remain visible before the guide leaves the sensor's field of view.
    motion.backup_lidar_guide_estimator = lambda snapshot: (
        guide(snapshot) if snapshot.sequence < 6 else None)
    heading = motion.backup_lidar_heading_estimator
    if bad_initial_plane:
        motion.backup_lidar_heading_estimator = lambda snapshot: (
            None if snapshot.sequence < 3 else heading(snapshot))
    assert motion.backup_with_lidar() is (not bad_initial_plane)
    committed = any('Guide visibility handoff:' in str(call)
                    for call in motion.node.get_logger().info.call_args_list)
    assert committed is (not bad_initial_plane)
    assert state.linear == state.angular == 0


@pytest.mark.parametrize('legacy_cap,gyro_delta_deg,expected', [
    (True, .18, False), (False, .18, True), (False, 2., False),
])
def test_measured_corridor_distinguishes_small_gyro_change_from_unsafe_heading(
        simulation, monkeypatch, legacy_cap, gyro_delta_deg, expected):
    motion, state = simulation
    state.gap = .48
    state.offset = .003
    state.yaw = math.radians(.85)
    motion.node.parameters.update(
        use_lidar_guide_centering=True,
        backup_guide_center_tolerance=.015,
        backup_guide_visibility_handoff_clearance=.50,
        backup_guide_known_width=.54)
    if legacy_cap:
        from docking import motion as motion_module
        original = motion_module.straight_entry_heading_budget

        def old_budget(rows, distance, center, half, side, limit, rear, **kwargs):
            return original(rows, distance, center, half, side,
                            min(limit, math.radians(1)), rear, **kwargs)

        monkeypatch.setattr(motion_module, 'straight_entry_heading_budget', old_budget)
    motion.current_integrated_imu_yaw = lambda: (
        state.yaw + (math.radians(gyro_delta_deg)
                     if state.gap + state.x < .38 else 0.))
    assert motion.backup_with_lidar() is expected
    assert state.linear == state.angular == 0
    if not expected:
        message = motion.node.get_logger().error.call_args.args[0]
        assert 'bounded straight heading/travel' in message
        assert 'heading=' in message and 'budget=' in message


@pytest.mark.parametrize('unsafe_side', [False, True])
def test_isolated_return_or_unsafe_side_cannot_report_success(simulation, unsafe_side):
    motion, state = simulation
    if unsafe_side:
        state.gap = .020
        motion._rear_safety_clearances = lambda _: [(0, .003)]
    else:
        motion._rear_clearances = lambda _: [(0, state.gap + state.x)]
        motion._rear_safety_clearances = motion._rear_clearances
    assert not motion.backup_with_lidar()
    assert 'blocked' in motion.node.get_logger().error.call_args.args[0]
    assert state.linear == state.angular == 0
    if not unsafe_side:
        assert state.gap + state.x > .0045


def test_overrun_is_still_rejected(simulation):
    motion, state = simulation
    state.gap = .002
    assert not motion.backup_with_lidar()
    assert 'overrun' in motion.node.get_logger().error.call_args.args[0]


@pytest.mark.parametrize('offset', [-.018, .018])
def test_guide_controller_centers_both_sides_before_final_straight_entry(simulation, offset):
    motion, state = simulation
    state.gap, state.offset = .371, offset
    motion.node.parameters['use_lidar_guide_centering'] = True
    assert motion.backup_with_lidar()
    # The 1 cm gate applies before the final 12 cm straight segment. That
    # segment retains up to 1 degree of accepted heading error, so its final
    # lateral position can differ by up to 12 cm * sin(1 degree).
    entry = next(sample for sample in state.sent if state.gap + sample[1] <= .12)
    assert abs(state.offset - entry[2]) <= .01
    assert abs(state.offset - state.y) <= .01 + .12 * math.sin(math.radians(1))
    moving = [sample for sample in state.sent if sample[3] < 0]
    assert moving
    assert moving[0][0] >= 10.2  # Wait for distinct guide observations.
    assert .012 <= abs(moving[0][3]) < .035
    assert any(w * offset < 0 for _, _, _, _, w in state.sent)
    assert state.linear == state.angular == 0


@pytest.mark.parametrize('missing_callback', [False, True])
@pytest.mark.parametrize('known_width', [0., .54])
def test_missing_guides_do_not_allow_blind_reverse_entry(simulation, missing_callback, known_width):
    motion, state = simulation
    state.gap = .371
    motion.node.parameters['use_lidar_guide_centering'] = True
    motion.node.parameters['backup_guide_known_width'] = known_width
    motion.backup_lidar_guide_estimator = None if missing_callback else lambda _: None
    assert not motion.backup_with_lidar()
    assert all(v == w == 0 for _, _, _, v, w in state.sent)
    assert 'Guide rails unavailable' in motion.node.get_logger().error.call_args.args[0]


def test_uncentered_robot_cannot_enter_final_straight_region(simulation):
    motion, state = simulation
    state.gap = .121
    motion.node.parameters['use_lidar_guide_centering'] = True
    # Persistent large offset (e.g. lateral wheel slip) must not be ignored
    # merely because the rear wall is now closer than the steering cutoff.
    motion.backup_lidar_guide_estimator = lambda _: SimpleNamespace(
        center_offset=.025, separation=.54)
    assert not motion.backup_with_lidar()
    assert state.gap + state.x > .11
    assert 'not centered' in motion.node.get_logger().error.call_args.args[0]


@pytest.mark.parametrize('offset', [-.04, .04])
def test_insufficient_side_clearance_stops_before_entering_guides(simulation, offset):
    motion, state = simulation
    state.gap, state.offset = .371, offset
    motion.node.parameters['use_lidar_guide_centering'] = True
    assert not motion.backup_with_lidar()
    assert all(v == w == 0 for _, _, _, v, w in state.sent)
    assert 'side clearance' in motion.node.get_logger().error.call_args.args[0]


def test_reflected_guide_offset_is_rechecked_then_entry_resumes(simulation):
    motion, state = simulation
    state.gap = .371
    motion.node.parameters.update(use_lidar_guide_centering=True,
                                  backup_guide_known_width=.54)
    estimator = motion.backup_lidar_guide_estimator

    def reflected(snapshot):
        # Reproduce the user's 35 mm offset / 577 mm width. It was
        # terminal after three scans, before any reverse command.
        if state.now < 10.7:
            return SimpleNamespace(center_offset=.035, separation=.577)
        return estimator(snapshot)

    motion.backup_lidar_guide_estimator = reflected
    assert motion.backup_with_lidar()
    assert all(v == w == 0 for t, _, _, v, w in state.sent if t < 10.9)
    assert any(v < 0 for t, _, _, v, _ in state.sent if t >= 10.9)
    assert any('recheck guide side clearance' in call.args[0]
               for call in motion.node.get_logger().warning.call_args_list)
    assert not motion.node.get_logger().error.called


@pytest.mark.parametrize('loss_after_bad_fit', [False, True])
def test_persistent_bad_clearance_has_one_bounded_stopped_recheck(
        simulation, loss_after_bad_fit):
    motion, state = simulation
    state.gap = .371
    motion.node.parameters.update(use_lidar_guide_centering=True,
                                  backup_guide_known_width=.54,
                                  backup_plane_reacquire_timeout_sec=3.)

    def guide(snapshot):
        if loss_after_bad_fit and state.now > 10.7:
            return None
        return SimpleNamespace(center_offset=.035, separation=.577)

    motion.backup_lidar_guide_estimator = guide
    assert not motion.backup_with_lidar()
    assert 13.2 <= state.now <= 13.5
    assert all(v == w == 0 for _, _, _, v, w in state.sent)
    message = motion.node.get_logger().error.call_args.args[0]
    assert 'side clearance' in message
    assert 'supported_width=0.530m' in message
    assert 'center_offset=+0.035m' in message


def test_one_clear_scan_amid_bad_clearances_cannot_resume_reverse(simulation):
    motion, state = simulation
    state.gap = .371
    motion.node.parameters.update(use_lidar_guide_centering=True,
                                  backup_guide_known_width=.54)
    # All centers are within the normal filter's jump tolerance. Stable
    # center readings alone must not count unsafe side clearances as recovery.
    motion.backup_lidar_guide_estimator = lambda snapshot: SimpleNamespace(
        center_offset=.024 if snapshot.sequence % 3 == 0 else .027,
        separation=.54)
    assert not motion.backup_with_lidar()
    assert all(v == w == 0 for _, _, _, v, w in state.sent)
    assert 'side clearance' in motion.node.get_logger().error.call_args.args[0]


@pytest.mark.parametrize('offset', [.026, .030])
def test_tilted_entry_uses_actual_rear_frame_before_reverse(simulation, offset):
    motion, state = simulation
    state.gap, state.offset, state.yaw = .371, offset, math.radians(-2.5)
    motion.last_odom.pose.pose.orientation.z = math.sin(state.yaw/2)
    motion.last_odom.pose.pose.orientation.w = math.cos(state.yaw/2)
    motion.last_wheel_yaw = state.yaw
    motion.node.parameters.update(use_lidar_guide_centering=True,
                                  backup_guide_known_width=.54,
                                  backup_guide_center_tolerance=.015)

    def guide(snapshot):
        corridor_y = state.offset - state.y
        return SimpleNamespace(center_offset=corridor_y, separation=.54,
                               rear_center_offset=corridor_y-.2295*math.sin(-state.yaw))

    motion.backup_lidar_guide_estimator = guide
    motion.should_stop = lambda: abs(state.x) >= .05
    # Both offsets fail the old origin-based entry check. Rear clearance
    # permits initial guided reverse; full-path proof is still required.
    # End this software trial after 5 cm; this verifies entry, not a full
    # physical docking run or the unchanged final straight-path criterion.
    assert not motion.backup_with_lidar()
    assert any(v < 0 for _, _, _, v, _ in state.sent)
    assert not motion.node.get_logger().error.called
    assert state.linear == state.angular == 0


def test_invalid_transformed_rear_offset_cannot_start_reverse(simulation):
    motion, state = simulation
    state.gap = .371
    motion.node.parameters.update(use_lidar_guide_centering=True,
                                  backup_guide_known_width=.54)
    motion.backup_lidar_guide_estimator = lambda _: SimpleNamespace(
        center_offset=.001, separation=.54, rear_center_offset=math.nan)
    assert not motion.backup_with_lidar()
    assert all(v == w == 0 for _, _, _, v, w in state.sent)


def test_rear_frame_correction_can_also_reject_a_tilted_entry(simulation):
    motion, state = simulation
    state.gap = .371
    motion.node.parameters.update(use_lidar_guide_centering=True,
                                  backup_guide_known_width=.54)
    # Opposite tilt moves the rear closer to the rail, even if the old
    # panel-origin offset looks acceptable. This is not an error discount.
    motion.backup_lidar_guide_estimator = lambda _: SimpleNamespace(
        center_offset=.015, separation=.54, rear_center_offset=.027)
    assert not motion.backup_with_lidar()
    assert all(v == w == 0 for _, _, _, v, w in state.sent)
    assert 'side clearance' in motion.node.get_logger().error.call_args.args[0]


def test_temporary_guide_loss_stops_and_reacquires_distinct_scans(simulation):
    motion, state = simulation
    state.gap = .371
    motion.node.parameters['use_lidar_guide_centering'] = True
    estimator = motion.backup_lidar_guide_estimator
    motion.backup_lidar_guide_estimator = lambda snapshot: (
        None if 11.0 < state.now < 11.5 else estimator(snapshot))
    assert motion.backup_with_lidar()
    assert all(v == w == 0 for t, _, _, v, w in state.sent if 11.12 < t < 11.65)
    assert any(v < 0 for t, _, _, v, _ in state.sent if t > 11.8)


def test_disabled_guide_does_not_fit_or_abort_on_reflective_rails(simulation):
    from pathlib import Path
    import yaml

    motion, state = simulation
    config = yaml.safe_load((Path(__file__).parents[1] / 'config/docking.yaml').read_text())
    motion.node.parameters.update(config['dock_turn_backup']['ros__parameters'])
    motion.node.parameters['use_lidar_guide_centering'] = False
    state.gap = .371
    motion.backup_lidar_guide_estimator = Mock(
        side_effect=AssertionError('disabled guide estimator must never control backup'))
    assert motion.backup_with_lidar()
    motion.backup_lidar_guide_estimator.assert_not_called()
    assert state.linear == state.angular == 0


def test_small_wall_error_slows_translation_before_stop_threshold(simulation):
    motion, state = simulation
    state.gap = .371
    state.yaw = math.radians(2.0)
    assert motion.backup_with_lidar()
    moving = [v for _, _, _, v, _ in state.sent if v < 0]
    assert .012 <= abs(moving[0]) < .025
    assert abs(state.y) < .01


@pytest.mark.parametrize('direction', [-1, 1])
def test_stationary_correction_uses_separate_speed_then_resumes_reverse(simulation, direction):
    motion, state = simulation
    state.gap = .371
    state.yaw = direction * math.radians(4)
    assert motion.backup_with_lidar()
    rotating = [(t, v, w) for t, _, _, v, w in state.sent if abs(w) > .0151]
    assert rotating
    assert all(v == 0 and abs(w) <= .06 for t, v, w in rotating)
    first_reverse = next(t for t, _, _, v, _ in state.sent if v < 0)
    assert first_reverse < 16.0
    assert state.linear == state.angular == 0


def test_stationary_correction_refuses_stale_gyro(simulation):
    motion, state = simulation
    state.gap, state.yaw = .371, math.radians(4)
    motion.imu_is_fresh = lambda: False
    assert not motion.backup_with_lidar()
    assert state.linear == state.angular == 0
    assert 'fresh independent IMU' in motion.node.get_logger().error.call_args.args[0]


@pytest.mark.parametrize('direction', [-1, 1])
def test_noisy_guide_bias_does_not_interrupt_continuous_reverse(simulation, direction):
    motion, state = simulation
    state.gap, state.offset = .42, direction * .010
    motion.node.parameters['use_lidar_guide_centering'] = True
    motion.backup_lidar_guide_estimator = lambda _: SimpleNamespace(
        center_offset=state.offset - state.y + .001 * math.sin(state.now * 5),
        separation=.54)
    assert motion.backup_with_lidar()
    moving = [s for s in state.sent if s[3] < 0]
    first = moving[0][0]
    # Cover at least 15 cm continuously, preserving caster loading instead
    # of alternating reverse and stationary alignment on guide noise.
    until = next(s[0] for s in moving if abs(s[1]) >= .15)
    assert all(s[3] < 0 for s in state.sent if first <= s[0] <= until)
    assert until - first < 8


@pytest.mark.parametrize('direction', [-1, 1])
def test_guide_bias_cannot_hide_real_wall_tilt(simulation, direction):
    motion, state = simulation
    state.gap, state.offset, state.yaw = .42, -direction * .018, direction * math.radians(4)
    motion.node.parameters['use_lidar_guide_centering'] = True
    motion.backup_with_lidar()
    rotating = [s for s in state.sent if abs(s[4]) > .0151]
    assert rotating
    assert all(s[3] == 0 and s[4] * direction < 0 for s in rotating)
    assert state.linear == state.angular == 0


@pytest.mark.parametrize('direction', [-1, 1])
def test_reduced_yaw_response_during_initial_reverse_does_not_force_stops(simulation, direction):
    motion, state = simulation
    state.gap, state.offset = .44, direction * .009
    # Sensitivity scenario only, not an identified caster model. Initial
    # translation must continue even with weak steering until 8 cm of travel.
    state.yaw_response = lambda distance: .35 if distance < .08 else 1.0
    motion.node.parameters['use_lidar_guide_centering'] = True
    assert motion.backup_with_lidar()
    first = next(s[0] for s in state.sent if s[3] < 0)
    until = next(s[0] for s in state.sent if abs(s[1]) >= .15)
    assert all(s[3] < 0 for s in state.sent if first <= s[0] <= until)
    assert abs(state.offset - state.y) < .01


@pytest.mark.parametrize('offset', [-.015, .015])
@pytest.mark.parametrize('yaw', [-math.radians(1), math.radians(1)])
def test_pre_spin_handoff_residual_meets_unchanged_final_entry_gates(simulation, offset, yaw):
    motion, state = simulation
    state.gap, state.offset, state.yaw = .371, offset, yaw
    motion.node.parameters['use_lidar_guide_centering'] = True
    assert motion.backup_with_lidar()
    entry = next(s for s in state.sent if state.gap + s[1] <= .12)
    assert abs(state.offset - entry[2]) <= .01
    assert state.linear == state.angular == 0
