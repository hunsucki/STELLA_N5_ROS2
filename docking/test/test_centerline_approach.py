"""Exercise the single approach loop with physical tag geometry and failures."""

import json
from collections import deque
import math
from pathlib import Path
from types import SimpleNamespace

from docking.centerline_approach import TagCenterlineApproach
from docking.dock_turn_backup import DockTurnBackup
from docking.tag_centering import AprilTagCentering
from geometry_msgs.msg import PoseStamped
import pytest
import rclpy
import yaml
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import TransformException
from test_backup_entry_completion import simulation  # noqa: F401


def set_normal(tag, heading):
    s, c, h = math.sin(heading/2), math.cos(heading/2), math.sqrt(.5)
    q = tag.pose.orientation
    q.x, q.y, q.z, q.w = -s*h, c*h, s*h, c*h


def prepare(simulation, monkeypatch, lateral=.18, runway=.55, yaw=math.radians(10.41), fault=None,
            visibility=False, lateral_offset=-.03):
    motion, state = simulation
    node = motion.node
    node.motion = motion
    node.should_stop = lambda: fault == 'cancel' and state.now >= 10.8
    node.parameters.update(apriltag_front_standoff=.95, apriltag_front_lateral_offset=lateral_offset,
                           apriltag_front_axle_x=.10, tag_refinement_lateral_tolerance=.025)
    node._pose_is_fresh = DockTurnBackup._pose_is_fresh.__get__(node)
    node.last_dock_pose_received_at = 0.
    state.yaw = yaw
    if fault == 'odom_translation_drift':
        motion.current_xy = lambda: (
            state.x+.04*min(1., (state.now-10.)/5),
            state.y-.03*min(1., (state.now-10.)/5))
    station = (.95+runway, lateral+lateral_offset)
    original_spin = rclpy.spin_once
    history = [(state.now, state.x, state.y, state.yaw)]

    def spin(node, timeout_sec):
        before = state.yaw
        requested = state.linear, state.angular
        if fault == 'motion_camera_delay':
            old = [r for r in state.sent if r[0] <= state.now-.2]
            state.linear, state.angular = (old[-1][3:] if old else (0., 0.))
        original_spin(node, timeout_sec)
        if fault == 'motion_camera_delay':
            state.linear, state.angular = requested
        # Front drive axle is 10cm ahead of base: in-place yaw shifts base.
        state.x += .10*(math.cos(before)-math.cos(state.yaw))
        state.y += .10*(math.sin(before)-math.sin(state.yaw))
        motion.last_odom.pose.pose.position.x = state.x
        motion.last_odom.pose.pose.position.y = state.y
        message = Imu()
        message.header.stamp = node.get_clock().now().to_msg()
        message.angular_velocity.z = state.actual_angular
        motion._imu_callback(message)
        history.append((state.now, state.x, state.y, state.yaw))

    monkeypatch.setattr(rclpy, 'spin_once', spin)
    centerer = AprilTagCentering.__new__(AprilTagCentering)
    centerer.node = node
    centerer.view_guard = SimpleNamespace(limit=lambda tag, v, w, *a, **kw: (v, w))
    if visibility:
        from test_tag_visibility import make_guard
        centerer.view_guard, _ = make_guard(node)
    old_buffer = getattr(node, 'tf_buffer', None)

    def transform_at(fixed, base, stamp):
        if base != 'base_link':
            return old_buffer.lookup_transform(fixed, base, stamp)
        if fault == 'missing_tf':
            raise TransformException('no camera-time odom transform')
        _, x, y, yaw = min(history, key=lambda row: abs(row[0]-stamp.nanoseconds/1e9))
        if fault == 'odom_translation_drift':
            x += .04*min(1., (state.now-10.)/5)
            y -= .03*min(1., (state.now-10.)/5)
        return SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=x, y=y),
            rotation=SimpleNamespace(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2))))

    node.tf_buffer = SimpleNamespace(lookup_transform=transform_at)
    last_sequence = [-1]

    def snapshot():
        elapsed = state.now-10.
        seq = int(elapsed*10)
        if seq != last_sequence[0]:
            last_sequence[0] = seq
            tag = PoseStamped()
            tag.header.frame_id = 'base_link'
            from rclpy.time import Time
            observed = min(history, key=lambda row: abs(row[0]-(state.now-.12))) if fault == 'motion_camera_delay' else history[-1]
            at, x, y, yaw = observed
            tag.header.stamp = Time(nanoseconds=int(at*1e9)).to_msg()
            c, s = math.cos(yaw), math.sin(yaw)
            dx, dy = station[0]-x, station[1]-y
            tag.pose.position.x, tag.pose.position.y = c*dx+s*dy, -s*dx+c*dy
            tag.pose.position.z = .142
            normal = -yaw
            if fault == 'normal_spike' and seq == 8:
                normal -= math.copysign(math.radians(1.1), lateral)
            if fault == 'perspective_bias':
                normal += math.radians(8)*max(0., min(1., (tag.pose.position.x-1.1)/.9))
            set_normal(tag, normal)
            if fault != 'frozen_tag' or elapsed < .8:
                node.last_dock_pose, node.last_dock_pose_received_at = tag, state.now
                if visibility:
                    centerer.view_guard.received_at = state.now
        if (fault == 'camera_gap' and .8 <= elapsed < 1.5
                or fault == 'camera_lost' and elapsed >= .8
                or fault == 'camera_repeated' and (.8 <= elapsed < 5.8 or 6.5 <= elapsed < 11.5)):
            node.last_dock_pose_received_at = 0.
        header = SimpleNamespace(stamp=node.get_clock().now().to_msg())
        if fault == 'motion_camera_delay':
            # Trial66 delivery median: camera/tag 88ms, front scan 101ms.
            from rclpy.time import Time
            header.stamp = Time(nanoseconds=int((state.now-.10)*1e9)).to_msg()
        return SimpleNamespace(sequence=seq, scan=SimpleNamespace(header=header))

    centerer.lidar = SimpleNamespace(snapshot=snapshot, project=lambda *args: [
        SimpleNamespace(x=.25 if fault == 'obstacle' and state.now >= 10.8 else 2.)]*3)
    node.tag_centering = centerer
    if fault in ('imu_lost', 'odom_lost', 'wheel_lost'):
        method = {'imu_lost': 'imu_is_fresh', 'odom_lost': 'odom_is_fresh',
                  'wheel_lost': 'wheel_yaw_is_fresh'}[fault]
        original_fresh = getattr(motion, method)
        setattr(motion, method, lambda: state.now < 10.8 and original_fresh())
    if fault == 'stalled':
        def stopped_spin(node, timeout_sec):
            state.linear = state.angular = 0.
            spin(node, timeout_sec)
        monkeypatch.setattr(rclpy, 'spin_once', stopped_spin)
    snapshot()
    return TagCenterlineApproach(node), station


@pytest.mark.parametrize('lateral', [-.18, -.08, 0., .08, .18])
@pytest.mark.parametrize('runway', [.55, 1.0])
def test_single_loop_centers_pose_without_coarse_action_or_repeat_trims(
        simulation, monkeypatch, lateral, runway):
    motion, state = simulation
    approach, station = prepare(simulation, monkeypatch, lateral, runway,
                                yaw=math.copysign(math.radians(10.41), lateral),
                                lateral_offset=0.)
    assert approach.run(), motion.node.get_logger().error.call_args
    assert abs(station[0]-.95-state.x) <= .02
    assert abs(station[1]-state.y) <= .025
    assert abs(state.yaw) <= math.radians(2)
    assert state.now-10 < 45
    assert all(0 <= v <= .08 and abs(w) <= .15 for _, _, _, v, w in state.sent)
    assert state.linear == state.angular == 0


@pytest.mark.parametrize('fault', ['normal_spike', 'perspective_bias', 'odom_translation_drift', 'motion_camera_delay', 'missing_tf', 'camera_gap', 'camera_lost', 'camera_repeated',
                                  'frozen_tag', 'imu_lost', 'odom_lost', 'wheel_lost',
                                  'obstacle', 'cancel', 'stalled'])
def test_approach_handles_noise_but_stops_on_unreliable_or_blocked_motion(simulation, monkeypatch, fault):
    motion, state = simulation
    approach, _ = prepare(simulation, monkeypatch, fault=fault)
    assert approach.run() is (fault in ('normal_spike', 'perspective_bias', 'odom_translation_drift', 'motion_camera_delay', 'camera_gap')), motion.node.get_logger().error.call_args
    if fault == 'odom_translation_drift':
        assert abs(state.y-.18) <= .025
        assert abs(state.x-.55) <= .02
    assert state.linear == state.angular == 0
    assert all(v >= 0 for _, _, _, v, _ in state.sent)
    if fault == 'camera_gap':
        assert all(v == w == 0 for t, _, _, v, w in state.sent if 10.9 <= t <= 11.6)
    if fault == 'cancel':
        assert state.now < 11.
    if fault in ('imu_lost', 'odom_lost', 'wheel_lost'):
        # Stop motion immediately, but keep the process alive only for the
        # shared, bounded observation recovery window before reporting loss.
        assert all(v == w == 0 for t, _, _, v, w in state.sent if t >= 10.9)
        assert state.now < 19.
        assert 'stopped observation wait exhausted' in motion.node.get_logger().error.call_args.args[0]
        assert all(v == w == 0 for t, _, _, v, w in state.sent if t >= 10.8)
    if fault == 'camera_repeated':
        assert state.now < 20.5  # Two gaps share the original cumulative 8s budget.


@pytest.mark.parametrize('side', [-1, 1])
def test_centerline_trajectory_preserves_measured_camera_view(simulation, monkeypatch, side):
    motion, state = simulation
    approach, station = prepare(simulation, monkeypatch, lateral=side*.18,
                                yaw=side*math.radians(10.41), visibility=True)
    assert approach.run(), motion.node.get_logger().error.call_args
    assert abs(station[1]+.03-state.y) <= .025
    assert abs(state.yaw) <= math.radians(2)


def test_trial63_recorded_normal_spike_does_not_jump_the_centerline(simulation, monkeypatch):
    motion, state = simulation
    approach, _ = prepare(simulation, monkeypatch)
    fixture = json.loads((Path(__file__).parent/'fixtures/front_normal_trial63.json').read_text())
    observations = {}
    for seq, sample in enumerate(fixture['samples']):
        state.now = 10.+sample['received_at']-fixture['samples'][0]['received_at']
        motion.node.now_ns = int(state.now*1e9)
        tag = PoseStamped()
        tag.header.frame_id = 'base_link'
        tag.header.stamp = motion.node.get_clock().now().to_msg()
        tag.pose.position.x, tag.pose.position.y = sample['x'], sample['y']
        set_normal(tag, math.radians(sample['heading_deg']))
        motion.node.last_dock_pose, motion.node.last_dock_pose_received_at = tag, state.now
        approach.centerer.lidar.snapshot = lambda: SimpleNamespace(
            sequence=seq, scan=SimpleNamespace(header=tag.header))
        observed = approach.observation(.95)
        if observed is not None:
            observations[seq] = observed
    assert max(p['lateral'] for p in fixture['samples']) > .20
    assert observations[9][1] < .20
    assert abs(observations[9][1]-observations[8][1]) < .01


def test_station_memory_survives_gap_then_updates_gradually(simulation, monkeypatch):
    motion, state = simulation
    approach, _ = prepare(simulation, monkeypatch)
    for _ in range(40):
        rclpy.spin_once(motion.node, timeout_sec=.05)
        errors = approach.observation(.95)
    assert errors is not None
    target = approach.target
    motion.node.last_dock_pose_received_at = 0.
    approach.centerer.lidar.snapshot = lambda: None
    assert approach.observation(.95) is None
    assert approach.target == target
    for sequence in range(100, 105):
        state.now += .1
        motion.node.now_ns = int(state.now*1e9)
        tag = PoseStamped()
        tag.header.frame_id = 'base_link'
        tag.header.stamp = motion.node.get_clock().now().to_msg()
        tag.pose.position.x, tag.pose.position.y = 1.5, .4
        set_normal(tag, math.radians(15))
        motion.node.last_dock_pose = tag
        motion.node.last_dock_pose_received_at = state.now
        approach.centerer.lidar.snapshot = lambda: SimpleNamespace(
            sequence=sequence, scan=SimpleNamespace(header=tag.header))
        reacquired = approach.observation(.95)
    assert 0 < approach.target[2]-target[2] < math.radians(15)
    assert abs(reacquired[1]-errors[1]) > .01
    assert approach.target != target


@pytest.mark.parametrize('scan_age', [.06, .17, .24])
def test_valid_camera_history_pairs_with_delayed_scan_and_available_odom_tf(
        simulation, monkeypatch, scan_age):
    motion, state = simulation
    approach, _ = prepare(simulation, monkeypatch)
    node = motion.node
    node.dock_pose_history = deque(maxlen=5)
    motion.current_xy = lambda: (0., 0.)
    motion.current_yaw = lambda: 0.
    requested = []

    def transform_at(fixed, base, at):
        requested.append(at.nanoseconds)
        if at.nanoseconds > node.now_ns-50_000_000:
            raise TransformException('camera newer than latest odom TF')
        return SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=0., y=0.),
            rotation=SimpleNamespace(x=0., y=0., z=0., w=1.)))

    node.tf_buffer.lookup_transform = transform_at
    for sequence in range(6):
        state.now += .1
        node.now_ns = int(state.now*1e9)
        for age in [.02]:
            tag = PoseStamped()
            tag.header.frame_id = 'base_link'
            tag.header.stamp = node.get_clock().now().to_msg()
            tag.header.stamp = Time(nanoseconds=node.now_ns-int(age*1e9)).to_msg()
            tag.pose.position.x = 1.5
            set_normal(tag, 0.)
            node.dock_pose_history.append((tag, state.now))
        node.last_dock_pose, node.last_dock_pose_received_at = tag, state.now
        scan_header = SimpleNamespace(stamp=Time(nanoseconds=node.now_ns-int(scan_age*1e9)).to_msg())
        approach.centerer.lidar.snapshot = lambda: SimpleNamespace(
            sequence=sequence, scan=SimpleNamespace(header=scan_header))
        errors = approach.observation(.95)
    assert errors == pytest.approx((.55, .03, 0.))
    assert approach.sequence >= 3
    # Acquisition-time selection stays within existing 300ms / 150ms limits.
    assert 0 <= node.now_ns-approach.pair[1] <= 300_000_000
    assert abs(approach.pair[1]-(node.now_ns-int(scan_age*1e9))) <= 150_000_000
    previous = approach.sequence
    assert approach.observation(.95) == pytest.approx(errors)
    assert approach.sequence == previous
    state.now += .4
    node.now_ns = int(state.now*1e9)
    assert approach.observation(.95) is None
    assert 'stale' in approach.reason


def test_trial65_odom_goal_does_not_override_observed_tag_position(simulation, monkeypatch):
    motion, state = simulation
    approach, _ = prepare(simulation, monkeypatch)
    state.x, state.y, state.yaw = -.0056, -1.0832, math.radians(-90.02)
    monkeypatch.setattr(motion, 'current_yaw', lambda: state.yaw)
    monkeypatch.setattr(motion, 'current_xy', lambda: (state.x, state.y))
    motion.node.tf_buffer.lookup_transform = lambda *args: SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=state.x, y=state.y), rotation=SimpleNamespace(
            x=0., y=0., z=math.sin(state.yaw/2), w=math.cos(state.yaw/2))))
    for sequence in range(100, 105):
        state.now += .1
        motion.node.now_ns = int(state.now*1e9)
        tag = PoseStamped()
        tag.header.frame_id = 'base_link'
        tag.header.stamp = motion.node.get_clock().now().to_msg()
        tag.pose.position.x, tag.pose.position.y = .9225, .0042
        set_normal(tag, math.radians(-2.36))
        motion.node.last_dock_pose = tag
        motion.node.last_dock_pose_received_at = state.now
        approach.centerer.lidar.snapshot = lambda: SimpleNamespace(
            sequence=sequence, scan=SimpleNamespace(header=tag.header))
        errors = approach.observation(.95)
    assert errors[1] == pytest.approx(.07218, abs=.001)
    assert abs(errors[0]) > .02
    assert abs(errors[1]) > .025
    assert errors[2] == pytest.approx(math.radians(-2.36))


def test_trial66_near_tag_can_correct_the_initial_far_field_centerline(simulation, monkeypatch):
    motion, state = simulation
    approach, _ = prepare(simulation, monkeypatch)
    rows = json.loads((Path(__file__).parent/'fixtures/front_centerline_trial66.json').read_text())['samples']
    targets = []
    for sequence, row in enumerate(rows):
        state.now = 10.+row['received_at']-rows[0]['received_at']
        motion.node.now_ns = int(state.now*1e9)
        state.x, state.y, state.yaw = row['odom']
        motion.current_yaw = lambda: state.yaw
        motion.current_xy = lambda: (state.x, state.y)
        tag = PoseStamped()
        tag.header.frame_id = 'base_link'
        tag.header.stamp = motion.node.get_clock().now().to_msg()
        tag.pose.position.x, tag.pose.position.y = row['x'], row['y']
        q = tag.pose.orientation
        q.x, q.y, q.z, q.w = row['q']
        motion.node.last_dock_pose = tag
        motion.node.last_dock_pose_received_at = state.now
        motion.node.tf_buffer.lookup_transform = lambda *args: SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=state.x, y=state.y), rotation=SimpleNamespace(
                x=0., y=0., z=math.sin(state.yaw/2), w=math.cos(state.yaw/2))))
        approach.centerer.lidar.snapshot = lambda: SimpleNamespace(
            sequence=sequence, scan=SimpleNamespace(header=tag.header))
        if approach.observation(.95) is not None:
            targets.append(approach.target)
    assert math.degrees(targets[-1][2]-targets[0][2]) < -5
    assert abs(targets[-1][0]-targets[0][0]) > .08
    assert max(abs(b[0]-a[0]) for a, b in zip(targets, targets[1:])) < .025


def test_final_turn_uses_driven_axle_goal_to_avoid_wrong_way_position_chasing():
    v, w = TagCenterlineApproach.command((.001, .026, math.radians(-12.5)),
                                       .10, .08, .15, (.02, .025), math.radians(2))
    assert v == 0.
    assert w < 0  # Turning right moves base onto the goal about the front axle.


@pytest.mark.parametrize('lateral', [-.18, -.05, .05, .18])
@pytest.mark.parametrize('fault', [None, 'motion_camera_delay', 'perspective_bias'])
def test_calibrated_turn_point_converges_with_20mm_lateral_tolerance(
        simulation, monkeypatch, lateral, fault):
    motion, state = simulation
    approach, station = prepare(simulation, monkeypatch, lateral=lateral,
        yaw=math.copysign(math.radians(10.41), lateral), fault=fault,
        visibility=True, lateral_offset=-.02)
    motion.node.parameters['tag_refinement_lateral_tolerance'] = .02
    assert approach.run(), (motion.node.get_logger().error.call_args, [c.args[0] for c in motion.node.get_logger().info.call_args_list[-8:]])
    assert abs(station[1]+.02-state.y) <= .02
    assert abs(state.yaw) <= math.radians(2)


@pytest.mark.parametrize('side', [-1, 1])
def test_axle_inside_tolerance_does_not_stop_with_body_center_outside(side):
    # Turning the remaining <2deg moves the body center the missing few mm.
    v, w = TagCenterlineApproach.command((.015, side*.023, -side*math.radians(1.94)),
        .10, .08, .15, (.02, .02), math.radians(2))
    assert v == 0.
    assert w*side < 0.


@pytest.mark.parametrize('side', [-1, 1])
@pytest.mark.parametrize('tolerance,expected', [(.012, False), (.02, True), (.025, True)])
def test_trial75_final_axle_residual_uses_a_feasible_turn_then_finishes(
        simulation, monkeypatch, side, tolerance, expected):
    # Trial75 at elapsed 25.2s: x=.018m, y=.035m, heading=-7.45deg.
    # The axle is 2.2cm from the target line. A final axle turn places the
    # body within 2.5cm. Bounded line following can also meet a 2cm gate
    # with a short forward move; there is too little runway for 1.2cm here.
    motion, state = simulation
    approach, station = prepare(simulation, monkeypatch,
        lateral=side*.035, runway=.018, yaw=side*math.radians(7.45),
        lateral_offset=-.02, visibility=True)
    motion.node.parameters['tag_refinement_lateral_tolerance'] = tolerance
    assert approach.run() is expected, motion.node.get_logger().error.call_args
    assert state.linear == state.angular == 0
    if expected:
        assert abs(state.yaw) <= math.radians(2)
        assert abs(station[1]+.02-state.y) <= tolerance
        assert abs(state.x-.018) <= .02
        if tolerance == .025:
            assert all(v == 0 for _, _, _, v, w in state.sent)
            assert all(w*side <= 0 for _, _, _, v, w in state.sent)
        assert state.now-10 < 6


@pytest.mark.parametrize('side', [-1, 1])
@pytest.mark.parametrize('x,y,heading_deg', [(.188,.013,-2.67), (.135,.017,-4.16)])
def test_trial69_near_centerline_aligns_heading_before_chasing_goal_bearing(
        side, x, y, heading_deg):
    # Real pre-oscillation errors, and their mirror on the other side.
    heading = side*math.radians(heading_deg)
    v,w = TagCenterlineApproach.command((x, side*y, heading),
                                       .10,.08,.15,(.02,.025),math.radians(2))
    assert v == 0.
    assert w*heading > 0
    assert abs(w) <= .15


@pytest.mark.parametrize('side', [-1, 1])
def test_near_centerline_alignment_keeps_progress_toward_original_turn_distance(side):
    v,w = TagCenterlineApproach.command((.15, side*.015, side*math.radians(1)),
                                       .10,.08,.15,(.02,.025),math.radians(2))
    assert 0 < v <= .08
    assert w*side > 0


def test_trial69_recorded_tag_view_accepts_the_centerline_heading_correction():
    from test_tag_visibility import make_guard
    record = json.loads((Path(__file__).parent/'fixtures/front_centerline_trial69.json').read_text())
    guard, transform = make_guard()
    camera = record['camera_info']
    guard.info.width, guard.info.height, guard.info.p = camera['width'], camera['height'], camera['p']
    tf = record['camera_transform']
    for axis in ('x','y','z'):
        setattr(transform.translation,axis,tf[axis])
    for axis, value in zip(('x','y','z','w'),tf['q']):
        setattr(transform.rotation,axis,value)
    for row in record['samples']:
        tag = PoseStamped()
        for axis in ('x','y','z'):
            setattr(tag.pose.position,axis,row['tag'][axis])
        for axis,value in zip(('x','y','z','w'),row['tag']['q']):
            setattr(tag.pose.orientation,axis,value)
        x,y,heading_deg = row['errors']
        requested = TagCenterlineApproach.command((x,y,math.radians(heading_deg)),
                                                  .10,.08,.15,(.02,.025),math.radians(2))
        limited = guard.limit(tag,*requested,.10,max_angular_speed=.15)
        assert limited == requested
        assert limited[0] == 0 and limited[1] < 0


@pytest.mark.parametrize('side', [-1, 1])
def test_near_centerline_heading_settles_and_reaches_original_standoff(
        simulation, monkeypatch, side):
    motion, state = simulation
    approach, _ = prepare(simulation,monkeypatch,lateral=side*.013,runway=.188,
                          yaw=side*math.radians(2.67),lateral_offset=0.,visibility=True)
    assert approach.run(), motion.node.get_logger().error.call_args
    assert abs(state.x-.188) <= .02
    assert abs(state.y-side*.013) <= .025
    assert abs(state.yaw) <= math.radians(2)
    assert state.linear == state.angular == 0


@pytest.mark.parametrize('lateral', [-.10, .10])
def test_distant_biased_normal_is_refined_before_the_turn_point(simulation, monkeypatch, lateral):
    motion, state = simulation
    approach, _ = prepare(simulation, monkeypatch, lateral=lateral, runway=1.6,
                          yaw=math.radians(5), fault='perspective_bias')
    assert approach.run(), motion.node.get_logger().error.call_args
    assert abs(state.y-lateral) <= .025
    assert abs(state.x-1.6) <= .02
    assert abs(state.yaw) <= math.radians(2)


def test_turn_standoff_cannot_be_reduced_to_fit_the_position_tolerance(simulation, monkeypatch):
    motion, state = simulation
    approach, _ = prepare(simulation, monkeypatch)
    motion.node.parameters['apriltag_front_standoff'] = .60
    assert not approach.run()
    assert all(v == w == 0 for _, _, _, v, w in state.sent)
    assert state.x == state.y == 0.


def test_centered_station_does_not_inherit_the_old_three_centimeter_bias(simulation, monkeypatch):
    motion, state = simulation
    approach, station = prepare(simulation, monkeypatch, lateral=.10, runway=1.0,
                                lateral_offset=0.)
    assert approach.run(), motion.node.get_logger().error.call_args
    assert abs(state.y-station[1]) <= .025
    assert abs(state.yaw) <= math.radians(2)


@pytest.mark.parametrize('side', [-1, 1])
@pytest.mark.parametrize('fault', [None, 'motion_camera_delay'])
@pytest.mark.parametrize('elapsed', [10.8, 11.8, 12.8])
def test_trial86_terminal_poses_follow_line_without_point_pursuit(
        simulation, monkeypatch, side, fault, elapsed):
    record = json.loads((Path(__file__).parent/'fixtures/front_terminal_trial86.json').read_text())
    row = next(r for r in record['samples'] if r['elapsed'] == elapsed)
    motion, state = simulation
    approach, station = prepare(simulation, monkeypatch,
        runway=row['x'], lateral=side*row['y'],
        yaw=-side*math.radians(row['heading_deg']),
        lateral_offset=0., visibility=True, fault=fault)
    motion.node.parameters.update(tag_refinement_lateral_tolerance=.012,
                                  tag_refinement_yaw_tolerance=math.radians(1))
    assert approach.run(), (motion.node.get_logger().error.call_args,
        [c.args[0] for c in motion.node.get_logger().info.call_args_list[-5:]])
    assert abs(state.y-station[1]) <= .012
    assert abs(state.x-(station[0]-.95)) <= .02
    assert abs(state.yaw) <= math.radians(1)
    assert state.now-10 < 30
    assert state.linear == state.angular == 0


@pytest.mark.parametrize('side', [-1, 1])
def test_terminal_side_point_cannot_keep_turning_beyond_line_heading_limit(side):
    # Even with substantial residual lateral error and almost no runway,
    # unwind toward the panel normal instead of the adjacent goal point.
    v, w = TagCenterlineApproach.command(
        (.020, side*.061, -side*math.radians(18)),
        .10, .08, .15, (.02, .012), math.radians(1))
    assert w*side < 0
    assert 0 <= v <= .08


@pytest.mark.parametrize('side', [-1, 1])
@pytest.mark.parametrize('fault', [None, 'motion_camera_delay'])
@pytest.mark.parametrize('elapsed', [19.2, 20.3, 21.3, 22.3, 23.3])
def test_trial92_preserves_runway_for_lateral_convergence(
        simulation, monkeypatch, side, fault, elapsed):
    record = json.loads((Path(__file__).parent/'fixtures/front_terminal_trial92.json').read_text())
    row = next(r for r in record['samples'] if r['elapsed'] == elapsed)
    motion, state = simulation
    approach, station = prepare(simulation, monkeypatch,
        runway=row['x'], lateral=side*row['y'],
        yaw=-side*math.radians(row['heading_deg']),
        lateral_offset=0., visibility=True, fault=fault)
    profile = yaml.safe_load((Path(__file__).parents[1]/'config/docking.yaml').read_text())['dock_turn_backup']['ros__parameters']
    motion.node.parameters.update(tag_refinement_lateral_tolerance=.012,
                                  tag_refinement_yaw_tolerance=math.radians(1),
                                  apriltag_view_margin_fraction=profile['apriltag_view_margin_fraction'])
    result = approach.run()
    if not result:
        print('\n'.join(c.args[0] for c in motion.node.get_logger().info.call_args_list))
    assert result, (motion.node.get_logger().error.call_args,
        [c.args[0] for c in motion.node.get_logger().info.call_args_list[-5:]])
    assert abs(state.y-station[1]) <= .012
    assert abs(state.x-(station[0]-.95)) <= .02
    assert abs(state.yaw) <= math.radians(1)
    assert state.linear == state.angular == 0
