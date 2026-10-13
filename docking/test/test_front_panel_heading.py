"""Panel-heading fusion, acquisition-time TF, and measured front replay."""
import json
import math
from pathlib import Path
import random
from types import SimpleNamespace as S

from docking.lidar_geometry import PlanarTransform,project_scan_to_base,yaw_from_quaternion
from docking.tag_centering import apriltag_normal_yaw
from geometry_msgs.msg import PoseStamped
import pytest
import yaml
from rclpy.time import Time
from tf2_ros import TransformException
from test_apriltag_front_centering import fused_centerer
from test_backup_entry_completion import simulation  # noqa: F401
from test_centerline_approach import prepare,set_normal


@pytest.mark.parametrize('seed', [7,42,123])
def test_recorded_front_panel_normal_is_supported_around_tag_without_using_rails(seed):
    data=json.loads((Path(__file__).parent/'fixtures/front_panel_trial84.json').read_text())
    t=data['transform'];tf=PlanarTransform(t['x'],t['y'],yaw_from_quaternion(*t['q']))
    centerer=fused_centerer([])
    before=random.getstate()
    try:
        random.seed(seed)
        for sequence,row in enumerate(data['scans']):
            centerer.lidar.project=lambda snap,c,w,lo,hi: project_scan_to_base(
                [v if v is not None else math.inf for v in row['ranges']],
                row['angle_min'],row['angle_increment'],row['range_min'],row['range_max'],
                tf,c,w,lo,hi)
            p=row['tag'];tag=PoseStamped()
            tag.pose.position.x,tag.pose.position.y=p['x'],p['y']
            q=tag.pose.orientation;q.x,q.y,q.z,q.w=p['q']
            assert abs(apriltag_normal_yaw(tag)) < math.radians(.3)
            normal=centerer.panel(S(sequence=sequence),tag)
            assert normal is not None
            assert math.radians(4.3) < normal < math.radians(5.3)
            assert (tag.pose.position.x,tag.pose.position.y)==(p['x'],p['y'])
    finally:
        random.setstate(before)


@pytest.mark.parametrize('missing_scan_tf', [False,True])
def test_panel_uses_scan_time_yaw_while_tag_center_uses_camera_time(
        simulation,monkeypatch,missing_scan_tf):
    motion,state=simulation
    approach,_=prepare(simulation,monkeypatch,lateral_offset=0.)
    node=motion.node
    node.parameters['tag_front_use_lidar_panel_heading']=True
    motion.current_xy=lambda:(0.,0.)
    motion.current_yaw=lambda:0.
    motion.current_odom_yaw_rate=lambda:0.
    normal=math.radians(6)
    yaw=math.radians(.75)
    camera_stamp,scan_stamp=0,0
    def transform_at(fixed,base,at):
        if at.nanoseconds==scan_stamp:
            if missing_scan_tf:raise TransformException('scan-time odom unavailable')
            a=0.
        else:a=yaw
        return S(transform=S(translation=S(x=0.,y=0.),
            rotation=S(x=0.,y=0.,z=math.sin(a/2),w=math.cos(a/2))))
    node.tf_buffer.lookup_transform=transform_at
    approach.centerer.panel=lambda scan,tag:normal
    errors=None
    for sequence in range(6):
        state.now+=.1;node.now_ns=int(state.now*1e9)
        camera_stamp,scan_stamp=node.now_ns-30_000_000,node.now_ns-130_000_000
        tag=PoseStamped();tag.header.frame_id='base_link'
        tag.header.stamp=Time(nanoseconds=camera_stamp).to_msg()
        tag.pose.position.x=1.5*math.cos(yaw)+.1*math.sin(yaw)
        tag.pose.position.y=-1.5*math.sin(yaw)+.1*math.cos(yaw)
        set_normal(tag,normal+math.radians(4.5))
        node.last_dock_pose,node.last_dock_pose_received_at=tag,state.now
        approach.centerer.lidar.snapshot=lambda:S(sequence=sequence,
            scan=S(header=S(stamp=Time(nanoseconds=scan_stamp).to_msg())))
        errors=approach.observation(.95)
    if missing_scan_tf:
        assert errors is None
        assert 'front scan acquisition time' in approach.reason
    else:
        assert approach.station==pytest.approx((1.5,.1,normal))
        assert approach.target==pytest.approx((1.5-.95*math.cos(normal),
                                              .1-.95*math.sin(normal),normal))


@pytest.mark.parametrize('side', [-1,1])
def test_single_approach_uses_panel_normal_despite_persistent_tag_angle_bias(
        simulation,monkeypatch,side):
    motion,state=simulation
    approach,station=prepare(simulation,monkeypatch,lateral=side*.18,
        yaw=side*math.radians(10.41),visibility=True,lateral_offset=0.,
        fault='motion_camera_delay')
    node=motion.node;node.parameters['tag_front_use_lidar_panel_heading']=True
    original=approach.centerer.lidar.snapshot
    def snapshot():
        scan=original()
        set_normal(node.last_dock_pose,apriltag_normal_yaw(node.last_dock_pose)+math.radians(4.5))
        return scan
    approach.centerer.lidar.snapshot=snapshot
    def panel(scan,tag):
        q=node.tf_buffer.lookup_transform('odom','base_link',
            Time.from_msg(scan.scan.header.stamp)).transform.rotation
        return -yaw_from_quaternion(q.x,q.y,q.z,q.w)
    approach.centerer.panel=panel
    assert approach.run(),node.get_logger().error.call_args
    assert abs(state.y-station[1]) <= .025
    assert abs(state.x-(station[0]-.95)) <= .02
    assert abs(state.yaw) <= math.radians(2)
    assert state.linear==state.angular==0


@pytest.mark.parametrize('lateral', [-.18, -.05, .05, .18])
@pytest.mark.parametrize('fault', [None, 'motion_camera_delay', 'perspective_bias'])
def test_runtime_turn_point_leaves_margin_before_spin(
        simulation, monkeypatch, lateral, fault):
    # Trial85 stopped at y=19mm under the former 25mm acceptance gate,
    # then had measured side gaps of 2cm/8cm. Exercise the tighter installed
    # profile from both sides, with axle motion, camera view and latency.
    motion, state = simulation
    approach, station = prepare(simulation, monkeypatch, lateral=lateral,
        runway=.69, yaw=math.copysign(math.radians(10.41), lateral),
        visibility=True, lateral_offset=0., fault=fault)
    config = yaml.safe_load((Path(__file__).parents[1]/'config/docking.yaml').read_text())
    profile = config['dock_turn_backup']['ros__parameters']
    node = motion.node
    for name in ('tag_refinement_lateral_tolerance', 'tag_refinement_yaw_tolerance',
                 'tag_front_use_lidar_panel_heading'):
        node.parameters[name] = profile[name]
    original = approach.centerer.lidar.snapshot

    def snapshot():
        scan = original()
        set_normal(node.last_dock_pose,
                   apriltag_normal_yaw(node.last_dock_pose)+math.radians(4.5))
        return scan

    approach.centerer.lidar.snapshot = snapshot

    def panel(scan, tag):
        q = node.tf_buffer.lookup_transform('odom', 'base_link',
            Time.from_msg(scan.scan.header.stamp)).transform.rotation
        return -yaw_from_quaternion(q.x, q.y, q.z, q.w)

    approach.centerer.panel = panel
    assert approach.run(), node.get_logger().error.call_args
    assert abs(state.y-station[1]) <= .012
    assert abs(state.x-(station[0]-.95)) <= .02
    assert abs(state.yaw) <= math.radians(1)
    assert state.now-10 < 30
    assert state.linear == state.angular == 0
