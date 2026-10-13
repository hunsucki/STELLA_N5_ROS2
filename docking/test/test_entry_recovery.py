"""Repositioning keeps the full entry margin and observes the swept path."""
import math
from types import SimpleNamespace

import pytest
from docking.entry_recovery import (
    DockEntryRecovery, interpolate_path, path_is_clear, reposition_plan)


@pytest.mark.parametrize('offset', [-.033,.033])
def test_plan_retreats_before_lateral_motion_and_stays_bounded(offset):
    plan=reposition_plan(offset)
    assert plan[0]==(.10,0,0)
    assert plan[-1][1]*offset>0
    assert abs(plan[-1][1])<=.012
    assert plan[-1][0]==.215 and plan[-1][2]==0
    assert max(abs(p[2]) for p in plan)<math.radians(6)
    assert sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip([(0,0,0)]+plan,plan))<.216


@pytest.mark.parametrize('offset', [0,.01,.046,.06,math.nan,math.inf])
def test_uncertain_or_excessive_offset_does_not_generate_a_plan(offset):
    assert reposition_plan(offset) is None


@pytest.mark.parametrize('offset', [-.041,.041,-.045,.045])
def test_trial36_larger_offset_plan_establishes_two_centimeter_margin(offset):
    plan = reposition_plan(offset)
    assert plan[0] == (.10, 0., 0.)
    assert plan[-1][0] == .28 and plan[-1][2] == 0.
    assert max(abs(p[2]) for p in plan) < math.radians(6)
    assert .54 / 2 - abs(offset - plan[-1][1]) - .22 >= .02
    assert sum(math.hypot(b[0]-a[0], b[1]-a[1])
               for a,b in zip([(0,0,0)] + plan,plan)) < .281


def test_straight_escape_can_increase_small_rear_clearance_but_cannot_move_closer():
    obstacle=[(-.25,-.225)]
    assert path_is_clear(obstacle,interpolate_path((0,0,0),[(.1,0,0)]),True)
    assert not path_is_clear(obstacle,interpolate_path((0,0,0),[(-.01,0,0)]),True)
    assert not path_is_clear([(-.22,-.21)],[(.01,0,0)],True)


def test_rotation_path_checks_corner_sweep_even_when_endpoint_is_clear():
    obstacle=[(.27,-.18)]
    path=interpolate_path((0,0,0),[(0,0,.3),(0,0,0)])
    assert not path_is_clear(obstacle,path)
    assert path_is_clear(obstacle,[(0,0,0)])


def test_clear_front_endpoint_does_not_hide_obstacle_on_route():
    assert not path_is_clear([(.28,0)],interpolate_path((0,0,0),[(.6,0,0)]),True)


def sensor(ranges):
    return (SimpleNamespace(transform=SimpleNamespace(x=.202,y=0,yaw=math.pi)),
            SimpleNamespace(scan=SimpleNamespace(ranges=ranges,angle_min=-math.pi,
                angle_increment=2*math.pi/719,range_min=.05,range_max=16)))


def test_wraparound_front_beams_and_only_small_bracketed_angular_holes():
    ranges=[2.0]*720
    assert DockEntryRecovery._visible(.6,0,[sensor(ranges)])
    for index in [0,1,719]:
        values=list(ranges);values[index]=math.inf
        assert DockEntryRecovery._visible(.6,0,[sensor(values)])
    values=list(ranges)
    for index in [716,717,718,719,0,1,2,3,4]:
        values[index]=math.inf
    assert not DockEntryRecovery._visible(.6,0,[sensor(values)])
    values=list(ranges);values[0]=.1
    assert not DockEntryRecovery._visible(.6,0,[sensor(values)])


def test_two_lidars_do_not_infer_free_space_from_invalid_returns():
    assert not DockEntryRecovery._visible(.6,0,[sensor([math.inf]*720)]*2)
    assert not DockEntryRecovery._visible(.6,0,[sensor([.1]*720)]*2)


from test_backup_entry_completion import simulation


@pytest.mark.parametrize('seed', [7, 42, 123])
def test_trial47_recorded_guide_jitter_acquires_a_sliding_stationary_window(monkeypatch, seed):
    import json
    import random
    from pathlib import Path
    from unittest.mock import Mock
    from docking.lidar_alignment import LidarPlaneAligner
    from docking.lidar_geometry import PlanarTransform, project_scan_to_base, yaw_from_quaternion
    import yaml
    data = json.loads((Path(__file__).parent/'fixtures'/'entry_acquisition_trial47.json').read_text())
    state = SimpleNamespace(now=10.)
    monkeypatch.setattr('time.monotonic', lambda: state.now)
    monkeypatch.setattr('docking.entry_recovery.spin_for',
                        lambda node, dt, stop: setattr(state, 'now', state.now+dt))
    parameters = {'backup_rear_reference_x': -.2295}
    node = SimpleNamespace(declare_parameter=lambda n,v: parameters.setdefault(n,v),
        get_parameter=lambda n: SimpleNamespace(value=parameters[n]),
        should_stop=lambda: False, get_logger=lambda: Mock())
    LidarPlaneAligner.declare_parameters(node)
    parameters.update(yaml.safe_load((Path(__file__).parents[1]/'config'/'docking.yaml').read_text())
                      ['dock_turn_backup']['ros__parameters'])
    m = data['transform']
    transform = PlanarTransform(m['x'],m['y'],yaw_from_quaternion(*m['q']))

    def snapshot():
        index = int((state.now-10.)/.1)
        if index >= len(data['scans']):
            return None
        return SimpleNamespace(sequence=index, row=data['scans'][index])

    def project(scan, center, width, minimum, maximum):
        r = scan.row
        return project_scan_to_base([v if v is not None else math.inf for v in r['ranges']],
            r['angle_min'],r['angle_increment'],r['range_min'],r['range_max'],
            transform,center,width,minimum,maximum)

    node.lidar = SimpleNamespace(snapshot=snapshot, project=project)
    motion = SimpleNamespace(is_stationary=lambda _: True, current_odom_yaw_rate=lambda: 0.)
    node.lidar_aligner = LidarPlaneAligner(node,motion,node.lidar)
    recovery = DockEntryRecovery.__new__(DockEntryRecovery)
    recovery.node,recovery.motion = node,motion
    rng = random.getstate()
    try:
        random.seed(seed)
        result = recovery.observations()
    finally:
        random.setstate(rng)
    assert result is not None
    offset,width = result
    assert -.046 < offset < -.026
    assert .50 <= width <= .58
    assert state.now <= 12.01


def test_repeated_single_scan_cannot_verify_entry_pair(monkeypatch):
    from unittest.mock import Mock
    state = SimpleNamespace(now=10.)
    monkeypatch.setattr('time.monotonic',lambda:state.now)
    monkeypatch.setattr('docking.entry_recovery.spin_for',
                        lambda node,dt,stop:setattr(state,'now',state.now+dt))
    recovery = DockEntryRecovery.__new__(DockEntryRecovery)
    recovery.motion = SimpleNamespace(is_stationary=lambda _:True,current_odom_yaw_rate=lambda:0.)
    recovery.node = SimpleNamespace(should_stop=lambda:False,get_logger=lambda:Mock(),
        lidar=SimpleNamespace(snapshot=lambda:SimpleNamespace(sequence=1)),
        lidar_aligner=SimpleNamespace(estimate_error=lambda _:(0.,0.,100,100,.5),
            estimate_guide_center=lambda _:SimpleNamespace(center_offset=-.038,separation=.54)))
    assert recovery.observations() is None


@pytest.mark.parametrize('offset', [-.033,.033])
def test_complete_reposition_loop_establishes_margin_before_allowing_backup(simulation,offset):
    motion,state=simulation
    state.gap=.371
    motion.node.parameters['use_lidar_guide_centering']=True
    motion.node.parameters['use_dock_entry_recovery']=True
    motion.node.should_stop=motion.should_stop
    motion.node.lidar=motion.lidar
    recovery=DockEntryRecovery.__new__(DockEntryRecovery)
    recovery.node=motion.node
    recovery.motion=motion
    recovery.observations=lambda:(offset-state.y,.54)
    recovery.clear=lambda path,escape:True
    assert recovery.run()
    assert state.x==pytest.approx(.215,abs=.008)
    assert state.y*offset>0
    assert abs(offset-state.y)<.025
    assert abs(state.yaw)<math.radians(1)
    assert all(v>=0 for _,_,_,v,_ in state.sent)
    assert state.linear==state.angular==0
    assert state.now<35


@pytest.mark.parametrize('offset', [-.041,.041])
def test_trial36_reposition_loop_uses_bounded_longer_return_distance(simulation, offset):
    motion,state=simulation
    state.gap=.435
    motion.node.parameters.update(use_lidar_guide_centering=True,
                                  use_dock_entry_recovery=True, backup_max_travel=.75)
    motion.node.should_stop=motion.should_stop
    motion.node.lidar=motion.lidar
    recovery=DockEntryRecovery.__new__(DockEntryRecovery)
    recovery.node=motion.node
    recovery.motion=motion
    recovery.observations=lambda:(offset-state.y,.54)
    recovery.clear=lambda path,escape:True
    assert recovery.run()
    assert state.x == pytest.approx(.28, abs=.008)
    assert .54/2 - abs(offset-state.y) - .22 >= .02
    assert state.linear == state.angular == 0
    assert all(v>=0 for _,_,_,v,_ in state.sent)
    assert state.now<35


def test_reposition_rejects_unobserved_space_without_moving(simulation):
    motion,state=simulation
    state.gap=.371
    motion.node.parameters.update(use_lidar_guide_centering=True,use_dock_entry_recovery=True)
    motion.node.should_stop=motion.should_stop
    motion.node.lidar=motion.lidar
    recovery=DockEntryRecovery.__new__(DockEntryRecovery)
    recovery.node=motion.node
    recovery.motion=motion
    recovery.observations=lambda:(.033,.54)
    recovery.clear=lambda path,escape:False
    assert not recovery.run()
    assert not state.sent


def test_reposition_does_not_allow_reverse_if_lateral_error_did_not_improve(simulation):
    motion,state=simulation
    state.gap=.371
    motion.node.parameters.update(use_lidar_guide_centering=True,use_dock_entry_recovery=True)
    motion.node.should_stop=motion.should_stop
    motion.node.lidar=motion.lidar
    recovery=DockEntryRecovery.__new__(DockEntryRecovery)
    recovery.node=motion.node
    recovery.motion=motion
    recovery.observations=lambda:(.033,.54)
    recovery.clear=lambda path,escape:True
    assert not recovery.run()
    assert all(v>=0 for _,_,_,v,_ in state.sent)
    assert state.linear==state.angular==0


@pytest.mark.parametrize('fault', ['stop_request','stale_imu','new_obstacle'])
def test_live_fault_during_reposition_stops_without_reversing(simulation,fault):
    motion,state=simulation
    state.gap=.371
    motion.node.parameters.update(use_lidar_guide_centering=True,use_dock_entry_recovery=True)
    motion.node.should_stop=lambda:fault=='stop_request' and state.now>11
    motion.should_stop=motion.node.should_stop
    motion.node.lidar=motion.lidar
    if fault=='stale_imu':
        motion.imu_is_fresh=lambda:state.now<=11
    recovery=DockEntryRecovery.__new__(DockEntryRecovery)
    recovery.node=motion.node
    recovery.motion=motion
    recovery.observations=lambda:(.033-state.y,.54)
    recovery.clear=lambda path,escape:fault!='new_obstacle' or state.now<=11
    assert not recovery.run()
    assert 0<state.x<.05
    assert state.linear==state.angular==0
    assert all(v>=0 for _,_,_,v,_ in state.sent)


@pytest.mark.parametrize('disabled', ['use_dock_entry_recovery','use_lidar_backup','use_lidar_guide_centering'])
def test_disabled_or_legacy_modes_do_not_start_recovery(simulation,disabled):
    motion,state=simulation
    motion.node.parameters.update(use_lidar_guide_centering=True,use_dock_entry_recovery=True)
    motion.node.parameters[disabled]=False
    recovery=DockEntryRecovery.__new__(DockEntryRecovery)
    recovery.node=motion.node
    recovery.observations=lambda:pytest.fail('disabled recovery must not acquire or drive')
    assert recovery.run()
    assert not state.sent


def test_sufficient_original_margin_skips_repositioning(simulation):
    motion,state=simulation
    motion.node.parameters.update(use_lidar_guide_centering=True,use_dock_entry_recovery=True)
    recovery=DockEntryRecovery.__new__(DockEntryRecovery)
    recovery.node=motion.node
    recovery.observations=lambda:(.005,.54)
    assert recovery.run()
    assert not state.sent


@pytest.mark.parametrize('offset', [-.020, .020])
def test_measured_width_prevents_unnecessary_recovery_from_narrow_rail_fit(simulation, offset):
    motion, state = simulation
    motion.node.parameters.update(use_lidar_guide_centering=True, use_dock_entry_recovery=True,
                                  backup_guide_known_width=.54,
                                  backup_guide_known_width_uncertainty=.01)
    recovery = DockEntryRecovery.__new__(DockEntryRecovery)
    recovery.node, recovery.motion = motion.node, motion
    recovery.observations = lambda: (offset, .50)
    recovery.clear = lambda *args: pytest.fail('sufficient calibrated margin must skip recovery')
    assert recovery.run()
    assert not state.sent
    assert 'reposition_required=False' in motion.node.get_logger().info.call_args.args[0]


@pytest.mark.parametrize('known_width', [0., .54])
def test_real_margin_shortfall_still_requires_a_verified_escape_path(simulation, known_width):
    motion, state = simulation
    state.gap = .371
    motion.node.parameters.update(use_lidar_guide_centering=True, use_dock_entry_recovery=True,
                                  backup_guide_known_width=known_width)
    motion.node.lidar = motion.lidar
    recovery = DockEntryRecovery.__new__(DockEntryRecovery)
    recovery.node, recovery.motion = motion.node, motion
    recovery.observations = lambda: (.033, .54)
    recovery.clear = lambda *args: False
    assert not recovery.run()
    assert not state.sent
    assert 'reposition_required=True' in motion.node.get_logger().info.call_args.args[0]


@pytest.mark.parametrize('known,uncertainty,offset,width', [
    (.49, .01, .02, .54), (.54, .001, .02, .54),
    (math.nan, .01, .02, .54), (.54, .01, math.nan, .54),
    (.54, .01, .02, .49), (.54, .01, .02, math.inf),
])
def test_invalid_calibration_or_observation_cannot_skip_entry_checks(
        simulation, known, uncertainty, offset, width):
    motion, state = simulation
    motion.node.parameters.update(use_lidar_guide_centering=True, use_dock_entry_recovery=True,
                                  backup_guide_known_width=known,
                                  backup_guide_known_width_uncertainty=uncertainty)
    recovery = DockEntryRecovery.__new__(DockEntryRecovery)
    recovery.node, recovery.motion = motion.node, motion
    recovery.observations = lambda: (offset, width)
    assert not recovery.run()
    assert not state.sent


def test_visibility_failure_reports_missing_neighbors_without_assuming_free_space():
    values = [math.inf] * 720
    values[0] = 2.
    diagnostics = []
    assert not DockEntryRecovery._visible(.6, 0, [sensor(values)], diagnostics)
    assert diagnostics[0]['reason'] == 'missing_neighbor_returns'
    assert diagnostics[0]['center'] == 2.
