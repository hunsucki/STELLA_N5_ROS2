"""Regressions for captured v3 failures without relaxing physical limits."""
import json
import math
from pathlib import Path
from types import SimpleNamespace as S

import pytest
import rclpy
from sensor_msgs.msg import Imu

from docking.control import straight_entry_path
from test_backup_entry_completion import simulation  # noqa: F401
from test_tag_visibility import make_guard, observed_tag

FIXTURES = Path(__file__).parent/'fixtures'


@pytest.mark.parametrize('fault', [None, 'scan', 'imu', 'cancel'])
def test_backup_start_waits_stopped_for_new_feedback(simulation, monkeypatch, fault):
    motion, state = simulation
    original_snapshot = motion.lidar.snapshot
    motion.lidar.snapshot = lambda: (
        None if fault == 'scan' or state.now < 10.25 else original_snapshot())
    motion.imu_is_fresh = lambda: fault != 'imu' and state.now >= 10.15
    motion.should_stop = lambda: fault == 'cancel' and state.now >= 10.5
    original_spin = rclpy.spin_once

    def spin(node, timeout_sec):
        original_spin(node, timeout_sec)
        imu = Imu()
        imu.header.stamp = motion.node.get_clock().now().to_msg()
        motion._imu_callback(imu)

    monkeypatch.setattr(rclpy, 'spin_once', spin)
    assert motion.backup_with_lidar() is (fault is None)
    assert state.linear == state.angular == 0
    if fault is None:
        assert all(v == w == 0 for t, _, _, v, w in state.sent if t < 10.5)
        assert state.gap+state.x <= .0245
    else:
        assert all(v == w == 0 for _, _, _, v, w in state.sent)
        assert state.now <= 12.05


@pytest.mark.parametrize('sign', [-1, 1])
def test_recorded_trial008_signed_pose_preserves_uncertainty_and_body_margin(sign):
    record = json.loads((FIXTURES/'conservative_path_trial008.json').read_text())
    observations = [(sign*c,w,sign*h) for c,w,h in record['observations']]
    anchor = sign*record['anchor_heading']
    path = straight_entry_path(observations,.317,.22,.02,math.radians(3),-.2295,.54,.01,anchor)
    assert path is not None
    samples = record['samples']
    origin = samples[0]
    yaw = record['anchor_yaw']
    previous_heading = anchor
    previous = origin
    progress = uncertainty = legacy_deviation = 0.
    minimum = 1.
    for row in samples[1:]:
        heading = anchor-sign*(row['gyro']-origin['gyro'])
        dx,dy=row['x']-previous['x'],row['y']-previous['y']
        step=math.hypot(dx,dy)
        side=sign*(-dx*math.sin(yaw)+dy*math.cos(yaw))
        before,after=previous_heading-anchor,heading-anchor
        predicted=step*.5*(math.sin(before)+math.sin(after))
        uncertainty += (abs(side-predicted)+step*.5*abs(math.sin(after)-math.sin(before))
                        +step*max(0.,1-min(math.cos(before),math.cos(after))))
        legacy_deviation += max(abs(side),step*max(abs(math.sin(before)),abs(math.sin(after))))
        legacy_deviation += step*max(0.,1-min(math.cos(before),math.cos(after)))
        progress += step
        dx,dy=row['x']-origin['x'],row['y']-origin['y']
        displacement=(-dx*math.cos(yaw)-dy*math.sin(yaw),
                      sign*(-dx*math.sin(yaw)+dy*math.cos(yaw)))
        remaining=max(.081,.317-progress)
        current=path.clearance(heading,progress,remaining,uncertainty,displacement=displacement)
        minimum=min(minimum,current)
        # Independent brute force corners across the angular uncertainty:
        # the analytical sweep must not overstate their clearance.
        for center,_,observed in observations:
            offset=displacement[0]*math.sin(observed)+displacement[1]*math.cos(observed)-center
            for delta in (-math.radians(.1),0.,math.radians(.1)):
                angle=heading+observed-anchor+delta
                for future in (0.,remaining):
                    for body_x in (-.2295,.2295):
                        for body_y in (-.22,.22):
                            corner=offset+(future-body_x)*math.sin(angle)+body_y*math.cos(angle)
                            assert current <= .265-abs(corner)-.003-uncertainty-progress*math.sin(math.radians(.1))+1e-10
        previous,previous_heading=row,heading
    assert minimum >= .02
    assert uncertainty > 0.  # Observed position never erases accumulated uncertainty.
    # Recorder interpolation differs slightly from the node's samples. The
    # logged stop state, with adjacent fitted observations, reproduces the
    # boundary rejection; the resampled path remains close to that boundary.
    assert abs(path.clearance(heading,progress,.081,legacy_deviation)-.02) < .001
    stop=record['logged_stop']
    assert path.clearance(sign*stop['heading'],stop['progress'],stop['remaining'],stop['deviation']) < .02
    # The same yaw with a real 4cm side shift or >3deg still cannot authorize entry.
    assert path.clearance(heading,progress,.081,uncertainty,
                          displacement=(displacement[0],sign*.04)) < .02
    assert path.clearance(sign*math.radians(3.1),progress,.081,uncertainty,
                          displacement=displacement) == -math.inf


def test_recorded_trial013_actual_outward_yaw_pauses_while_still_visible(monkeypatch):
    from geometry_msgs.msg import PoseStamped
    record=json.loads((FIXTURES/'conservative_front_trial013.json').read_text())
    clock=[record['samples'][0]['tag']['stamp']]
    monkeypatch.setattr('time.monotonic',lambda:clock[0])
    guard,transform=make_guard()
    baseline,baseline_transform=make_guard()
    t=record['transform']
    for tf in (transform,baseline_transform):
        for axis in ('x','y','z'):setattr(tf.translation,axis,t[axis])
        for axis,value in zip(('x','y','z','w'),t['q']):setattr(tf.rotation,axis,value)
    info=record['calibration']
    guard.info.p=info['p']
    baseline.info.p=info['p']
    held=[]
    for index,sample in enumerate(record['samples']):
        clock[0]=sample['tag']['stamp']
        guard.received_at=baseline.received_at=clock[0]
        tag=PoseStamped()
        for axis in ('x','y','z'):setattr(tag.pose.position,axis,sample['tag'][axis])
        for axis,value in zip(('x','y','z','w'),sample['tag']['q']):setattr(tag.pose.orientation,axis,value)
        command=sample['cmd']
        without_feedback=baseline.limit(tag,command['vx'],command['wz'],.1)
        assert without_feedback is not None and without_feedback[0]>0
        guard.node.motion=S(imu_is_fresh=lambda:True,current_imu_yaw_rate=lambda:sample['imu']['wz'])
        result=guard.limit(tag,command['vx'],command['wz'],.1)
        if index == 0 or clock[0]-record['samples'][1]['tag']['stamp'] < .2:
            # Far from edge, or a brief normal response lag: preserve command.
            assert result == without_feedback
        else:
            assert result == (0.,0.) and guard.last_limited
            held.append(clock[0])
    assert held and held[0]<record['samples'][-1]['tag']['stamp']
    # Settling alone must not resume translation with the tag still at the edge.
    guard.node.motion.current_imu_yaw_rate=lambda:0.
    result=guard.limit(tag,command['vx'],command['wz'],.1)
    assert result is not None and result[0]==0. and result[1]<0.


@pytest.mark.parametrize('side', [-1,1])
def test_measured_inward_rotation_keeps_normal_visibility_control(side):
    guard,_=make_guard()
    tag=observed_tag(side*math.radians(15))
    expected=guard.limit(tag,.04,side*.08,.1)
    guard.node.motion=S(imu_is_fresh=lambda:True,current_imu_yaw_rate=lambda:side*.08)
    assert guard.limit(tag,.04,side*.08,.1)==expected


@pytest.mark.parametrize('sign', [-1,1])
def test_nominal_straight_ray_keeps_existing_clearance(sign):
    heading=sign*math.radians(2)
    path=straight_entry_path([(sign*.01,.54,heading)]*5,.16,.22,.02,
                             math.radians(3),-.2295,.54,.01,heading)
    assert path is not None
    assert path.clearance(heading,.08,.08,.001,displacement=(.08,0.)) == pytest.approx(
        path.clearance(heading,.08,.08,.001))


@pytest.mark.parametrize('displacement', [(math.nan,0.),(0.,math.inf),(.2,0.),(0.,),()])
def test_invalid_signed_position_cannot_authorize_motion(displacement):
    path=straight_entry_path([(0.,.54,0.)]*5,.32,.22,.02,math.radians(3),-.2295,.54,.01,0.)
    assert path is not None
    assert path.clearance(0.,.1,.22,displacement=displacement)==-math.inf
