"""Recorded loss of view must not restart forward motion at the image edge."""
from copy import deepcopy
import json
import math
from pathlib import Path
from types import SimpleNamespace as S

from geometry_msgs.msg import PoseStamped
import pytest

from docking.tag_visibility import image_bounds
from test_tag_visibility import make_guard, observed_tag


def recorded_guard(monkeypatch):
    data=json.loads((Path(__file__).parent/'fixtures/front_view_recovery_right002.json').read_text())
    clock=[data['samples'][0]['tag']['stamp']]
    monkeypatch.setattr('time.monotonic',lambda:clock[0])
    guard,transform=make_guard()
    get=guard.node.get_parameter
    guard.node.get_parameter=lambda name:S(value=.05) if name=='apriltag_view_margin_fraction' else get(name)
    for axis in ('x','y','z'):setattr(transform.translation,axis,data['transform'][axis])
    for axis,value in zip(('x','y','z','w'),data['transform']['q']):setattr(transform.rotation,axis,value)
    guard.info.p=data['calibration']['p']
    rate=[0.]
    guard.node.motion=S(imu_is_fresh=lambda:True,current_imu_yaw_rate=lambda:rate[0])
    for row in data['samples']:
        clock[0]=row['tag']['stamp']
        guard.received_at=clock[0]
        tag=PoseStamped()
        for axis in ('x','y','z'):setattr(tag.pose.position,axis,row['tag'][axis])
        for axis,value in zip(('x','y','z','w'),row['tag']['q']):setattr(tag.pose.orientation,axis,value)
        rate[0]=row['imu']['wz']
        result=guard.limit(tag,row['cmd']['vx'],row['cmd']['wz'],.1)
        if getattr(guard,'recovering_view',False):
            assert result is not None and result[0]==0.
    assert guard.recovering_view
    return guard,transform,tag,clock,rate


def test_recorded_right002_zero_gyro_does_not_restart_forward_motion(monkeypatch):
    guard,transform,tag,clock,rate=recorded_guard(monkeypatch)
    rate[0]=0.
    # This is the transition that previously resumed forward motion at ~633px.
    result=guard.limit(tag,.073,-.15,.1)
    assert result is not None and result[0]==0. and result[1]<0.
    before=image_bounds(tag,transform,guard.info,.154)
    after=image_bounds(tag,transform,guard.info,.154,*result,1.,.1)
    assert before[1]>630 and after[1]<before[1]
    # A stop/verification call and a brief reversal do not clear recovery.
    assert guard.limit(tag,0.,0.,.1)==(0.,0.)
    rate[0]=.10
    assert guard.limit(tag,.08,-.15,.1)==(0.,0.)
    rate[0]=0.
    assert guard.limit(tag,.08,-.15,.1)[0]==0.


def turn_tag(tag,angle,axle):
    result=deepcopy(tag)
    p=result.pose.position
    c,s=math.cos(angle),math.sin(angle)
    p.x,p.y=axle+c*(p.x-axle)+s*p.y,-s*(p.x-axle)+c*p.y
    q=result.pose.orientation
    c,s=math.cos(angle/2),math.sin(angle/2)
    q.x,q.y,q.z,q.w=c*q.x+s*q.y,c*q.y-s*q.x,c*q.z-s*q.w,c*q.w+s*q.z
    return result


def test_recorded_view_recovers_by_rotation_then_releases_translation(monkeypatch):
    guard,transform,tag,clock,rate=recorded_guard(monkeypatch)
    rate[0]=0.
    stopped_turns=0
    for _ in range(120):
        clock[0]+=.05
        guard.received_at=clock[0]
        result=guard.limit(tag,.073,-.05,.1)
        assert result is not None
        if result[0]>0.:
            bounds=image_bounds(tag,transform,guard.info,.154)
            assert bounds[1]<=576. and stopped_turns>0 and not guard.recovering_view
            break
        assert result[1]<0.  # Turn inward without translation.
        stopped_turns+=1
        rate[0]=result[1]
        tag=turn_tag(tag,result[1]*.05,.1)
    else:
        pytest.fail('Visible inward rotation must recover the existing view margin')


@pytest.mark.parametrize('fault',['imu','calibration','cropped'])
def test_recovery_never_bypasses_fresh_feedback_or_full_tag_visibility(monkeypatch,fault):
    guard,_,tag,_,rate=recorded_guard(monkeypatch)
    rate[0]=0.
    if fault=='imu':guard.node.motion.imu_is_fresh=lambda:False
    elif fault=='calibration':guard.info=None
    else:tag=observed_tag(math.radians(-35))
    assert guard.limit(tag,.08,-.15,.1) is None


@pytest.mark.parametrize('side',[-1,1])
def test_untriggered_visible_motion_keeps_existing_command(side):
    guard,_=make_guard()
    tag=observed_tag(side*math.radians(5))
    expected=guard.limit(tag,.08,side*.025,.1)
    guard.node.motion=S(imu_is_fresh=lambda:True,current_imu_yaw_rate=lambda:side*.025)
    assert guard.limit(tag,.08,side*.025,.1)==expected
