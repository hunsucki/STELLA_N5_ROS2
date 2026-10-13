"""Temporary motion feedback loss stops and reacquires within the same budget."""
import math

import pytest
import rclpy

from docking.motion import MotionController
from test_backup_entry_completion import simulation  # noqa: F401
from test_centerline_approach import prepare


@pytest.mark.parametrize('sensor', ['odom','imu','wheel'])
@pytest.mark.parametrize('fault', ['delivery','old_header','future_header'])
def test_front_feedback_gap_holds_zero_then_resumes(simulation,monkeypatch,sensor,fault):
    motion,state = simulation
    approach,station = prepare(simulation,monkeypatch,lateral=.08,lateral_offset=0.)
    method = {'odom':'odom_is_fresh','imu':'imu_is_fresh','wheel':'wheel_yaw_is_fresh'}[sensor]
    stamp_attr = {'odom':'last_odom_stamp_nanoseconds','imu':'last_imu_stamp_nanoseconds',
                  'wheel':'last_wheel_yaw_stamp_nanoseconds'}[sensor]
    begins, ends = 10.8, 12.0
    if fault == 'delivery':
        fresh = getattr(motion,method)
        setattr(motion,method,lambda:not begins <= state.now < ends and fresh())
    else:
        setattr(motion,method,getattr(MotionController,method).__get__(motion))
        spin = rclpy.spin_once
        def delayed(node,timeout_sec):
            spin(node,timeout_sec)
            if begins <= state.now < ends:
                offset = -1 if fault == 'old_header' else 1
                setattr(motion,stamp_attr,int((state.now+offset)*1e9))
        monkeypatch.setattr(rclpy,'spin_once',delayed)
    assert approach.run(),motion.node.get_logger().error.call_args
    during = [r for r in state.sent if begins+.06 <= r[0] < ends]
    assert during and all(v == w == 0 for _,_,_,v,w in during)
    assert any(v > 0 for t,_,_,v,_ in state.sent if t > ends)
    assert abs(state.x-(station[0]-.95)) <= .02
    assert abs(state.y-station[1]) <= .025
    assert abs(state.yaw) <= math.radians(2)
    assert state.linear == state.angular == 0
    assert any(f'{sensor}_fresh=False' in str(c)
               for c in motion.node.get_logger().warning.call_args_list)


@pytest.mark.parametrize('sensor', ['odom','imu','wheel'])
def test_front_feedback_loss_cannot_reset_the_total_wait_budget(simulation,monkeypatch,sensor):
    motion,state = simulation
    approach,_ = prepare(simulation,monkeypatch,lateral_offset=0.)
    method = {'odom':'odom_is_fresh','imu':'imu_is_fresh','wheel':'wheel_yaw_is_fresh'}[sensor]
    fresh = getattr(motion,method)
    setattr(motion,method,lambda:not (10.8 <= state.now < 16. or state.now >= 16.9) and fresh())
    assert not approach.run()
    error = motion.node.get_logger().error.call_args.args[0]
    assert 'stopped observation wait exhausted' in error
    assert state.now < 24
    assert state.linear == state.angular == 0
    assert all(v == w == 0 for t,_,_,v,w in state.sent if t >= 17.0)
