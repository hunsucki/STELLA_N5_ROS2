"""Finish the existing half-turn using time-aligned physical panel scans."""
import math
from types import SimpleNamespace as S

import numpy as np
import pytest
import rclpy
from rclpy.time import Time
from tf2_ros import TransformException

from docking.lidar_geometry import normalize_angle
from test_backup_entry_completion import simulation  # noqa: F401
from test_backup_initial_heading import sync_initial_pose


def prepare_panel_spin(simulation,monkeypatch,start=170.,direction=1,factor=1.,fault=None):
    motion,state = simulation
    state.yaw = math.radians(start)
    sync_initial_pose(motion,state)
    origin = state.yaw
    goal = origin+direction*math.pi
    motion.node.parameters.update(spin_yaw=direction*math.radians(176),
                                  spin_use_lidar_heading=True)
    motion.current_yaw = lambda:normalize_angle(origin+factor*(state.yaw-origin))
    history = [(state.now,state.yaw)]
    spin = rclpy.spin_once
    def tick(node,timeout_sec):
        spin(node,timeout_sec)
        history.append((state.now,state.yaw))
    monkeypatch.setattr(rclpy,'spin_once',tick)
    def yaw_at(t):return float(np.interp(t,[r[0] for r in history],[r[1] for r in history]))
    def snapshot():
        seq = 1 if fault == 'single_scan' else int((state.now-10)*10)
        stamp = Time(nanoseconds=int((state.now-.12)*1e9)).to_msg()
        return S(sequence=seq,scan=S(header=S(stamp=stamp)))
    motion.lidar.snapshot = snapshot
    def estimate(snapshot):
        if fault == 'no_plane':return None
        stamp = snapshot.scan.header.stamp
        at = stamp.sec+stamp.nanosec/1e9
        error = normalize_angle(goal-yaw_at(at))
        if fault == 'out_of_bound':error += math.radians(20)
        if fault == 'inconsistent':error += math.radians(3 if snapshot.sequence%2 else -3)
        return error,0.,40 if fault == 'weak' else 89,115,.1 if fault == 'short' else .5
    motion.backup_lidar_heading_estimator = estimate
    def transform(fixed,base,at):
        if fault == 'missing_tf':raise TransformException('no acquisition-time odom')
        yaw = origin+factor*(yaw_at(at.nanoseconds/1e9)-origin)
        return S(transform=S(rotation=S(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2))))
    motion.node.tf_buffer = S(lookup_transform=transform)
    if fault == 'imu_loss':motion.imu_is_fresh = lambda:False
    return motion,state,goal


@pytest.mark.parametrize('start', [-170.,170.])
@pytest.mark.parametrize('direction', [-1,1])
@pytest.mark.parametrize('factor', [1.,176/183.])
def test_panel_finishes_real_half_turn_despite_odom_scale_error(
        simulation,monkeypatch,start,direction,factor):
    motion,state,goal = prepare_panel_spin(simulation,monkeypatch,start,direction,factor)
    assert motion.spin_180(),motion.node.get_logger().error.call_args
    assert abs(state.yaw-goal) <= math.radians(1.5)
    assert any('Spin target confirmed by three rear panel scans' in str(c)
               for c in motion.node.get_logger().info.call_args_list)
    assert state.linear == state.angular == state.actual_angular == 0
    assert all(v == 0 and abs(w) <= .15 for _,_,_,v,w in state.sent)


@pytest.mark.parametrize('fault', ['no_plane','weak','short','missing_tf','out_of_bound',
                                  'inconsistent','single_scan','imu_loss'])
def test_unconfirmed_panel_keeps_original_bounded_odom_target(simulation,monkeypatch,fault):
    motion,state,_ = prepare_panel_spin(simulation,monkeypatch,fault=fault)
    assert motion.spin_180(),motion.node.get_logger().error.call_args
    assert abs(state.yaw-math.radians(170+176)) <= math.radians(1)
    assert not any('Spin target confirmed' in str(c)
                   for c in motion.node.get_logger().info.call_args_list)
    assert state.linear == state.angular == 0


def test_panel_target_uses_scan_time_transform_not_current_yaw(simulation):
    motion,state = simulation
    state.yaw = math.radians(176)
    motion.backup_lidar_heading_estimator = lambda _: (math.radians(10),0,89,115,.5)
    queries = []
    def transform(fixed,base,at):
        queries.append(at.nanoseconds)
        yaw = math.radians(170)
        return S(transform=S(rotation=S(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2))))
    motion.node.tf_buffer = S(lookup_transform=transform)
    stamp = Time(nanoseconds=9_900_000_000).to_msg()
    target = motion._spin_panel_target(S(scan=S(header=S(stamp=stamp))),math.radians(176))
    assert normalize_angle(target-math.pi) == pytest.approx(0.)
    assert queries == [9_900_000_000]


def test_spin_still_times_out_without_chassis_rotation(simulation,monkeypatch):
    motion,state,_ = prepare_panel_spin(simulation,monkeypatch)
    state.yaw_response = lambda _:0.
    motion.node.parameters['motion_timeout_sec'] = 2.
    assert not motion.spin_180()
    assert state.linear == state.angular == 0
