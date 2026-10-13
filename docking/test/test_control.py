import math
from types import SimpleNamespace
from unittest.mock import Mock

from docking.control import spin_for
from docking.motion import MotionController
import pytest
import rclpy
from rclpy.time import Time


def test_busy_executor_does_not_turn_20hz_motor_loop_into_callback_rate(monkeypatch):
    wall = [10.0]
    callbacks = [0]
    sent = []
    monkeypatch.setattr('time.monotonic', lambda: wall[0])
    monkeypatch.setattr(rclpy, 'ok', lambda: True)

    def spin_once(node, timeout_sec):
        # One immediately ready callback every millisecond; spin_once's
        # timeout is a maximum wait, not a rate limit.
        wall[0] += min(timeout_sec, 0.001)
        callbacks[0] += 1

    monkeypatch.setattr(rclpy, 'spin_once', spin_once)
    motion = MotionController.__new__(MotionController)
    motion.node = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(value=20.0),
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=wall[0])),
        get_logger=Mock(),
    )
    motion.should_stop = lambda: False
    motion.odom_is_fresh = lambda: True
    motion.cmd_vel_pub = SimpleNamespace(publish=lambda msg: sent.append(wall[0]))
    assert motion._run_until(
        lambda: wall[0] >= 10.5, lambda: MotionController.twist(angular_z=0.08), 1.0)
    assert 9 <= len(sent) <= 11
    assert min(b - a for a, b in zip(sent, sent[1:])) >= 0.05 - 1e-9
    assert callbacks[0] >= 400


def test_control_wait_can_be_interrupted_without_another_motion_command(monkeypatch):
    wall = [0.0]
    monkeypatch.setattr('time.monotonic', lambda: wall[0])
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    monkeypatch.setattr(rclpy, 'spin_once', lambda *a, **kw: wall.__setitem__(0, wall[0] + .01))
    spin_for(None, 1.0, lambda: wall[0] >= .03)
    assert wall[0] == pytest.approx(.03)

