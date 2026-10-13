"""Real process signals must not destroy ROS before motor-stop cleanup."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest


@pytest.mark.parametrize('backup_success,live_centered,expected_code', [
    (True, True, 0), (False, True, 4), (True, False, 4)])
def test_real_main_preserves_backup_result_in_test_mode(
        tmp_path, backup_success, live_centered, expected_code):
    script = r'''
import sys
from types import SimpleNamespace
from unittest.mock import Mock
from docking import dock_turn_backup as module

class MotionlessNode(module.DockTurnBackup):
    def __init__(self):
        super().__init__()
        # Verify real constructor wiring before replacing physical actions.
        assert self.motion.backup_lidar_guide_estimator == self.lidar_aligner.estimate_guide_center
        assert self.motion.backup_lidar_heading_estimator == self.lidar_aligner.estimate_error
        assert self.tag_centering.lidar.expected_frame() == 'base_scan'
        assert self.tag_centering.lidar.subscription.topic_name == '/scan'
        assert not hasattr(self, 'dock_client')
        assert not hasattr(self, 'entry_recovery')
        self._approach_tag_centerline = lambda: sys.argv[2] == 'True'
        self.motion = SimpleNamespace(
            wait_for_odom=lambda: True, advance_before_spin=lambda: True,
            spin_180=lambda: True, backup=lambda: sys.argv[1] == 'True',
            stop_robot=lambda: None)
        if sys.argv[2] != 'True':
            self.motion.spin_180 = Mock(side_effect=AssertionError('uncentered robot must not turn'))
            self.motion.backup = Mock(side_effect=AssertionError('uncentered robot must not reverse'))
        self.stack.stop_robot = lambda: None
        self.stack.start = lambda: True
        self.lidar_aligner.align = Mock(side_effect=AssertionError('no separate rear rotation stage'))
        self.charging.wait_for_existing_charge = lambda: False
        self.charging.verify = Mock(side_effect=AssertionError('test mode must skip charging'))
        for name in ('_wait_for_base_transform', '_wait_for_docking_lidar', '_wait_for_detected_dock_pose'):
            setattr(self, name, lambda: True)

module.DockTurnBackup = MotionlessNode
sys.exit(module.main(args=['exit_code_test', '--ros-args', '-p', 'development_test_mode:=true']))
'''
    env = dict(os.environ, ROS_DOMAIN_ID='223', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
               DOCKING_LOCK_FILE=str(tmp_path / 'lock'))
    result = subprocess.run(
        [sys.executable, '-c', script, str(backup_success), str(live_centered)],
        env=env, capture_output=True, text=True, timeout=10)
    assert result.returncode == expected_code, result.stdout + result.stderr
    assert ('Development test completed' in result.stdout + result.stderr) is (
        backup_success and live_centered)


@pytest.mark.parametrize('failed_step', [0, 1, 2, 3])
def test_cleanup_attempts_all_steps_even_when_one_raises(failed_step):
    from types import SimpleNamespace
    from docking.dock_turn_backup import DockTurnBackup

    calls = []
    failure = RuntimeError('injected cleanup failure')

    def record(name):
        calls.append(name)
        if len(calls) - 1 == failed_step:
            raise failure

    node = SimpleNamespace(
        motion=SimpleNamespace(stop_robot=lambda: record('stop')),
        charging=SimpleNamespace(cancel_unconfirmed_charging=lambda: record('charger')),
        stack=SimpleNamespace(cleanup=lambda: record('stack')),
    )
    with pytest.raises(RuntimeError) as caught:
        DockTurnBackup.cleanup_managed_processes(node)
    assert caught.value is failure
    assert calls == ['stop', 'charger', 'stack', 'stop']


@pytest.mark.parametrize('timeout', [float('nan'), float('inf'), -1.0, 0.0])
def test_invalid_overall_timeout_is_rejected_before_startup(timeout):
    from types import SimpleNamespace
    from unittest.mock import Mock
    from docking.dock_turn_backup import DockTurnBackup
    from docking.safety import DockingExitCode

    node = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(value=timeout),
        get_logger=Mock(),
    )
    assert DockTurnBackup.run(node) == DockingExitCode.INVALID_REQUEST


@pytest.mark.parametrize('signum', [signal.SIGINT, signal.SIGTERM])
def test_main_keeps_ros_alive_for_cleanup_after_signal(tmp_path, signum):
    script = r'''
from pathlib import Path
import signal
import sys
import rclpy
from docking import dock_turn_backup as module
from docking.safety import DockingExitCode
ready, cleaned = map(Path, sys.argv[1:])
class MotionlessNode:
    def __init__(self):
        self.stop = None
        self.node = rclpy.create_node('shutdown_order_test')
    def request_stop(self, sig):
        self.stop = sig
    def run(self):
        ready.write_text('ready')
        while self.stop is None:
            rclpy.spin_once(self.node, timeout_sec=.05)
        return DockingExitCode.SIGINT if self.stop == signal.SIGINT else DockingExitCode.SIGTERM
    def cleanup_managed_processes(self):
        cleaned.write_text(str(rclpy.ok()))
    def destroy_node(self):
        self.node.destroy_node()
    def get_logger(self):
        return self.node.get_logger()
module.DockTurnBackup = MotionlessNode
sys.exit(module.main(args=['shutdown_order_test']))
'''
    ready, cleaned = tmp_path / 'ready', tmp_path / 'cleaned'
    env = dict(os.environ, ROS_DOMAIN_ID='221', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
               DOCKING_LOCK_FILE=str(tmp_path / 'lock'))
    process = subprocess.Popen(
        [sys.executable, '-c', script, str(ready), str(cleaned)],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        deadline = time.monotonic() + 10.0
        while not ready.exists() and process.poll() is None and time.monotonic() < deadline:
            time.sleep(.02)
        assert ready.exists(), 'Motionless test node did not initialize'
        process.send_signal(signum)
        output, _ = process.communicate(timeout=5.0)
        assert process.returncode == 128 + signum, output
        assert cleaned.read_text() == 'True', output
        assert 'ExternalShutdownException' not in output
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate()
