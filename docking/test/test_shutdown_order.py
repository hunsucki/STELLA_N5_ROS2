"""Real process signals must not destroy ROS before motor-stop cleanup."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest


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
