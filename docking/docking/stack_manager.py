import os
import shutil
import signal
import subprocess
import time
from typing import Callable

from rclpy.node import Node


class ManagedStack:
    @staticmethod
    def declare_parameters(node: Node, default_docking_params: str) -> None:
        node.declare_parameter('manage_stack', True)
        node.declare_parameter('start_apriltag', True)
        node.declare_parameter('start_bridge', True)
        node.declare_parameter('start_docking_server', True)
        node.declare_parameter('docking_params_file', default_docking_params)
        node.declare_parameter('stack_startup_delay_sec', 2.0)

    def __init__(self, node: Node, stop_robot: Callable[[], None]) -> None:
        self.node = node
        self.stop_robot = stop_robot
        self.processes: list[subprocess.Popen] = []

    def start(self) -> bool:
        ros2 = shutil.which('ros2')
        if ros2 is None:
            self.node.get_logger().error('Could not find ros2 executable in PATH')
            return False

        if bool(self.node.get_parameter('start_apriltag').value):
            self._start_process(
                'apriltag',
                [ros2, 'launch', 'docking', 'apriltag_36h11.launch.py'])

        if bool(self.node.get_parameter('start_bridge').value):
            self._start_process(
                'apriltag_bridge',
                [ros2, 'run', 'docking', 'apriltag_bridge'])

        if bool(self.node.get_parameter('start_docking_server').value):
            params_file = str(self.node.get_parameter('docking_params_file').value)
            self._start_process(
                'docking_server',
                [
                    ros2, 'run', 'opennav_docking', 'opennav_docking',
                    '--ros-args', '--params-file', params_file,
                ])

        delay = float(self.node.get_parameter('stack_startup_delay_sec').value)
        if delay > 0.0:
            time.sleep(delay)

        for process in self.processes:
            if process.poll() is not None:
                self.node.get_logger().error(
                    f'Managed process exited early: pid={process.pid}, '
                    f'code={process.returncode}')
                return False

        return True

    def cleanup(self) -> None:
        self.stop_robot()

        for process in reversed(self.processes):
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass

        deadline = time.monotonic() + 5.0
        for process in reversed(self.processes):
            remaining = max(deadline - time.monotonic(), 0.0)
            if process.poll() is None:
                try:
                    process.wait(timeout=remaining)
                except subprocess.TimeoutExpired:
                    process.terminate()

        for process in reversed(self.processes):
            if process.poll() is None:
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    process.kill()

    def _start_process(self, name: str, command: list[str]) -> None:
        self.node.get_logger().info(f'Starting {name}: {" ".join(command)}')
        env = os.environ.copy()
        env.setdefault('RCUTILS_LOGGING_USE_STDOUT', '1')
        process = subprocess.Popen(command, env=env, start_new_session=True)
        self.processes.append(process)
