import os
import shutil
import signal
import subprocess
import tempfile
import time
from typing import Callable
import uuid

from rclpy.node import Node
import yaml


class ManagedStack:
    @staticmethod
    def declare_parameters(node: Node, default_docking_params: str) -> None:
        node.declare_parameter('manage_stack', True)
        node.declare_parameter('start_apriltag', True)
        node.declare_parameter('start_bridge', True)
        node.declare_parameter('start_docking_server', True)
        node.declare_parameter('docking_params_file', default_docking_params)
        node.declare_parameter('stack_startup_delay_sec', 2.0)

    def __init__(
            self,
            node: Node,
            stop_robot: Callable[[], None],
            should_stop: Callable[[], bool]) -> None:
        self.node = node
        self.stop_robot = stop_robot
        self.should_stop = should_stop
        self.processes: list[subprocess.Popen] = []
        self._launch_pids: set[int] = set()
        self._params_directory: tempfile.TemporaryDirectory | None = None
        self.owns_docking_server = (
            bool(node.get_parameter('manage_stack').value)
            and bool(node.get_parameter('start_docking_server').value))
        # A pre-existing Nav2 stack (including one on another computer) may
        # already own /dock_robot. Every managed run gets its own endpoints.
        self.namespace = (
            f'/stella_docking_{uuid.uuid4().hex[:12]}'
            if self.owns_docking_server else '')
        self.server_node_name = (
            f'{self.namespace}/docking_server' if self.owns_docking_server
            else str(node.get_parameter('docking_server_node').value))
        self.dock_action = self._endpoint('dock_action')
        self.refinement_pose_topic = self._endpoint('tag_refinement_target_pose_topic')
        self.detected_pose_topic = self._endpoint(
            'dock_pose_topic', private=bool(node.get_parameter('start_bridge').value))

    def _endpoint(self, parameter: str, private: bool = True) -> str:
        configured = str(self.node.get_parameter(parameter).value)
        if self.owns_docking_server and private:
            return f'{self.namespace}/{configured.lstrip("/")}'
        return self.node.resolve_topic_name(configured)

    def _server_command(self, ros2: str) -> list[str]:
        params_file = str(self.node.get_parameter('docking_params_file').value)
        with open(params_file, encoding='utf-8') as stream:
            configuration = yaml.safe_load(stream)
        if not isinstance(configuration, dict):
            raise ValueError('docking_params_file must contain a ROS parameter mapping')
        # YAML node selectors must follow the new namespace as well. Keep
        # wildcard sections and all values, especially the calibrated offsets.
        for selector in ('docking_server', '/docking_server'):
            if selector in configuration:
                configuration[self.server_node_name] = configuration.pop(selector)
        self._params_directory = tempfile.TemporaryDirectory(prefix='stella-docking-')
        private_params = os.path.join(self._params_directory.name, 'docking.yaml')
        with open(private_params, 'w', encoding='utf-8') as stream:
            yaml.safe_dump(configuration, stream, sort_keys=False)
        command = [
            ros2, 'run', 'opennav_docking', 'opennav_docking',
            '--ros-args', '--params-file', private_params,
            '-r', f'__ns:={self.namespace}',
        ]
        remappings = {
            'dock_robot': self.dock_action,
            'detected_dock_pose': self.detected_pose_topic,
            'dock_pose': self.refinement_pose_topic,
            'cmd_vel': self.node.resolve_topic_name(
                str(self.node.get_parameter('cmd_vel_topic').value)),
            'tf': '/tf', 'tf_static': '/tf_static',
            'navigate_to_pose': '/navigate_to_pose',
            'battery_state': '/battery_state', 'joint_states': '/joint_states',
            'local_costmap/costmap_raw': '/local_costmap/costmap_raw',
            'local_costmap/costmap_raw_updates': '/local_costmap/costmap_raw_updates',
            'local_costmap/published_footprint': '/local_costmap/published_footprint',
        }
        for source, target in remappings.items():
            command.extend(['-r', f'{source}:={target}'])
        return command

    def start(self) -> bool:
        ros2 = shutil.which('ros2')
        if ros2 is None:
            self.node.get_logger().error('Could not find ros2 executable in PATH')
            return False

        if bool(self.node.get_parameter('start_apriltag').value):
            self._start_process(
                'apriltag',
                [ros2, 'launch', 'docking', 'apriltag_36h11.launch.py'])
            if self.should_stop():
                return False

        if bool(self.node.get_parameter('start_bridge').value):
            self._start_process(
                'apriltag_bridge',
                [ros2, 'run', 'docking', 'apriltag_bridge',
                 '--ros-args', '--params-file',
                 str(self.node.get_parameter('docking_params_file').value),
                 '-p', f'pose_topic:={self.detected_pose_topic}'])
            if self.should_stop():
                return False

        if bool(self.node.get_parameter('start_docking_server').value):
            self.node.get_logger().info(
                f'Using isolated docking server {self.server_node_name}; '
                f'action={self.dock_action}, target={self.refinement_pose_topic}')
            self._start_process(
                'docking_server',
                self._server_command(ros2))
            if self.should_stop():
                return False

        delay = float(self.node.get_parameter('stack_startup_delay_sec').value)
        deadline = time.monotonic() + max(delay, 0.0)
        while not self.should_stop() and time.monotonic() < deadline:
            time.sleep(min(0.1, max(deadline - time.monotonic(), 0.0)))

        if self.should_stop():
            return False

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
                    if process.pid in self._launch_pids:
                        # launch forwards the signal to its children. Sending
                        # to its whole group also would signal them twice.
                        process.send_signal(signal.SIGINT)
                    else:
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
                    try:
                        os.killpg(process.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass

        self.stop_robot()

        for process in reversed(self.processes):
            if process.poll() is None:
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass

        for process in reversed(self.processes):
            if process.poll() is None:
                try:
                    process.wait(timeout=0.5)
                except subprocess.TimeoutExpired:
                    self.node.get_logger().error(
                        f'Could not reap managed process pid={process.pid}')

        self.stop_robot()

        if self._params_directory is not None:
            self._params_directory.cleanup()
            self._params_directory = None

    def _start_process(self, name: str, command: list[str]) -> None:
        self.node.get_logger().info(f'Starting {name}: {" ".join(command)}')
        env = os.environ.copy()
        env.setdefault('RCUTILS_LOGGING_USE_STDOUT', '1')
        process = subprocess.Popen(command, env=env, start_new_session=True)
        self.processes.append(process)
        if len(command) > 1 and command[1] == 'launch':
            self._launch_pids.add(process.pid)
