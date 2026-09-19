"""Regression tests for competing Nav2 docking servers on the same ROS graph."""

from pathlib import Path
import signal
from types import SimpleNamespace
from unittest.mock import Mock

from action_msgs.msg import GoalStatus
from docking.dock_turn_backup import DockTurnBackup
from docking.lifecycle import DockingLifecycleManager
from docking.stack_manager import ManagedStack
import pytest
import yaml


class FakeNode:
    def __init__(self, overrides=None):
        self.parameters = {'cmd_vel_topic': '/cmd_vel'}
        ManagedStack.declare_parameters(
            self, str(Path(__file__).parents[1] / 'config' / 'docking.yaml'))
        DockingLifecycleManager.declare_parameters(self)
        DockTurnBackup._declare_docking_parameters(self)
        self.parameters.update(overrides or {})
        self.logger = Mock()

    def declare_parameter(self, name, default):
        self.parameters[name] = default

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def get_logger(self):
        return self.logger

    def resolve_topic_name(self, name):
        return '/' + name.lstrip('/')


def test_managed_runs_use_different_private_action_and_pose_endpoints():
    stacks = [ManagedStack(FakeNode(), Mock(), lambda: False) for _ in range(2)]
    assert stacks[0].namespace != stacks[1].namespace
    for stack in stacks:
        assert stack.dock_action != '/dock_robot'
        assert stack.server_node_name != '/docking_server'
        assert stack.detected_pose_topic.startswith(stack.namespace + '/')
        assert stack.refinement_pose_topic.startswith(stack.namespace + '/')


@pytest.mark.parametrize('override', [
    {'manage_stack': False}, {'start_docking_server': False},
])
def test_external_server_mode_preserves_configured_endpoints(override):
    node = FakeNode({
        **override, 'dock_action': '/other/dock_robot',
        'docking_server_node': '/other/docking_server',
        'tag_refinement_target_pose_topic': '/other/dock_pose',
    })
    stack = ManagedStack(node, Mock(), lambda: False)
    assert not stack.owns_docking_server
    assert stack.dock_action == '/other/dock_robot'
    assert stack.server_node_name == '/other/docking_server'
    assert stack.refinement_pose_topic == '/other/dock_pose'
    assert stack.detected_pose_topic == '/detected_dock_pose'


def test_managed_server_can_use_an_external_detector():
    stack = ManagedStack(FakeNode({'start_bridge': False}), Mock(), lambda: False)
    assert stack.detected_pose_topic == '/detected_dock_pose'
    assert stack.refinement_pose_topic.startswith(stack.namespace)


def test_namespaced_server_preserves_calibration_and_root_robot_topics():
    stack = ManagedStack(FakeNode(), Mock(), lambda: False)
    try:
        command = stack._server_command('/usr/bin/ros2')
        path = Path(command[command.index('--params-file') + 1])
        config = yaml.safe_load(path.read_text())
        parameters = config[stack.server_node_name]['ros__parameters']
        assert parameters['simple_charging_dock']['external_detection_translation_x'] == -0.95
        assert parameters['controller_frequency'] == 10.0
        for rule in ['cmd_vel:=/cmd_vel', 'tf:=/tf', 'tf_static:=/tf_static',
                     f'dock_robot:={stack.dock_action}',
                     f'dock_pose:={stack.refinement_pose_topic}',
                     f'detected_dock_pose:={stack.detected_pose_topic}']:
            assert rule in command
    finally:
        stack.cleanup()
    assert not path.exists()


@pytest.mark.parametrize('counts,unique', [
    ((1, 1, 1), True), ((2, 2, 2), False), ((1, 2, 1), False), ((0, 0, 0), False),
])
def test_goal_is_not_sent_to_ambiguous_action_services(counts, unique):
    node = DockTurnBackup.__new__(DockTurnBackup)
    node.stack = SimpleNamespace(dock_action='/dock_robot')
    node.count_services = lambda name: dict(zip(
        ('send_goal', 'get_result', 'cancel_goal'), counts))[name.rsplit('/', 1)[1]]
    node.get_logger = Mock(return_value=Mock())
    assert node._dock_server_is_unique() == unique
    if not unique:
        client = Mock()
        assert node._send_and_wait(client, Mock()) is None
        client.send_goal_async.assert_not_called()


def test_unknown_action_result_cancels_accepted_goal_and_explains_failure():
    node = DockTurnBackup.__new__(DockTurnBackup)
    node.stack = SimpleNamespace(dock_action='/dock_robot')
    node._dock_server_is_unique = lambda: True
    node._wait_for_future = lambda future: True
    logger = Mock()
    node.get_logger = lambda: logger
    client = Mock()
    goal_handle = client.send_goal_async.return_value.result.return_value
    goal_handle.accepted = True
    goal_handle.get_result_async.return_value.result.return_value = SimpleNamespace(
        status=GoalStatus.STATUS_UNKNOWN)
    goal_handle.cancel_goal_async.return_value.done.return_value = True

    assert node._send_and_wait(client, Mock()) is None
    goal_handle.cancel_goal_async.assert_called_once()
    assert 'STATUS_UNKNOWN' in logger.error.call_args.args[0]


def test_lifecycle_targets_selected_server_and_refuses_duplicate_services():
    node = FakeNode()
    node.create_client = Mock(return_value=SimpleNamespace(
        srv_name='/private/docking_server/get_state',
        wait_for_service=lambda timeout_sec: True))
    node.count_services = lambda service: 2
    manager = DockingLifecycleManager(node, '/private/docking_server')
    assert not manager.configure_and_activate()
    assert [call.args[1] for call in node.create_client.call_args_list] == [
        '/private/docking_server/get_state', '/private/docking_server/change_state']


def test_cleanup_signals_launch_once_but_signals_ros2_run_process_group(monkeypatch):
    stack = ManagedStack(FakeNode(), Mock(), lambda: False)

    def process(pid):
        state = SimpleNamespace(running=True)
        return SimpleNamespace(
            pid=pid, poll=lambda: None if state.running else 0,
            send_signal=Mock(),
            wait=lambda timeout: setattr(state, 'running', False))

    launch = process(1001)
    wrapper = process(1002)
    stack.processes = [launch, wrapper]
    stack._launch_pids = {launch.pid}
    killpg = Mock()
    monkeypatch.setattr('docking.stack_manager.os.killpg', killpg)
    stack.cleanup()
    launch.send_signal.assert_called_once_with(signal.SIGINT)
    wrapper.send_signal.assert_not_called()
    killpg.assert_called_once_with(wrapper.pid, signal.SIGINT)
