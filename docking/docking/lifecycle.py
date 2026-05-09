from typing import Any

from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
import rclpy
from rclpy.node import Node


class DockingLifecycleManager:
    @staticmethod
    def declare_parameters(node: Node) -> None:
        node.declare_parameter('activate_docking_server', True)
        node.declare_parameter('lifecycle_timeout_sec', 20.0)

    def __init__(self, node: Node) -> None:
        self.node = node

    def configure_and_activate(self) -> bool:
        timeout = float(self.node.get_parameter('lifecycle_timeout_sec').value)
        get_state_client = self.node.create_client(GetState, '/docking_server/get_state')
        change_state_client = self.node.create_client(
            ChangeState, '/docking_server/change_state')

        self.node.get_logger().info('Waiting for docking_server lifecycle services...')
        if not get_state_client.wait_for_service(timeout_sec=timeout):
            self.node.get_logger().error(
                'docking_server get_state service is not available')
            return False
        if not change_state_client.wait_for_service(timeout_sec=timeout):
            self.node.get_logger().error(
                'docking_server change_state service is not available')
            return False

        state = self._get_state(get_state_client, timeout)
        if state is None:
            return False
        if state == State.PRIMARY_STATE_ACTIVE:
            self.node.get_logger().info('docking_server is already active')
            return True

        if state == State.PRIMARY_STATE_UNCONFIGURED:
            if not self._change_state(
                    change_state_client, Transition.TRANSITION_CONFIGURE, timeout):
                return False
            state = self._get_state(get_state_client, timeout)
            if state is None:
                return False

        if state == State.PRIMARY_STATE_INACTIVE:
            return self._change_state(
                change_state_client, Transition.TRANSITION_ACTIVATE, timeout)

        self.node.get_logger().error(f'docking_server is in unsupported state id={state}')
        return False

    def _get_state(self, client: Any, timeout: float) -> int | None:
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            self.node.get_logger().error('Failed to get docking_server lifecycle state')
            return None
        state = future.result().current_state
        self.node.get_logger().info(f'docking_server lifecycle state: {state.label}')
        return state.id

    def _change_state(self, client: Any, transition_id: int, timeout: float) -> bool:
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            self.node.get_logger().error(
                f'Lifecycle transition {transition_id} did not return')
            return False
        if not future.result().success:
            self.node.get_logger().error(f'Lifecycle transition {transition_id} failed')
            return False
        return True
