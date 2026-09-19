"""Check the real Nav2 server alongside a competing public action server.

Uses DDS domain 219 and synthetic TF only. Does not run the robot motion steps.
Run after sourcing the workspace: python3 test/check_docking_server_isolation.py
"""

import math
import os
from pathlib import Path
import tempfile
import threading
import time


def main():
    os.environ['ROS_DOMAIN_ID'] = '219'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

    import rclpy
    import yaml
    from docking.dock_turn_backup import DockTurnBackup
    from geometry_msgs.msg import TransformStamped
    from nav2_msgs.action import DockRobot
    from rcl_interfaces.srv import GetParameters
    from rclpy.action import ActionClient, ActionServer, GoalResponse
    from rclpy.executors import SingleThreadedExecutor
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

    root = Path(__file__).parents[1]
    config = yaml.safe_load((root / 'config' / 'docking.yaml').read_text())
    config['docking_server']['ros__parameters']['simple_charging_dock'][
        'use_external_detection_pose'] = False
    calls = []
    with tempfile.TemporaryDirectory(prefix='dock-isolation-test-') as directory:
        params = Path(directory) / 'docking.yaml'
        params.write_text(yaml.safe_dump(config))
        rclpy.init(args=[
            '--ros-args', '--params-file', str(params),
            '-p', 'start_apriltag:=false', '-p', 'start_bridge:=false',
            '-p', f'docking_params_file:={params}',
        ])
        # Simulate another machine's already running /dock_robot endpoint.
        other = rclpy.create_node('existing_nav2_server')

        def reject(goal):
            calls.append(goal)
            return GoalResponse.REJECT

        public_server = ActionServer(
            other, DockRobot, '/dock_robot',
            execute_callback=lambda handle: DockRobot.Result(), goal_callback=reject)
        broadcaster = StaticTransformBroadcaster(other)
        transform = TransformStamped()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)
        executor = SingleThreadedExecutor()
        executor.add_node(other)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        node = DockTurnBackup()
        try:
            heading_limit = node.get_parameter(
                'tag_refinement_translation_heading_limit').value
            assert math.isclose(
                heading_limit, math.radians(8.0), abs_tol=1e-9)
            for name, expected_degrees in (
                    ('lidar_align_max_rotation', 18.0),
                    ('lidar_align_rotation_margin', 3.0),
                    ('lidar_align_hard_max_rotation', 30.0)):
                assert math.isclose(
                    node.get_parameter(name).value,
                    math.radians(expected_degrees),
                    abs_tol=1e-9)
            assert node.stack.start()
            assert node.lifecycle.configure_and_activate(node.should_stop)
            assert node._wait_for_dock_server()
            assert node._wait_for_base_transform()
            node._warmup_docking_server_tf_buffer()
            assert node._dock_near_tag(), 'Private server goal/result exchange failed'
            assert not calls, 'A docking goal leaked to the public server'
            parameter_client = node.create_client(
                GetParameters, f'{node.stack.server_node_name}/get_parameters')
            assert parameter_client.wait_for_service(timeout_sec=3.0)
            future = parameter_client.call_async(GetParameters.Request(names=[
                'max_retries',
                'controller_frequency',
                'simple_charging_dock.external_detection_translation_x']))
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
            assert future.done()
            values = future.result().values
            assert values[0].integer_value == 1
            assert [value.double_value for value in values[1:]] == [10.0, -0.95]

            # Even identical node names must not hide duplicate action services.
            duplicate = rclpy.create_node('existing_nav2_server')
            duplicate_server = ActionServer(
                duplicate, DockRobot, '/dock_robot',
                execute_callback=lambda handle: DockRobot.Result(), goal_callback=reject)
            executor.add_node(duplicate)
            external_client = ActionClient(node, DockRobot, '/dock_robot')
            saved_action = node.stack.dock_action
            node.stack.dock_action = '/dock_robot'
            try:
                deadline = time.monotonic() + 5.0
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
                    if node._dock_server_counts() == (2, 2, 2):
                        break
                assert node._dock_server_counts() == (2, 2, 2)
                assert node._send_and_wait(external_client, DockRobot.Goal()) is None
                assert not calls, 'Duplicate-server guard sent a goal'
            finally:
                node.stack.dock_action = saved_action
                external_client.destroy()
                executor.remove_node(duplicate)
                duplicate_server.destroy()
                duplicate.destroy_node()
            print('PASS: real Nav2 private goal succeeded; public server untouched; '
                  'calibration preserved; duplicate public endpoints blocked.')
        finally:
            node.stack.cleanup()
            node.destroy_node()
            executor.shutdown()
            thread.join(timeout=3.0)
            public_server.destroy()
            other.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
