import math
from pathlib import Path

from docking.dock_turn_backup import DockTurnBackup
from docking.docking_lidar import DockingLidar
from docking.lidar_alignment import LidarPlaneAligner
from docking.motion import MotionController
import pytest
import yaml


class ParameterCapture:
    def __init__(self):
        self.defaults = {}

    def declare_parameter(self, name, default):
        self.defaults[name] = default


def test_declared_lidar_defaults_match_new_mount_contract():
    node = ParameterCapture()
    MotionController.declare_parameters(node)
    LidarPlaneAligner.declare_parameters(node)
    DockingLidar.declare_parameters(node)
    DockTurnBackup._declare_docking_parameters(node)
    DockTurnBackup._declare_safety_parameters(node)

    assert node.defaults['docking_lidar_topic'] == '/scan_2'
    assert node.defaults['docking_lidar_frame'] == 'base_scan2'
    assert node.defaults['docking_lidar_scan_max_age_sec'] == 0.30
    assert node.defaults['backup_lidar_sector_center_base'] == math.pi
    assert math.isnan(node.defaults['backup_lidar_sector_center'])
    assert node.defaults['backup_lidar_sector_width'] == math.radians(20.0)
    assert node.defaults['backup_lidar_safety_sector_width'] == math.radians(60.0)
    assert node.defaults['imu_topic'] == '/imu/data'
    assert node.defaults['imu_max_age_sec'] == 0.25
    assert node.defaults['imu_stationary_yaw_rate'] == math.radians(0.5)
    assert node.defaults['spin_min_angular_speed'] == 0.025
    assert node.defaults['spin_slowdown_angle'] == math.radians(40.0)
    assert node.defaults['spin_tolerance'] == math.radians(1.0)
    assert node.defaults['spin_stable_cycles'] == 5
    assert node.defaults['use_pre_spin_forward'] is False
    assert node.defaults['pre_spin_forward_duration_sec'] == 1.0
    assert node.defaults['pre_spin_forward_speed'] == 0.03
    assert node.defaults['pre_spin_forward_max_distance'] == 0.05
    assert node.defaults['wheel_yaw_diagnostics_topic'] == (
        '/wheel_odometry/yaw_diagnostics')
    assert node.defaults['wheel_yaw_max_age_sec'] == 0.50
    assert node.defaults['backup_heading_kp'] == 0.20
    assert node.defaults['backup_heading_kd'] == 0.0
    assert node.defaults['backup_reverse_angular_command_sign'] == 1.0
    assert node.defaults['backup_heading_tolerance'] == math.radians(1.0)
    assert node.defaults['backup_heading_max_angular_speed'] == 0.004
    assert node.defaults['backup_heading_max_angular_accel'] == 0.010
    assert node.defaults['backup_heading_pause_error'] == math.radians(3.0)
    assert node.defaults['backup_heading_resume_stable_cycles'] == 3
    assert node.defaults['use_lidar_heading_during_backup'] is True
    assert node.defaults['backup_lidar_heading_filter_coef'] == 0.15
    assert node.defaults['backup_lidar_heading_max_error'] == math.radians(5.0)
    assert node.defaults['backup_lidar_heading_min_inlier_ratio'] == 0.70
    assert node.defaults['backup_lidar_heading_min_line_length'] == 0.15
    assert node.defaults['backup_lidar_heading_max_jump'] == math.radians(2.5)
    assert node.defaults['backup_lidar_heading_stable_cycles'] == 3
    assert node.defaults['backup_lidar_heading_disable_clearance'] == 0.10
    assert node.defaults['backup_lidar_heading_rebase_error'] == math.radians(1.0)
    assert node.defaults['use_lidar_guide_centering'] is False
    assert node.defaults['backup_guide_center_kp'] == 0.35
    assert node.defaults['backup_guide_center_tolerance'] == 0.015
    assert node.defaults['backup_guide_center_max_heading'] == math.radians(2.0)
    assert node.defaults['backup_guide_center_filter_coef'] == 0.20
    assert node.defaults['backup_guide_center_max_jump'] == 0.05
    assert node.defaults['backup_guide_center_stable_cycles'] == 3
    assert node.defaults['backup_guide_center_disable_clearance'] == 0.12
    assert node.defaults['backup_lidar_min_range'] == 0.05
    assert node.defaults['backup_rear_reference_x'] == -0.2295
    assert node.defaults['backup_lidar_success_min_points'] >= 5
    assert node.defaults['backup_lidar_success_min_angle_span'] >= math.radians(3.0)
    assert node.defaults['backup_lidar_stable_cycles'] >= 3
    assert node.defaults['backup_blocked_timeout_sec'] == 1.0
    assert node.defaults['lidar_align_sector_center_base'] == math.pi
    assert math.isnan(node.defaults['lidar_align_sector_center'])
    assert node.defaults['lidar_align_sector_width'] == math.radians(60.0)
    assert node.defaults['lidar_align_min_line_length'] == 0.15
    assert node.defaults['lidar_align_tolerance'] == math.radians(1.0)
    assert node.defaults['lidar_align_stable_cycles'] == 5
    assert node.defaults['lidar_align_ransac_iterations'] == 100
    assert node.defaults['lidar_align_candidate_max_error'] == math.radians(15.0)
    assert node.defaults['lidar_align_timeout_sec'] == 12.0
    assert node.defaults['lidar_align_max_rotation'] == math.radians(18.0)
    assert node.defaults['lidar_align_acquisition_stable_cycles'] == 3
    assert node.defaults['lidar_align_acquisition_max_residual'] == (
        math.radians(3.0))
    assert node.defaults['backup_max_travel'] == 0.60
    assert node.defaults['verify_tag_front_stop_pose'] is False
    assert node.defaults['use_tag_pose_refinement'] is True
    assert node.defaults['tag_refinement_target_pose_topic'] == '/dock_pose'
    assert node.defaults['tag_refinement_target_wait_timeout_sec'] == 1.0
    assert node.defaults['tag_refinement_longitudinal_tolerance'] == 0.04
    assert node.defaults['tag_refinement_lateral_tolerance'] == 0.025
    assert node.defaults['tag_refinement_yaw_tolerance'] == math.radians(2.0)
    assert node.defaults['tag_refinement_timeout_sec'] == 18.0
    assert node.defaults['tag_refinement_max_initial_yaw'] == math.radians(25.0)
    assert node.defaults['tag_refinement_max_travel'] == 0.18
    assert node.defaults['tag_refinement_max_yaw_excursion'] == math.radians(30.0)
    assert node.defaults['tag_refinement_abort_on_failure'] is True
    assert node.defaults['tag_front_stop_distance'] == 0.80
    assert node.defaults['tag_front_longitudinal_tolerance'] == 0.04
    assert node.defaults['tag_front_lateral_tolerance'] == 0.025
    assert node.defaults['tag_front_pose_max_age_sec'] == 0.30
    assert node.defaults['tag_front_stable_cycles'] == 5
    assert node.defaults['tag_front_verify_timeout_sec'] == 3.0
    assert node.defaults['lidar_align_max_tracking_residual'] == math.radians(5.0)
    assert node.defaults['lidar_align_hard_tracking_residual'] == (
        math.radians(12.0))
    assert node.defaults['lidar_guide_sector_width'] == math.radians(150.0)
    assert node.defaults['lidar_guide_min_abs_y'] == 0.14
    assert node.defaults['lidar_guide_max_abs_y'] == 0.40
    assert node.defaults['lidar_guide_min_separation'] == 0.44
    assert node.defaults['lidar_guide_max_separation'] == 0.60
    assert node.defaults['lidar_guide_min_inliers'] == 8
    assert node.defaults['lidar_guide_min_line_length'] == 0.15
    assert node.defaults['lidar_guide_max_x'] == -0.12
    assert node.defaults['development_test_mode'] is True
    assert node.defaults['total_timeout_sec'] == 100.0


def test_tag_front_pose_error_uses_tag_center_in_base_link():
    longitudinal, lateral = DockTurnBackup._tag_front_pose_errors(
        tag_x=0.81,
        tag_y=-0.012,
        target_distance=0.80,
    )

    assert longitudinal == pytest.approx(0.01)
    assert lateral == pytest.approx(-0.012)


def test_tag_front_pose_error_rejects_nonfinite_measurement():
    longitudinal, lateral = DockTurnBackup._tag_front_pose_errors(
        tag_x=math.nan,
        tag_y=0.0,
        target_distance=0.80,
    )

    assert math.isinf(longitudinal)
    assert math.isinf(lateral)


def test_fixed_refinement_goal_is_resolved_in_robot_frame():
    longitudinal, lateral, yaw = DockTurnBackup._fixed_goal_errors_in_base(
        target_x=1.0,
        target_y=2.0,
        target_yaw=math.radians(100.0),
        current_x=1.0,
        current_y=1.0,
        current_yaw=math.radians(90.0),
    )

    assert longitudinal == pytest.approx(1.0)
    assert lateral == pytest.approx(0.0, abs=1e-9)
    assert yaw == pytest.approx(math.radians(10.0))


def test_fixed_refinement_position_tolerance_does_not_rotate_with_robot():
    before = DockTurnBackup._fixed_goal_errors_in_target(
        target_x=1.0,
        target_y=0.0,
        target_yaw=0.0,
        current_x=0.98,
        current_y=-0.01,
        current_yaw=math.radians(20.0),
    )
    after = DockTurnBackup._fixed_goal_errors_in_target(
        target_x=1.0,
        target_y=0.0,
        target_yaw=0.0,
        current_x=0.98,
        current_y=-0.01,
        current_yaw=0.0,
    )

    assert before[:2] == pytest.approx(after[:2])
    assert before[2] == pytest.approx(math.radians(-20.0))
    assert after[2] == pytest.approx(0.0)


def test_refinement_command_moves_forward_and_steers_toward_lateral_error():
    linear, angular = DockTurnBackup._tag_refinement_command(
        longitudinal=0.10,
        lateral=0.02,
        yaw_error=0.0,
        within_position=False,
        linear_kp=0.5,
        angular_k_alpha=1.0,
        angular_k_beta=-0.3,
        final_yaw_kp=1.0,
        max_linear_speed=0.025,
        max_angular_speed=0.08,
    )

    assert linear == pytest.approx(0.025)
    assert angular > 0.0
    assert angular <= 0.08


def test_refinement_command_reverses_without_flipping_lateral_correction():
    linear, angular = DockTurnBackup._tag_refinement_command(
        longitudinal=-0.10,
        lateral=0.02,
        yaw_error=0.0,
        within_position=False,
        linear_kp=0.5,
        angular_k_alpha=1.0,
        angular_k_beta=-0.3,
        final_yaw_kp=1.0,
        max_linear_speed=0.025,
        max_angular_speed=0.08,
    )

    assert linear == pytest.approx(-0.025)
    assert angular < 0.0


def test_refinement_rotates_in_place_only_after_position_is_good():
    linear, angular = DockTurnBackup._tag_refinement_command(
        longitudinal=0.01,
        lateral=0.01,
        yaw_error=math.radians(3.0),
        within_position=True,
        linear_kp=0.5,
        angular_k_alpha=1.0,
        angular_k_beta=-0.3,
        final_yaw_kp=1.0,
        max_linear_speed=0.025,
        max_angular_speed=0.08,
    )

    assert linear == 0.0
    assert angular == pytest.approx(math.radians(3.0))


def test_camera_approach_keeps_room_for_the_spin():
    config_path = Path(__file__).parents[1] / 'config' / 'docking.yaml'
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    parameters = config['docking_server']['ros__parameters']

    assert parameters['max_retries'] == 0
    assert parameters['simple_charging_dock']['filter_coef'] == 0.1
    assert parameters['simple_charging_dock']['docking_threshold'] == 0.15
    assert parameters['controller']['v_linear_max'] == 0.10
    assert parameters['controller']['v_linear_min'] == 0.05

    helper_parameters = config['dock_turn_backup']['ros__parameters']
    assert helper_parameters['use_tag_pose_refinement'] is True
    assert helper_parameters['use_pre_spin_forward'] is False
    assert helper_parameters['tag_refinement_abort_on_failure'] is True
