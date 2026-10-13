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
    assert node.defaults['backup_heading_kp'] == 0.35
    assert node.defaults['backup_heading_kd'] == 0.0
    assert node.defaults['backup_reverse_angular_command_sign'] == 1.0
    assert node.defaults['backup_heading_tolerance'] == math.radians(1.0)
    assert node.defaults['backup_heading_max_angular_speed'] == 0.015
    assert node.defaults['backup_heading_max_angular_accel'] == 0.030
    assert node.defaults['backup_heading_pause_error'] == math.radians(3.0)
    assert node.defaults['backup_heading_resume_stable_cycles'] == 3
    assert node.defaults['use_lidar_heading_during_backup'] is True
    assert node.defaults['backup_lidar_heading_filter_coef'] == 0.15
    assert node.defaults['backup_lidar_heading_max_error'] == math.radians(12.0)
    assert node.defaults['backup_lidar_motion_residual'] == math.radians(2.0)
    assert node.defaults['backup_lidar_wheel_guard_residual'] == math.radians(4.0)
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
    assert node.defaults['backup_target_rear_clearance'] == 0.0145
    assert node.defaults['backup_clearance_tolerance'] == 0.010
    assert node.defaults['backup_lidar_success_min_points'] >= 5
    assert node.defaults['backup_lidar_success_min_angle_span'] >= math.radians(3.0)
    assert node.defaults['backup_lidar_stable_cycles'] >= 3
    assert node.defaults['backup_max_yaw_drift'] == math.radians(8.0)
    assert node.defaults['backup_blocked_timeout_sec'] == 1.0
    assert node.defaults['lidar_align_sector_center_base'] == math.pi
    assert math.isnan(node.defaults['lidar_align_sector_center'])
    assert node.defaults['lidar_align_sector_width'] == math.radians(60.0)
    assert node.defaults['lidar_align_min_line_length'] == 0.15
    assert node.defaults['lidar_align_tolerance'] == math.radians(1.0)
    assert node.defaults['lidar_align_stable_cycles'] == 5
    assert node.defaults['lidar_align_ransac_iterations'] == 100
    assert node.defaults['lidar_align_ransac_threshold'] == .010
    assert node.defaults['lidar_align_candidate_max_error'] == math.radians(15.0)
    assert node.defaults['lidar_align_timeout_sec'] == 18.0
    assert node.defaults['lidar_align_max_rotation'] == math.radians(18.0)
    assert node.defaults['lidar_align_rotation_margin'] == math.radians(3.0)
    assert node.defaults['lidar_align_hard_max_rotation'] == math.radians(30.0)
    assert node.defaults['lidar_align_acquisition_stable_cycles'] == 3
    assert node.defaults['lidar_align_acquisition_max_residual'] == (
        math.radians(3.0))
    assert node.defaults['backup_max_travel'] == 0.60
    assert node.defaults['tag_refinement_longitudinal_tolerance'] == .02
    assert node.defaults['tag_refinement_lateral_tolerance'] == .025
    assert node.defaults['tag_refinement_yaw_tolerance'] == math.radians(2)
    assert node.defaults['tag_centerline_speed'] == .08
    assert node.defaults['tag_centerline_angular_speed'] == .15
    assert node.defaults['tag_centerline_max_travel'] == 2.0
    assert node.defaults['tag_front_observation_recovery_timeout_sec'] == 8.0
    assert node.defaults['tag_front_pose_max_age_sec'] == .30
    assert node.defaults['lidar_align_max_tracking_residual'] == math.radians(5.0)
    assert node.defaults['lidar_align_tracking_outlier_cycles'] == 2
    assert node.defaults['lidar_align_hard_tracking_residual'] == (
        math.radians(12.0))
    assert node.defaults['lidar_guide_sector_width'] == math.radians(220.0)
    assert node.defaults['lidar_guide_min_abs_y'] == 0.14
    assert node.defaults['lidar_guide_max_abs_y'] == 0.40
    assert node.defaults['lidar_guide_min_separation'] == 0.44
    assert node.defaults['lidar_guide_max_separation'] == 0.60
    assert node.defaults['lidar_guide_min_inliers'] == 8
    assert node.defaults['lidar_guide_min_line_length'] == 0.15
    assert node.defaults['lidar_guide_max_x'] == -0.12
    assert node.defaults['development_test_mode'] is True
    assert node.defaults['total_timeout_sec'] == 180.0


def test_runtime_uses_one_tag_approach_and_verified_guide_handoff():
    config = yaml.safe_load((Path(__file__).parents[1]/'config/docking.yaml').read_text())
    p = config['dock_turn_backup']['ros__parameters']
    assert p['start_docking_server'] is False
    assert p['apriltag_front_standoff'] == .95
    assert p['tag_centerline_speed'] == .08
    assert p['tag_refinement_lateral_tolerance'] == .012
    assert p['tag_refinement_yaw_tolerance'] == pytest.approx(math.radians(1))
    assert p['apriltag_front_lateral_offset'] == -.03
    assert p['tag_front_use_lidar_panel_heading'] is True
    assert p['development_test_mode'] is True
    assert p['use_lidar_guide_centering'] is True
    assert p['backup_guide_visibility_handoff_clearance'] == .32
    assert p['backup_speed'] == .035
    assert p['backup_guide_center_max_heading'] == pytest.approx(math.radians(2.5))
    assert p['backup_target_rear_clearance'] == .028
    assert p['backup_clearance_tolerance'] == .006
