import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
URDF_DIRECTORY = REPOSITORY_ROOT / 'stella_description' / 'urdf'


@pytest.mark.parametrize(
    'urdf_name',
    ['stella.urdf', 'stella_realsense.urdf', 'stella_web_cam.urdf'],
)
def test_all_robot_profiles_use_relocated_docking_lidar(urdf_name):
    root = ET.parse(URDF_DIRECTORY / urdf_name).getroot()
    joint = root.find("./joint[@name='scan2_joint']")

    assert joint is not None
    assert joint.find('parent').attrib['link'] == 'base_link'
    assert joint.find('child').attrib['link'] == 'base_scan2'

    origin = joint.find('origin')
    xyz = [float(value) for value in origin.attrib['xyz'].split()]
    rpy = [float(value) for value in origin.attrib['rpy'].split()]

    assert xyz == pytest.approx([-0.166, 0.0, 0.223])
    assert rpy == pytest.approx([0.0, 0.0, 3.1415])
    assert abs(math.atan2(math.sin(rpy[2]), math.cos(rpy[2])) - math.pi) < 1e-3


def test_rear_reference_matches_collision_geometry():
    root = ET.parse(URDF_DIRECTORY / 'stella_realsense.urdf').getroot()
    collision_box = root.find("./link[@name='base_link']/collision/geometry/box")
    collision = root.find("./link[@name='base_link']/collision")
    size_x = float(collision_box.attrib['size'].split()[0])
    collision_origin_x = float(collision.find('origin').attrib['xyz'].split()[0])
    rear_reference_x = collision_origin_x - size_x / 2.0

    assert rear_reference_x == pytest.approx(-0.2295)
    assert -0.166 - rear_reference_x == pytest.approx(0.0635)
