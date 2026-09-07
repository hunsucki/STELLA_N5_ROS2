from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'gimbal_camera_capture'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['PyYAML', 'setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='lab@ntrex.co.kr',
    description=(
        'Capture images and control two SIYI gimbals through ROS 2 topics.'
    ),
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capture_node = gimbal_camera_capture.capture_node:main',
            'control_node = gimbal_camera_capture.control_node:main',
        ],
    },
)
