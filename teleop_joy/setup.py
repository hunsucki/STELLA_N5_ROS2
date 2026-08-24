from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'teleop_joy'


setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='STELLA N5 developer',
    maintainer_email='user@example.com',
    description='Xbox gamepad teleoperation for the STELLA N5 mobile base.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_joy_node = teleop_joy.teleop_joy_node:main',
        ],
    },
)
