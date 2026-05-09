from glob import glob
from setuptools import find_packages, setup

package_name = 'docking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*launch*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NTREX',
    maintainer_email='lab@ntrex.co.kr',
    description='AprilTag docking helper package.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_bridge = docking.apriltag_bridge:main',
            'dock_turn_backup = docking.dock_turn_backup:main',
        ],
    },
)
