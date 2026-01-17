from setuptools import setup
import os
from glob import glob

package_name = 'jupiter_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
        (os.path.join('share', package_name, 'param'), glob(os.path.join('param', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=[
        'setuptools',
        'arm_jupiter',
        'opencv-python',
        'numpy',
        'transforms3d',
    ],
    zip_safe=True,
    maintainer='yahboom',
    maintainer_email='yahboom@todo.todo',
    description='The jupiter_bringup package for ROS2',
    license='TODO',
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        'console_scripts': [
            'base = jupiter_bringup.base:main',
            'device_srv = jupiter_bringup.device_srv:main',
            'calibrate_angular = jupiter_bringup.calibrate_angular:main',
            'calibrate_linear = jupiter_bringup.calibrate_linear:main',
            'jupiter_driver = jupiter_bringup.jupiter_driver:main',
            'jupiter_patrol = jupiter_bringup.jupiter_patrol:main',
            'rtcm_ublox_bridge = jupiter_bringup.rtcm_ublox_bridge:main',
        ],
    },
)