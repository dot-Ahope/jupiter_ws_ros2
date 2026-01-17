#!/usr/bin/env python3
from setuptools import setup

package_name = 'rtcm_ublox_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ngii_rtk_full.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='A bridge to forward RTCM messages to UBLOX GPS receiver',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rtcm_ublox_bridge_node = rtcm_ublox_bridge.rtcm_ublox_bridge_node:main',
        ],
    },
)
