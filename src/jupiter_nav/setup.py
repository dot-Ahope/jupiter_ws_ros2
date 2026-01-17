from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'jupiter_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch 파일들 (jupiter_full_system, nav2_navigation)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Config 파일들 (ekf_config, slam_params, nav2_params)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # RViz 설정 파일들
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # Maps 디렉토리 (비어있지만 구조 생성)
        (os.path.join('share', package_name, 'maps'), glob('maps/*') if os.path.exists('maps') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jupiter User',
    maintainer_email='user@jupiter.local',
    description='Jupiter Navigation and SLAM Integration Package - Nav2 + SLAM Toolbox + EKF',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'odom_to_path = scripts.odom_to_path:main',
        ],
    },
)
