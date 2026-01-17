from setuptools import setup
import os
from glob import glob

package_name = 'jupiter_laser'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.py'))),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/config', glob(os.path.join('config', '*.*'))),
        ('share/' + package_name + '/rviz', glob(os.path.join('rviz', '*.*')))
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'sensor_msgs',
        'geometry_msgs',
        'tf2_ros',
        'jupiter_msgs',
        'laser_geometry',
    ],
    zip_safe=True,
    maintainer='ycz',
    maintainer_email='ycz@todo.todo',
    description='The jupiter_laser package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laser_avoidance = jupiter_laser.laser_avoidance:main',
            'laser_tracker = jupiter_laser.laser_tracker:main',
            'laser_warning = jupiter_laser.laser_warning:main',
            'ydlidar_node = jupiter_laser.ydlidar_node:main'
        ]
    },
    # 필요한 의존성 패키지 추가
    package_data={
        package_name: [
            'resource/*',
            'launch/*',
            'config/*',
            'rviz/*'
        ]
    }
)
