from setuptools import setup
from glob import glob
import os

package_name = 'jupiter_safety'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='Tier 2 safety layer: stuck detection + forced recovery (perception-independent)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stuck_detector = jupiter_safety.stuck_detector:main',
            'forced_recovery = jupiter_safety.forced_recovery:main',
            'alert_publisher = jupiter_safety.alert_publisher:main',
        ],
    },
)
