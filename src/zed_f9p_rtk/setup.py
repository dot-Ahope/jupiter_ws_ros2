import os
from glob import glob
from setuptools import setup

package_name = 'zed_f9p_rtk'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='alops',
    maintainer_email='alops@todo.todo',
    description='Standalone RTK GNSS and NTRIP Client Node for Jetson Nano/Orin',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zed_f9p_rtk_node = zed_f9p_rtk.zed_f9p_rtk_node:main'
        ],
    },
)
