import os
from glob import glob
from setuptools import setup

package_name = 'jupiter_ctrl'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    # data_files는 런치 파일, package.xml 등 비-소스 코드를 설치합니다.
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 디렉토리 안의 모든.launch.py 파일을 설치합니다.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user', # TODO: 유지보수자 이름으로 변경
    maintainer_email='user@todo.todo', # TODO: 유지보수자 이메일로 변경
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # entry_points는 파이썬 스크립트를 실행 가능한 노드로 등록합니다.
    entry_points={
        'console_scripts': [
            'jupiter_keyboard = jupiter_ctrl.jupiter_keyboard:main',
            'jupiter_joy = jupiter_ctrl.jupiter_joy:main',
            'turtlebot_joy = jupiter_ctrl.turtlebot_joy:main',
            'twist_joy = jupiter_ctrl.twist_joy:main',
        ],
    },
)