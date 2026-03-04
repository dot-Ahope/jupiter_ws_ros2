from setuptools import setup
import os
from glob import glob

package_name = 'jupiter_nav_VO'

setup(
    name=package_name,
    version='0.0.1',
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
    maintainer_email='user@todo.todo',
    description='Visual Odometry and GPS Navigation package for Jupiter',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_covariance_adapter = jupiter_nav_VO.odom_covariance_adapter:main',
            'vslam_covariance_adapter = jupiter_nav_VO.vslam_covariance_adapter:main',
            'goal_pose_restamper = jupiter_nav_VO.goal_pose_restamper:main',
            #odom_covariance_adapter: 사용자가 터미널에 입력할 커맨드(명령어) 이름입니다.
            # jupiter_nav_VO.odom_covariance_adapter: 실행될 파이썬 모듈(파일) 경로입니다.
            # main: 해당 파일 안에 정의된 main이라는 이름의 함수를 실행하라는 뜻입니다.
            # 터미널에서 단순히 ros2 run jupiter_nav_VO odom_covariance_adapter라고 입력하면 시스템이 자동으로 이 진입점을 찾아 main() 함수를 실행
        ],
    },
)
