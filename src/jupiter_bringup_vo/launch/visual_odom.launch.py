from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # RTAB-Map Odometry Node
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'base_footprint',
                'odom_frame_id': 'odom',
                'publish_tf': False,  # EKF will publish TF
                'wait_for_transform': 0.2,
                'approx_sync': True,
                'approx_sync_max_interval': 0.05,
                'queue_size': 10,
                'subscribe_rgbd': False,
                'guess_frame_id': 'odom',
                'Odom/Strategy': '0', # 0=Frame-to-Map (F2M) 1=Frame-to-Frame (F2F)
                'Odom/ResetCountdown': '1',
                'Odom/GuessMotion': 'true',
                'Vis/FeatureType': '6', # 6=GFTT/ORB
                'Vis/CorType': '0', # 0=Features Matching
                'Vis/EstimationType': '1', # 1=3D->2D (PnP)
                'Vis/MinInliers': '10',
                'Reg/Force3DoF': 'true', # 2D Robot
            }],
            remappings=[
                ('rgb/image', '/camera/rgb/image_raw'),
                ('rgb/camera_info', '/camera/rgb/camera_info'),
                ('depth/image', '/camera/depth/image_raw'),
                ('odom', '/odometry/visual')
            ]
        )
    ])
