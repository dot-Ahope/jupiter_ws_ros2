from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_prefix = FindPackageShare('jupiter_bringup')
    
    # u-blox F9P RTK GPS node
    ublox_params_path = PathJoinSubstitution([pkg_prefix, 'param', 'ublox', 'f9p_rover_params.yaml'])
    
    ublox_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps',
        output='screen',
        parameters=[ublox_params_path],
        respawn=True,
        respawn_delay=1.0
    )
    
    # Static transform for GPS sensor
    gps_tf_params_path = PathJoinSubstitution([pkg_prefix, 'param', 'ublox', 'gps_tf.yaml'])
    
    gps_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_gps_link',
        parameters=[gps_tf_params_path]
    )
    
    return LaunchDescription([
        ublox_node,
        gps_tf_node
    ])
