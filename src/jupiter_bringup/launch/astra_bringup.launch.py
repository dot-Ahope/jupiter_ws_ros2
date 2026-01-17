from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Launch Arguments
    use_gui = LaunchConfiguration('use_gui')
    robot_model = LaunchConfiguration('robot_model')
    
    # Declare launch arguments
    declare_use_gui = DeclareLaunchArgument(
        'use_gui',
        default_value='false',
        description='Whether to use joint_state_publisher_gui'
    )
    
    declare_robot_model = DeclareLaunchArgument(
        'robot_model',
        default_value='astra',
        description='Robot model type [astra, camera]'
    )
    
    # Robot State Publisher
    # URDF 파일 경로 구성
    urdf_file = PathJoinSubstitution([
        FindPackageShare('jupiter_description'),
        'urdf',
        f'jupiter_{LaunchConfiguration("robot_model")}.urdf'
    ])
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['cat ', PathJoinSubstitution([
                FindPackageShare('jupiter_description'),
                'urdf',
                'jupiter_astra.urdf'
            ])])
        }]
    )
    
    # Joint State Publisher (GUI)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        condition=IfCondition(use_gui)
    )
    
    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(use_gui)
    )
    
    # Include jupiter_joy launch
    jupiter_joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('jupiter_ctrl'),
                'launch',
                'jupiter_joy.launch.py'
            ])
        ])
    )
    
    # Include astra camera launch
    astra_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('astra_camera'),
                'launch',
                'astrapro.launch.py'
            ])
        ])
    )
    
    # Static TF Publishers
    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera',
        arguments=['0.0484', '0', '0.10403', '0', '0', '0', 'base_link', 'camera_link']
    )
    
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=['0.0484', '0', '0.10403', '0', '0', '0', 'base_link', 'laser']
    )
    
    return LaunchDescription([
        # Launch Arguments
        declare_use_gui,
        declare_robot_model,
        
        # Nodes
        robot_state_publisher,
        joint_state_publisher_gui,
        joint_state_publisher,
        
        # Included Launch Files
        jupiter_joy_launch,
        astra_launch,
        
        # Static TF Publishers
        base_to_camera_tf,
        base_to_laser_tf
    ])
