import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    # Package directories
    pkg_four_wheel_bot = FindPackageShare('four_wheel_bot_pkg')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    
    # Robot model file - Using Ackermann bot
    urdf_file = PathJoinSubstitution([
        pkg_four_wheel_bot,
        'urdf',
        'ackermann_bot.urdf.xacro'
    ])
    
    # Process the URDF file
    robot_description_content = Command(['xacro ', urdf_file])
    
    # World file
    world_file = PathJoinSubstitution([
        pkg_four_wheel_bot,
        'worlds',
        'simple_world.sdf'
    ])

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': ['-r ', world_file],
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', 
            '-name', 'ackermann_bot',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.2'
        ],
        output='screen'
    )

    # Bridge for cmd_vel, camera, and lidar
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/world/simple_world/model/ackermann_bot/link/base_link/sensor/camera_sensor/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/world/simple_world/model/ackermann_bot/link/base_link/sensor/lidar_sensor/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/world/simple_world/model/ackermann_bot/link/base_link/sensor/lidar_sensor/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    # Teleop keyboard node (delayed to allow robot to spawn)
    teleop_keyboard = TimerAction(
        period=3.0,  # Wait 3 seconds
        actions=[
            Node(
                package='four_wheel_bot_pkg',
                executable='teleop_twist_keyboard.py',
                name='teleop_twist_keyboard',
                output='screen',
                prefix='xterm -e',  # Run in separate terminal
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }]
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher,
        gazebo_launch,
        spawn_robot,
        bridge,
        teleop_keyboard
    ])
