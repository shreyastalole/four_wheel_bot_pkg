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
    
    # Robot model file
    urdf_file = PathJoinSubstitution([
        pkg_four_wheel_bot,
        'urdf',
        'diff_drive_bot.urdf.xacro'
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

    # Gazebo simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_ros_gz_sim,
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': [world_file]
        }.items()
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'circle_bot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Bridge between ROS and Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    # Circle mover - starts after a delay to let everything initialize
    circle_mover = TimerAction(
        period=5.0,  # Wait 5 seconds for everything to start
        actions=[
            Node(
                package='four_wheel_bot_pkg',
                executable='circle_mover.py',
                name='circle_mover',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        circle_mover
    ])
