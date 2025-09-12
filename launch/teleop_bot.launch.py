import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    # Package directories
    pkg_four_wheel_bot = FindPackageShare('four_wheel_bot_pkg')
    
    # Launch arguments
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='diff_drive',
        choices=['diff_drive', 'ackermann'],
        description='Type of robot to launch (diff_drive or ackermann)'
    )

    # Include the appropriate robot launch file
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_four_wheel_bot,
                'launch',
                [LaunchConfiguration('robot_type'), '_bot.launch.py']
            ])
        ])
    )

    return LaunchDescription([
        robot_type_arg,
        robot_launch
    ])
