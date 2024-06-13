import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    package_name='vox'
    
    detect_node = Node(
        package='tracker',
        executable='tracker',
        parameters=[params_file],
    )

    return LaunchDescription([
        tracker_launch,
    ])
