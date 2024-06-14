import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
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
        parameters=[os.path.join(get_package_share_directory(package_name), "config", "tracker.yaml")]
    )

    return LaunchDescription([
        detect_node
    ])
