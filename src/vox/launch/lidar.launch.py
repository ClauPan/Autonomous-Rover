import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
	lidar = Node(
		package="rplidar_ros",
		executable="rplidar_composition",
		output="screen",
		parameters=[{
			"serial_port": "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0",
			"frame_id": "laser_frame",
			"angle_compensate": True,
			"scan_mode": "Standard"
		}]
	)

	return LaunchDescription([
		lidar,
	])
