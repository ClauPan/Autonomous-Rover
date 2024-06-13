import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
	package_name = "vox"
	package_path = os.path.join(get_package_share_directory(package_name))

	robot_state_publisher = Node(
		package="robot_state_publisher",
		executable="robot_state_publisher",
		output="screen",
		parameters=[{"robot_description": Command(['xacro ', os.path.join(package_path, "description", "robot.urdf.xacro")])}]
	)

	manager = TimerAction(
		period=3.0, 
		actions=[Node(
			package="controller_manager",
			executable="ros2_control_node",
			parameters=[
				{"robot_description": Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])}, 
				os.path.join(package_path, "config", "skid_steer.yaml")
			]
		)]
	)

	interface = RegisterEventHandler(
		event_handler=OnProcessStart(
			target_action=manager,
			on_start=[Node(
				package="controller_manager",
				executable="spawner",
				arguments=["skid_steer"],
			)]
		)
	)

	joint = RegisterEventHandler(
		event_handler=OnProcessStart(
			target_action=manager,
			on_start=[Node(
				package="controller_manager",
				executable="spawner",
				arguments=["broadcaster"],
			)]
		)
	)

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

	imu = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([
			os.path.join(get_package_share_directory("mpu9250driver"), "launch", "mpu9250driver_launch.py")
		])
	)

	ekf = Node(
		package='robot_localization',
		executable='ekf_node',
		name='ekf_filter_node',
		output='screen',
		parameters=[os.path.join(package_path, 'config', 'ekf.yaml')]
	)

	return LaunchDescription([
		robot_state_publisher,
		manager,
		interface,
		joint,
		lidar,
		imu,
		ekf
	])
