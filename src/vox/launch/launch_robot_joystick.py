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
	rsp = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([
			os.path.join(get_package_share_directory(package_name), "launch", "rsp.launch.py")
		]),
	)

	joystick_parameters = os.path.join(get_package_share_directory('vox'), 'config', 'joystick.yaml')

    joystick_node = Node(
		package='joy',
		executable='joy_node',
		parameters=[joy_params],
	)

    joystick_teleop = Node(
		package='teleop_twist_joy',
		executable='teleop_node',
		name='teleop_node',
		parameters=[joy_params],
		remappings=[('/cmd_vel','/cmd_vel_joy')]
	)

	mux_params = os.path.join(get_package_share_directory(package_name), 'config' ,'twist_mux.yaml')
	mux = Node(
		package="twist_mux",
		executable="twist_mux",
		parameters=[mux_params],
		remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
	)

	robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
	controller_params_file = os.path.join(get_package_share_directory(package_name), "config", "skid_steer_config.yaml")

	manager = TimerAction(
		period=3.0, 
		actions=[Node(
			package="controller_manager",
			executable="ros2_control_node",
			parameters=[{"robot_description": robot_description}, controller_params_file]
		)]	
	)

	interface = RegisterEventHandler(
		event_handler=OnProcessStart(
			target_action=controller_manager,
			on_start=[Node(
				package="controller_manager",
				executable="spawner",
				arguments=["diff_cont"],
			)]
		)
	)

	joint = RegisterEventHandler(
		event_handler=OnProcessStart(
			target_action=controller_manager,
			on_start=[Node(
				package="controller_manager",
				executable="spawner",
				arguments=["joint_broad"],
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
		parameters=[os.path.join(get_package_share_directory(package_name), 'config', 'ekf.yaml')]
	)

	return LaunchDescription([
		rsp,
		joystick_node,
		joystick_teleop,
		mux
		manager,
		interface,
		joint,
		lidar,
		imu,
		ekf
	])
