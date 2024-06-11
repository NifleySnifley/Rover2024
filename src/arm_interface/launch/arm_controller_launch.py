from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_interface',
            executable='arm_spacenav_control_node',
            name='arm_spacenav_control',
            namespace='arm',
            parameters=["config/arm_controller_config.yaml"],
            output="screen",
            emulate_tty=True
        ),
        Node(
            package='arm_interface',
            executable='whacker_control_node',
            name='whacker_control',
            parameters=["config/whacker_controller_config.yaml"],
            output="screen",
            emulate_tty=True
        ),
        IncludeLaunchDescription(
			PythonLaunchDescriptionSource(
				PathJoinSubstitution([FindPackageShare("arm_interface"), "launch/arm_only_launch.py"])
			)
		)
    ])
