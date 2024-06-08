from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_interface',
            executable='arm_interface_node',
            name='arm_interface',
            namespace='arm',
            parameters=["config/arm_config.yaml"],
            output="screen",
            emulate_tty=True
        )
    ])
