from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='libjaguar_drivetrain',
            executable='jaguar_twist_drivetrain_node',
            name='drivetrain',
            parameters=["config/drivetrain_config.yaml"],
            output="screen",
            emulate_tty=True
        )
    ])
