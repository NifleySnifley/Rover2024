from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    ld = LaunchDescription()

    drivetrain_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("libjaguar_drivetrain"), "launch/drivetrain_launch.py"])
        )
    )

    navx_launch = Node(
        package="navx",
        executable="navx_node",
        name="navx_node",
        output="screen",
        emulate_tty=True,
    )
    
    camera_launch = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam_node",
        output="screen",
        emulate_tty=True,
    )

    ld.add_action(drivetrain_launch)
    ld.add_action(camera_launch)
    ld.add_action(navx_launch)

    # Return the launch description
    return ld
