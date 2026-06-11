from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Navigation controller responsible for generating cmd_vel
            # commands based on lane following and detected traffic signs.
            Node(
                package="autonomous_navigation",
                executable="controller",
                output="screen",
            ),
            # Image preprocessing node.
            # Resizes and rotates incoming camera frames before they are
            # consumed by the perception modules.
            Node(
                package="autonomous_navigation",
                executable="image_resizer",
                output="screen",
            ),
            # Lane detection node based on classical computer vision.
            # Processes the preprocessed image and publishes a steering
            # reference used by the navigation controller.
            Node(
                package="autonomous_navigation",
                executable="lineFollower",
                output="screen",
            ),
        ]
    )
