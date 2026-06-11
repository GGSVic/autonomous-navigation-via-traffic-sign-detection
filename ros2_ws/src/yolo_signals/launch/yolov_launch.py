from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Launches the YOLO traffic sign detector node
            Node(
                package="yolo_signals",
                executable="signsDetector",
                output="screen",
            ),
            # Visualization tool used during development and debugging
            Node(
                package="rqt_image_view",
                executable="rqt_image_view",
                output="screen",
            ),
        ]
    )
