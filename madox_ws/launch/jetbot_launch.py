from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="camera",
                namespace="jetbot",
                executable="ros2_camera",
                name="camera",
            ),
            Node(
                package="imu_node",
                namespace="jetbot",
                executable="ros2_rtimulib",
                name="imu",
            ),
        ]
    )
