from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    imu_node = Node(package="imu_node", executable="ros2_rtimulib")

    camera_node = Node(package="camera", executable="ros2_jetbot_camera")

    ld.add_action(imu_node)
    ld.add_action(camera_node)

    return ld
