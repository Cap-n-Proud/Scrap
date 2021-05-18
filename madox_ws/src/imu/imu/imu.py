#!/usr/bin/env python3

# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# https://roboticsbackend.com/ros2-python-publisher-example/

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from sensor_msgs.msg import Temparature
from sensor_msgs.msg import MagneticField

import time
import argparse

# IMU ROS https://github.com/apiyap/rtimulib2_ros/blob/master/src/rtimulib_ros.cpp


class ImuSensorNode(Node):
    def __init__(self):
        super().__init__("imu_sensor")
        self.imu_publisher = self.create_publisher(Int64, "temperature", 10)
        self.imu_timer = self.create_timer(2.0, self.publish)

    def publish(self):
        # frame_id https://answers.ros.org/question/9957/what-frame_id-to-put-in-a-sensor_msgsimu-message/
        imu_msg = Imu()
        temp_msg = Temparature()
        imu_msg.header.stamp = time.now()
        imu_msg.header.frame_id = "base_link"
        imu_msg.orientation.x = imu_data.fusionQPose.x()
        imu_msg.orientation.y = imu_data.fusionQPose.y()
        imu_msg.orientation.z = imu_data.fusionQPose.z()
        imu_msg.orientation.w = imu_data.fusionQPose.scalar()
        imu_msg.angular_velocity.x = imu_data.gyro.x()
        imu_msg.angular_velocity.y = imu_data.gyro.y()
        imu_msg.angular_velocity.z = imu_data.gyro.z()
        imu_msg.linear_acceleration.x = imu_data.accel.x()
        imu_msg.linear_acceleration.y = imu_data.accel.y()
        imu_msg.linear_acceleration.z = imu_data.accel.z()
        self.imu_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSensorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()


class Node_Name(Node):
    def __init__(self, arg1=False, arg2=False, arg3=0.4, *args, **kwargs):
        super().__init__("node_name")

        self.arg1 = arg1
        self.arg2 = arg2
        self.arg3 = arg3

        self.joy_topic = self.create_subscription(Joy, "joy", self.joy_topic, 10)

    def joy_topic(self, msg):
        print(msg)


def main(args=None):
    rclpy.init(args=args)

    node_name = Node_Name()
    print("Node ready")

    rclpy.spin(node_name)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node_name.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
