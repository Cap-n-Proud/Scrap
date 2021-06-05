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


# https://www.programcreek.com/python/?code=tianbot%2Ftianbot_racecar%2Ftianbot_racecar-master%2Fracecar_core%2Fscript%2Ftianbot_racecar_node.py
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from std_msgs.msg import Header

import argparse
import sys, getopt

sys.path.append(".")
import RTIMU
import os.path
import time
import math


# IMU ROS https://github.com/apiyap/rtimulib2_ros/blob/master/src/rtimulib_ros.cpp
# https://www.programcreek.com/python/?code=tomas789%2Fkitti2bag%2Fkitti2bag-master%2Fkitti2bag%2Fkitti2bag.py


class ImuSensorNode(Node):
    def __init__(self, debug=False, frequency=1 / 5):
        super().__init__("imu_sensor")
        self.debug = debug
        self.frequency = frequency
        self.orientation_covariance = [0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025]
        self.angular_velocity_covariance = [0.02, 0, 0, 0, 0.02, 0, 0, 0, 0.02]
        self.linear_acceleration_covariance = [0.04, 0, 0, 0, 0.04, 0, 0, 0, 0.04]

        self.init_IMU()
        self.imu_publisher = self.create_publisher(Imu, "imu_topic", 10)
        self.imu_timer = self.create_timer(self.frequency, self.publish_IMU)

    def init_IMU(self):
        SETTINGS_FILE = "RTIMULib"
        if not os.path.exists(SETTINGS_FILE + ".ini"):
            print("Settings file does not exist, will be created")

        s = RTIMU.Settings(SETTINGS_FILE)
        self.imu = RTIMU.RTIMU(s)
        pressure = RTIMU.RTPressure(s)

        print("IMU Name: " + self.imu.IMUName())
        print("Pressure Name: " + pressure.pressureName())

        if not self.imu.IMUInit():
            print("IMU Init Failed")
            sys.exit(1)
        else:
            print("IMU Init Succeeded")

        self.imu.setSlerpPower(0.02)
        self.imu.setGyroEnable(True)
        self.imu.setAccelEnable(True)
        self.imu.setCompassEnable(True)

        if not pressure.pressureInit():
            print("Pressure sensor Init Failed")
        else:
            print("Pressure sensor Init Succeeded")

        poll_interval = self.imu.IMUGetPollInterval()
        print("Recommended Poll Interval: %dmS\n" % poll_interval)

    def publish_IMU(self):
        # frame_id https://answers.ros.org/question/9957/what-frame_id-to-put-in-a-sensor_msgsimu-message/
        imu_msg = Imu()
        imu_data = self.imu.getIMUData()
        # imu_msg.header.stamp = 0
        print("publish " + str(self.debug))
        # https://wiki.ros.org/openrover-ros2
        if self.debug:
            print(imu_data)
        header = Header()
        header.stamp.sec = 1
        header.stamp.nanosec = 1

        header.frame_id = "base_link"
        imu_msg.header = header

        imu_msg.header.frame_id = "base_link"
        # imu_msg.orientation_covariance = self.orientation_covariance
        # imu_msg.angular_velocity_covariance = self.angular_velocity_covariance
        # imu_msg.linear_acceleration_covariance = self.linear_acceleration_covariance

        if imu_data["fusionQPoseValid"]:
            imu_msg.orientation.x = imu_data["fusionQPose"][0]
            imu_msg.orientation.y = imu_data["fusionQPose"][1]
            imu_msg.orientation.z = imu_data["fusionQPose"][2]
            imu_msg.orientation.w = imu_data["fusionQPose"][3]
        if imu_data["gyroValid"]:
            imu_msg.angular_velocity.x = imu_data["gyro"][0]
            imu_msg.angular_velocity.y = imu_data["gyro"][1]
            imu_msg.angular_velocity.z = imu_data["gyro"][2]
        if imu_data["accelValid"]:
            imu_msg.linear_acceleration.x = imu_data["accel"][0]
            imu_msg.linear_acceleration.y = imu_data["accel"][1]
            imu_msg.linear_acceleration.z = imu_data["accel"][2]
        self.imu_publisher.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)

    # Construct an argument parser
    parser = argparse.ArgumentParser()

    # Add arguments to the parser
    parser.add_argument(
        "-d",
        "--debug",
        required=False,
        action="store_true",
        default=False,
        help="Debug mode: robot will print extra log info in the console",
    )
    parser.add_argument(
        "-f",
        "--frequency",
        required=False,
        type=int,
        default=False,
        help="How often the messages will be published (seconds)",
    )

    args = parser.parse_args()
    if args.debug:
        print("Debug mode")

    node = ImuSensorNode(debug=args.debug, frequency=args.frequency)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
