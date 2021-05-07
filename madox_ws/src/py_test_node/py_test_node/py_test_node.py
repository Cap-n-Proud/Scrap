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

import rclpy
from rclpy.node import Node

# import board

from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray, Int16
from sensor_msgs.msg import Joy

import time
import argparse


class Node_Name(Node):
    def __init__(self, arg1=False, arg2=False, arg3=0.4 * args, **kwargs):
        super().__init__("robot")

        self.arg1 = arg1
        self.arg2 = arg2
        self.arg3 = arg3

        self.joy_topic = self.create_subscription(Joy, "joy", self.joy_topic, 10)

    def joy_topic(self, msg):
        print("topic")


def main(args=None):
    rclpy.init(args=args)

    node_name = Node_Name()
    print("Robot ready")

    rclpy.spin(node_name)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node_name.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
