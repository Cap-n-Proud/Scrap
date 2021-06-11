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

# from std_msgs.msg import Int32MultiArray, Int16
from sensor_msgs.msg import Imu, FluidPressure, Temperature

from sensor_msgs.msg import Joy

import time


def euler_from_quaternion(x, y, z, w, rad=False, approx=1):
    """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    if not rad:
        roll_x = round(math.degrees(roll_x), approx)
        pitch_y = round(math.degrees(pitch_y), approx)
        yaw_z = round(math.degrees(yaw_z), approx)
    return roll_x, pitch_y, yaw_z  # in radians


class PID_Node(Node):
    def __init__(self):
        super().__init__("PID_node")

        self.imu_topic = self.create_subscription(Imu, "/imu", self.imu_topic, 10)
        self.PID_pub = self.create_publisher(String, "/PID", qos_profile=10)

    def PID_topic(self, msg):
        euler = euler_from_quaternion(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
            False,
            1,
        )
        print(euler)


def main(args=None):
    rclpy.init(args=args)

    PID_node = PID_Node()
    print("Node ready")

    rclpy.spin(PID_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    PID_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
