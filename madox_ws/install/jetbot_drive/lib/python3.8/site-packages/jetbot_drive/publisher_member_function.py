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
import random

from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(String, "topic", 10)
        self.move = self.create_publisher(Int32MultiArray, "move", 10)
        self.stop_motors = self.create_publisher(String, "stop_motors", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        case = random.randint(0, 99)

        if case in range(0, 30):
            msg = String()
            msg.data = "Hello World: %d" % self.i
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.i += 1

        elif case in range(31, 70):
            a = [random.randint(-99, 99), random.randint(-99, 99)]
            msg = Int32MultiArray()
            msg.data = a
            # msg.label = "labelArray"
            self.move.publish(msg)
            self.get_logger().info("Publishing move: " + str(msg.data))

        elif case in range(71, 99):
            a = [random.randint(-99, 99), random.randint(-99, 99)]
            msg = String()
            msg.data = "stop motors"
            # msg.label = "labelArray"
            self.stop_motors.publish(msg)
            self.get_logger().info("Publishing move: " + str(msg.data))

        else:
            print("Incorrect option")


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
