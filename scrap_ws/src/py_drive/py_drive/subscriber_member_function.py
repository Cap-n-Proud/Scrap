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

from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray, Int16


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__("minimal_subscriber")
        self.subscription = self.create_subscription(
            String, "topic", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

        self.stop_motors = self.create_subscription(
            String, "stop_motors", self.stop_motors, 10
        )
        self.example_topic = self.create_subscription(
            String, "example_topic", self.example_topic, 10
        )

        self.joy_topic = self.create_subscription(
            String, "joy_topic", self.joy_topic, 10
        )
        self.test = self.create_subscription(Int32MultiArray, "test", self.test, 10)

        self.move = self.create_subscription(String, "move", self.move, 10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def test(self, msg):
        self.get_logger().info('test: "%s"' % msg.data)

    def example_topic(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def joy_topic(self, msg):
        self.get_logger().info('joy: "%s"' % msg.data)

    def stop_motors(self, msg):
        self.get_logger().info("I heard: " + str(msg.data))

    def move(self, msg):
        m = msg.data.split(",")
        self.get_logger().info("Move: " + str(m[0]) + "," + str(m[1]))


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
