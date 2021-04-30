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
from sensor_msgs.msg import Joy

import time
from adafruit_motorkit import MotorKit

# https://github.com/adafruit/Adafruit_CircuitPython_MotorKit.git


class Robot(Node):
    def __init__(self):
        super().__init__("robot")
        # self.kit = MotorKit()

        self.joy_topic = self.create_subscription(Joy, "joy", self.joy_topic, 10)

        self.stop_motors = self.create_subscription(
            String, "stop_motors", self.stop_motors, 10
        )

        self.move_web = self.create_subscription(String, "move_web", self.move_web, 10)

    def _left_speed(self, speed):
        """Set the speed of the left motor, taking into account its trim offset."""
        assert -1 <= speed <= 1, "Speed must be a value between -1 to 1 inclusive!"
        speed += self._left_trim
        speed = max(-1, min(1, speed))  # Constrain speed to 0-255 after trimming.
        # kit.motor1.throttle = speed

    def _right_speed(self, speed):
        """Set the speed of the right motor, taking into account its trim offset."""
        assert -1 <= speed <= 1, "Speed must be a value between -1 to 1 inclusive!"
        speed += self._right_trim
        speed = max(-1, min(1, speed))  # Constrain speed to 0-255 after trimming.
        # kit.motor2.throttle = speed

    def m(self, x, y):
        # speed = msg.data.split(",")
        # self.get_logger().info("Move: " + str(speed[0]) + "," + str(speed[1]))
        # # self._left_speed(float(speed[0]))
        # self._right_speed(float(speed[1]))
        # currentSpeedL = -map(dY - configuration.steerGain * dX, -100, 100, -configuration.maxSpeed, configuration.maxSpeed);
        # currentSpeedR = -map(dY + configuration.steerGain * dX, -100, 100,  -configuration.maxSpeed, configuration.maxSpeed);
        speedLimit = 0.3
        steerGain = 0.5
        speedL = max(-speedLimit, min(speedLimit, float(y) + steerGain * float(x)))
        speedR = max(-speedLimit, min(speedLimit, float(y) - steerGain * float(x)))
        self.get_logger().info('Left speed se to: "%s"' % speedL)
        self.get_logger().info('Right speed se to: "%s"' % speedR)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def test(self, msg):
        self.get_logger().info('test: "%s"' % msg.data)

    def example_topic(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def joy_topic(self, msg):
        self.get_logger().info("joy X: " + str(msg.axes[0]) + " Y: " + str(msg.axes[1]))
        x = msg.axes[0]
        y = msg.axes[1]

        self.m(x, y)

    def move_web(self, msg):
        self.get_logger().info("joy X: " + str(msg.axes[0]) + " Y: " + str(msg.axes[1]))
        x = msg.axes[0]
        y = msg.axes[1]

        self.m(self, x, y)

    def stop_motors(self, msg):
        self.get_logger().info("I heard: " + str(msg.data))


def main(args=None):
    rclpy.init(args=args)

    robot = Robot()
    print("Ready")
    rclpy.spin(robot)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
