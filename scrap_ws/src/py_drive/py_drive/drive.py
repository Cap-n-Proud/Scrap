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
    def __init__(self, steer_gain=0.5, speed_limit=0.3, left_trim=0, right_trim=0):
        super().__init__("robot")
        self.kit = MotorKit()

        self.steer_gain = steer_gain
        self.speed_limit = speed_limit
        self.left_trim = left_trim
        self.right_trim = right_trim

        self.joy_topic = self.create_subscription(Joy, "joy", self.joy_topic, 10)
        self.joy_web = self.create_subscription(String, "joy_web", self.joy_web, 10)

    def set_steer_gain(self, steer_gain):
        self.steer_gain = steer_gain

    def set_speed_limit(self, speed_limit):
        self.speed_limit = speed_limit

    def set_left_trim(self, left_trim):
        self.left_trim = left_trim

    def set_right_trim(self, right_trim):
        self.right_trim = right_trim

    def left_speed(self, speed):
        """Set the speed of the left motor, taking into account its trim offset."""
        assert -1 <= speed <= 1, "Speed must be a value between -1 to 1 inclusive!"
        speed += self.left_trim
        speed = max(-1, min(1, speed))
        kit.motor1.throttle = speed

    def right_speed(self, speed):
        """Set the speed of the right motor, taking into account its trim offset."""
        assert -1 <= speed <= 1, "Speed must be a value between -1 to 1 inclusive!"
        speed += self.right_trim
        speed = max(-1, min(1, speed))
        kit.motor2.throttle = speed

    def move(self, x, y):
        # speed = msg.data.split(",")
        # self.get_logger().info("Move: " + str(speed[0]) + "," + str(speed[1]))
        # # self._left_speed(float(speed[0]))
        # self._right_speed(float(speed[1]))
        # currentSpeedL = -map(dY - configuration.steerGain * dX, -100, 100, -configuration.maxSpeed, configuration.maxSpeed);
        # currentSpeedR = -map(dY + configuration.steerGain * dX, -100, 100,  -configuration.maxSpeed, configuration.maxSpeed);
        speedLimit = 0.3
        steerGain = 0.5
        speedL = max(
            -self.speedLimit, min(self.speedLimit, float(y) + self.steerGain * float(x))
        )
        speedR = max(
            -self.speedLimit, min(self.speedLimit, float(y) - self.steerGain * float(x))
        )
        self.get_logger().info('Left speed se to: "%s"' % speedL)
        self.get_logger().info('Right speed se to: "%s"' % speedR)

    def joy_topic(self, msg):
        self.get_logger().info("joy X: " + str(msg.axes[0]) + " Y: " + str(msg.axes[1]))
        self.move(msg.axes[0], msg.axes[1])

    def joy_web(self, msg):
        speed = msg.data.split(",")
        self.move(speed[0], speed[1])


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
