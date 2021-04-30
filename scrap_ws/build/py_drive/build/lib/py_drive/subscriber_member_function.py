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

import time
from adafruit_motorkit import MotorKit


# https://github.com/adafruit/Adafruit_CircuitPython_MotorKit.git


class Robot(Node):
    def __init__(self):
        super().__init__("robot")
        self.kit = MotorKit()

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

    def _left_speed(self, speed):
        """Set the speed of the left motor, taking into account its trim offset."""
        assert -1 <= speed <= 1, "Speed must be a value between -1 to 1 inclusive!"
        speed += self._left_trim
        speed = max(-1, min(1, speed))  # Constrain speed to 0-255 after trimming.
        kit.motor1.throttle = speed

    def _right_speed(self, speed):
        """Set the speed of the right motor, taking into account its trim offset."""
        assert -1 <= speed <= 1, "Speed must be a value between -1 to 1 inclusive!"
        speed += self._right_trim
        speed = max(-1, min(1, speed))  # Constrain speed to 0-255 after trimming.
        kit.motor2.throttle = speed

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
        speed = msg.data.split(",")
        self.get_logger().info("Move: " + str(speed[0]) + "," + str(speed[1]))
        # self._left_speed(float(speed[0]))
        # self._right_speed(float(speed[1]))
        # currentSpeedL = -map(dY - configuration.steerGain * dX, -100, 100, -configuration.maxSpeed, configuration.maxSpeed);
        # currentSpeedR = -map(dY + configuration.steerGain * dX, -100, 100,  -configuration.maxSpeed, configuration.maxSpeed);
        speedLimit = 0.3
        steerGain = 0.3
        speedL = max(
            -speedLimit, min(speedLimit, float(speed[1]) + steerGain * float(speed[0]))
        )
        speedR = max(
            -speedLimit, min(speedLimit, float(speed[1]) - steerGain * float(speed[0]))
        )
        self.kit.motor1.throttle = speedL
        self.kit.motor2.throttle = -speedR

    def get_ip_address(interface):
        state = get_network_interface_state(interface)
        if state == "down" or state == None:
            return None

        cmd = (
            "ifconfig %s | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'"
            % interface
        )
        return subprocess.check_output(cmd, shell=True).decode("ascii")[:-1]

    def get_network_interface_state(interface):
        if not os.path.exists("/sys/class/net/%s/operstate" % interface):
            # print("%s file does NOT exist" % interface)
            return None

        try:
            status = subprocess.check_output(
                "cat /sys/class/net/%s/operstate" % interface, shell=True
            ).decode("ascii")[:-1]
        except Exception as err:
            print("Exception: {0}".format(err))
            return None
        else:
            return status


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
