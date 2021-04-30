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

import time
import os
import subprocess

# https://github.com/adafruit/Adafruit_CircuitPython_MotorKit.git


class Robot:
    def __init__(self, steer_gain, speed_limit, left_trim=0, right_trim=0):
        self.steer_gain = steer_gain
        self.speed_limit = speed_limit
        self.left_trim = left_trim
        self.right_trim = right_trim

    def set_steer_gain(self, steer_gain):
        self.steer_gain = steer_gain

    def set_speed_limit(self, speed_limit):
        self.speed_limit = speed_limit

    def left_speed(self, speed):
        """Set the speed of the left motor, taking into account its trim offset."""
        assert -1 <= speed <= 1, "Speed must be a value between -1 to 1 inclusive!"
        speed += self.left_trim
        speed = max(-1, min(1, speed))  # Constrain speed to 0-255 after trimming.
        # kit.motor1.throttle = speed
        print("left speed set to " + str(speed))

    def right_speed(self, speed):
        """Set the speed of the right motor, taking into account its trim offset."""
        assert -1 <= speed <= 1, "Speed must be a value between -1 to 1 inclusive!"
        speed += self.right_trim
        speed = max(-1, min(1, speed))  # Constrain speed to 0-255 after trimming.
        # kit.motor2.throttle = speed
        print("right speed set to " + str(speed))

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
        # self.get_logger().info("Move: " + str(speed[0]) + "," + str(speed[1]))
        self.left_speed(float(msg[0]))
        self.right_speed(float(msg[1]))
        # currentSpeedL = -map(dY - configuration.steerGain * dX, -100, 100, -configuration.maxSpeed, configuration.maxSpeed);
        # currentSpeedR = -map(dY + configuration.steerGain * dX, -100, 100,  -configuration.maxSpeed, configuration.maxSpeed);
        # speedLimit = 0.3
        # steerGain = 0.3
        # speedL = max(
        #     -speedLimit, min(speedLimit, float(speed[1]) + steerGain * float(speed[0]))
        # )
        # speedR = max(
        #     -speedLimit, min(speedLimit, float(speed[1]) - steerGain * float(speed[0]))
        # )
        # self.kit.motor1.throttle = speedL
        # self.kit.motor2.throttle = -speedR

    def get_ip_address(self):
        import netifaces as ni

        print(ni.interfaces())

        for interface in ni.interfaces():
            print(interface + str(ni.ifaddresses(interface)))
            # print(ni.ifaddresses(interface)[2][0]["addr"])
        #     # if interface != "lo":
        #     return ni.ifaddresses(interface)[2][0]["addr"]

    def get_network_interface_state(self, interface):
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
    robot = Robot(0.3, 0.5)
    print("Ready")
    print(robot.steer_gain)
    robot.set_steer_gain(1)
    print(robot.steer_gain)
    robot.move([0.2, 0.5])
    print(robot.get_ip_address())


if __name__ == "__main__":
    main()
