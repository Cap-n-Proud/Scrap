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

import time
import Adafruit_SSD1306

from . import ads1115
from . import ina219

import os
import subprocess

# from apscheduler.schedulers.background import BackgroundScheduler


class Sys_Vars(Node):
    def __init__(self):
        super().__init__("minimal_publisher")

        adress = os.popen(
            "i2cdetect -y -r 1 0x48 0x48 | egrep '48' | awk '{print $2}'"
        ).read()
        if adress == "48\n":
            self.ads = ads1115.ADS1115()
        else:
            self.ads = None

        adress = os.popen(
            "i2cdetect -y -r 1 0x41 0x41 | egrep '41' | awk '{print $2}'"
        ).read()
        if adress == "41\n":
            self.ina = ina219.INA219(addr=0x41)
        else:
            self.ina = None

        self.info_sys_disk = self.create_publisher(String, "info_sys_disk", 10)
        self.info_sys_mem = self.create_publisher(String, "info_sys_mem", 10)
        self.info_sys_CPU = self.create_publisher(String, "info_sys_CPU", 10)

        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_info)
        self.i = 0

    def get_CPU(self):
        # Shell scripts for system monitoring from here : https://unix.stackexchange.com/questions/119126/command-to-display-memory-usage-disk-usage-and-cpu-load
        cmd = "top -bn1 | grep load | awk '{printf \"CPU Load: %.2f\", $(NF-2)}'"
        CPU = subprocess.check_output(cmd, shell=True)
        return CPU

    def get_mem(self):
        cmd = "free -m | awk 'NR==2{printf \"Mem:%s/%sMB %.1f%%\", $3,$2,$3*100/$2 }'"
        MemUsage = subprocess.check_output(cmd, shell=True)
        return MemUsage

    def get_disk(self):
        cmd = 'df -h | awk \'$NF=="/"{printf "Disk:%d/%dGB %s", $3,$2,$5}\''
        Disk = subprocess.check_output(cmd, shell=True)
        return Disk

    def publish_info(self):
        msg = String()
        msg.data = str(self.get_CPU())
        self.info_sys_CPU.publish(msg)
        self.get_logger().info('Publishing CPU: "%s"' % msg.data)

        msg.data = str(self.get_mem())
        self.info_sys_mem.publish(msg)
        self.get_logger().info('Publishing mem: "%s"' % msg.data)

        msg.data = str(self.get_disk())
        self.info_sys_disk.publish(msg)
        self.get_logger().info('Publishing disk: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    sys_var = Sys_Vars()

    rclpy.spin(sys_var)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sys_var.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
