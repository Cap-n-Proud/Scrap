#!/bin/bash
sudo mount 192.168.1.121:/mnt/VDev/Software Software
# sudo chmod 777 /dev/i2c-1
# node /home/robot/Madox/madox_ws/src/ros2-web-bridge/bin/rosbridge.js &
# node /home/robot/Madox/web/index.js &
# python3 /home/robot/Madox/web/mjpeg_http/mjpeg_http/server.py --bind 192.168.1.164 &
# python3 /home/robot/Madox/apps/stats.py &

cd /home/robot/Software/ROS2/Madox/madox_ws
source /opt/ros/foxy/setup.bash
. install/local_setup.bash

ros2 run py_drive listener -s &
ros2 run py_camera camera  &
