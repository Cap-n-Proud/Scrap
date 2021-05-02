#!/bin/bash
sudo mount 192.168.1.121:/mnt/VDev/Software Software
sudo chmod 777 /dev/i2c-1
node /home/robot/Scrap/scrap_ws/src/ros2-web-bridge/bin/rosbridge.js &
node /home/robot/Scrap/web/index.js &
python3 /home/robot/Scrap/web/mjpeg_http/mjpeg_http/server.py --bind 192.168.1.164 &
# python3 /home/robot/Scrap/apps/stats.py &

cd /home/robot/Software/ROS2/Scrap/scrap_ws
source /opt/ros/foxy/setup.bash
. install/local_setup.bash

ros2 run py_drive listener
