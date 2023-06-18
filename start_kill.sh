#!/bin/bash
source /home/pi/osu-uwrt/kill_switch/install/setup.bash
ROS_LOG_DIR=/tmp/roslogs ros2 launch robostop kill.launch.py
