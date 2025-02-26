#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

exec ros2 launch rover_launch rover.launch.py
