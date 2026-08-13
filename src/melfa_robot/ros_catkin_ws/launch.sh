#!/bin/bash

# Exit immediately if a command fails
set -e

roslaunch melfa_driver melfa_driver.launch robot_ip:=192.168.0.23 --screen &

sleep 5

roslaunch keyence_plc_driver lrtb2000c.launch &

sleep 5

python3 ~/catkin_ws/src/keyence_plc_driver/scripts/camera_output_opencv.py

wait

