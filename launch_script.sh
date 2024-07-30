#!/bin/bash
# export ROS_MASTER_URI=http://192.168.1.2:11311
roslaunch deprojection_pipeline bringup.launch &
roscd deprojection_pipeline/scripts
python3 ultralytics_yolo.py
