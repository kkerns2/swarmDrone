#!/usr/bin/bash

rosrun detector yolo_multi.py drone1 drone2 &
rosrun detector yolo_multi.py drone3 drone4 &
