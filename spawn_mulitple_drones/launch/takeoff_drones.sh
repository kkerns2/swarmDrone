#!/bin/bash

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

rostopic pub /drone1/takeoff std_msgs/Empty "{}"

rostopic pub /drone2/takeoff std_msgs/Empty "{}"

rostopic pub /drone3/takeoff std_msgs/Empty "{}"

rostopic pub /drone4/takeoff std_msgs/Empty "{}"
