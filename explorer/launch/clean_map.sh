#!/bin/bash

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

rostopic pub /drone1/syscommand std_msgs/String "reset"

rostopic pub /drone2/syscommand std_msgs/String "reset"

rostopic pub /drone3/syscommand std_msgs/String "reset"

rostopic pub /drone4/syscommand std_msgs/String "reset"
