#!/bin/bash

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

rostopic pub /drone4/syscommand std_msgs/String "reset"