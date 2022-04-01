#!/bin/bash

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

rostopic pub /drone2/syscommand std_msgs/String "reset"
