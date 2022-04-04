#!/bin/bash

source /opt/ros/noetic/setup.bash
source ~/testing_ws/devel/setup.bash

rostopic pub /drone3/syscommand std_msgs/String "reset"
