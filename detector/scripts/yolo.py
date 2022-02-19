#!/usr/bin/env python
import rospy
import sys

#sys.path.append('catkin_ws/src/yolov5')

def detector():
    #takes in current camera feed pipes it to detect.py and then returns value to camera feed
    pass
if __name__ == '__main__':
    try:
        detector()
    except rospy.ROSInterruptException:
        pass
