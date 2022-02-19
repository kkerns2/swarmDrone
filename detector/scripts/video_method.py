#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge

# Deep Learning imports
import torch

#sys method using detect.py
# import sys
# sys.path.append("/home/mcp/catkin_ws/src/detector/scripts/yolov5/")
# import detect



# model = torch.hub.load('ultralytics/yolov5', 'custom', repo_or_dir='custom.pt')

model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/mcp/custom.pt')
# model.conf = 0.7
# model.source = 0
# model = model.autoshape()  # for PIL/cv2/np inputs and NMS
# model.cuda()


# class Follower:
    
#     def __init__(self):
#         self.bridge = cv_bridge.CvBridge()
#         cv2.namedWindow("window", 1)
#         self.image_sub = rospy.Subscriber('/drone1/kinect/rgb/image_raw', Image, self.image_callback)


#     def image_callback(self, msg):
#         try:
#             image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
#         except cv_bridge.CvBridgeError as e:
#             print(e)
#         cv2.imshow("window", image)
#         cv2.waitKey(3)

def callback(msg):
    br = cv_bridge.CvBridge()

    rospy.loginfo("receiving video frame")

    current_frame = br.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    #insert YOLO Feed Here
    # result = model(current_frame, size=320)  # includes NMS
    result = model(cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB), size=320)

    # cv2.imshow("window", current_frame)
    cv2.imshow("window", result)
    # result.show()
    

    cv2.waitKey(200)

def receive_message():
    rospy.init_node('follower', anonymous=True)

    rospy.Subscriber('/drone1/front_cam/camera/image', Image, callback)
    rospy.spin()

    cv2.destroyAllWindows()

if __name__=='__main__':
    receive_message()
