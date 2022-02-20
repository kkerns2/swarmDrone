#!/usr/bin/env python
import cv2, cv_bridge
import os
import rospy
import torch
from sensor_msgs.msg import Image

# PREFERENCES
box_thickness = 1       
bgr = (0, 255, 0)   # color of box and label
label_font = cv2.FONT_HERSHEY_SIMPLEX
label_scale = 0.5
label_thickness = 1

# load model
dirname = os.path.dirname(__file__)
model_path = os.path.join(dirname, 'custom.pt')
model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)

device = 'cuda' if torch.cuda.is_available() else 'cpu'
model.to(device)

# model.conf = 0.7
# model.source = 0
# model = model.autoshape()  # for PIL/cv2/np inputs and NMS
# model.cuda()


def inference(frame, model):
    #   Example Output Dataframe
    #   xmin    ymin    xmax    ymax    confidence  class   name
    #0  156.79  96.36   176.80  122.60  0.96815     0       bb8
    #1  144.51  92.32   168.89  124.32  0.969062    0       bb8


    # convert results from inference
    # np array is slightly faster, but uglier to look at
    # only use pandas for viewing the results dataframe temporarily
    results = model(frame)
    np_results = results.pandas().xyxy[0].to_numpy() # np array
    # pd_results = results.pandas().xyxy[0] # pd dataframe
    # print(pd_results)

    # slicing:[rows, columns]
    cords = np_results[:, :-1]
    labels = np_results[:, -1:]
    return cords, labels


def plot_boxes(results, frame, model):
    cords, labels = results
    n = len(labels)
    # x_shape, y_shape = frame.shape[1], frame.shape[0]
    for i in range(n):
        row = cords[i]
        # If score is less than 0.4, do not predict
        if row[4] < 0.4:
            continue
        x1 = int(row[0])
        y1 = int(row[1])
        x2 = int(row[2])
        y2 = int(row[3])
        # classes = model.names # Get the name of label index
        # print(classes)


        # draw bounding box
        frame = cv2.rectangle(frame, \
                      (x1, y1), (x2, y2), \
                       bgr, box_thickness)


        # apply label over bounding box
        name = labels.ravel()[i]
        conf = str(round(row[4], 2))
        label = ''.join([name, ' ', conf])
        # print(label)

        cv2.putText(frame,\
                    label, \
                    (x1, y1-2), \
                    label_font, label_scale, bgr, label_thickness)

    return frame


def callback(msg):
    br = cv_bridge.CvBridge()
    # rospy.loginfo("receiving video frame")
    current_frame = br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    # convert bgr -> rgb for image detector to process
    frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
    # cv2.imshow('window', current_frame)


    results = inference(frame, model)
    # print(results)
    frame = plot_boxes(results, frame, model)


    # convert back rgb -> to brg for cv2 to display
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    cv2.imshow('window', frame)


    #insert YOLO Feed Here
    # result = model(current_frame, size=320)  # includes NMS
    # result = model(cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB), size=320)

    # cv2.imshow("window", current_frame)
    # cv2.imshow("window", result)
    # result.show()

    cv2.waitKey(1)

def receive_message():
    rospy.init_node('follower', anonymous=True)

    rospy.Subscriber('/drone1/front_cam/camera/image', Image, callback)
    rospy.spin()

    cv2.destroyAllWindows()

if __name__=='__main__':
    receive_message()