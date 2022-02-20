#!/usr/bin/env python
import cv2, cv_bridge
import os
import rospy
import torch
from sensor_msgs.msg import Image

from timeit import default_timer as timer

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


def score_frame(frame, model):
    # convert results to pandas dataframe
    results = model(frame)
    results = results.pandas().xyxy[0]


    # pandas[row, column]
    print(results) # uncomment to see the entire dataframe
    # print("labels")
    # print(results.iloc[:, -1:])
    # print("cords")
    # print(results.iloc[:, :-1])




    #original code snippit
    # labels = results.xyxyn[0][:, -1].numpy()
    # cord = results.xyxyn[0][:, :-1].numpy()
    # return labels, cord

    # new section
    # labels = results.iloc[:, -1:]
    # cords = results.iloc[:, :-1]

    # convert to np array for easier manipulation
    labels = results.iloc[:, -1:].to_numpy()
    cords = results.iloc[:, :-1].to_numpy()
    return labels, cords


def plot_boxes(results, frame, model):
    # example dataframe
    #      xmin    ymin    xmax   ymax  confidence  class    name
    # 0  749.50   43.50  1148.0  704.5    0.874023      0  person
    # 1  433.50  433.50   517.5  714.5    0.687988     27     tie
    # 2  114.75  195.75  1095.0  708.0    0.624512      0  person
    # 3  986.00  304.00  1028.0  420.0    0.286865     27     tie

    labels, cords = results
    n = len(labels)
    x_shape, y_shape = frame.shape[1], frame.shape[0]
    for i in range(n):
        row = cords[i]
        # If score is less than 0.4, do not predict
        if row[4] < 0.4:
            continue
        x1 = int(row[0])
        y1 = int(row[1])
        x2 = int(row[2])
        y2 = int(row[3])
        bgr = (0, 255, 0) # color of the box
        # classes = model.names # Get the name of label index
        # print(classes)
        label_font = cv2.FONT_HERSHEY_SIMPLEX #Font for the label.

        frame = cv2.rectangle(frame, \
                      (x1, y1), (x2, y2), \
                       bgr, 2) #Plot the boxes
        # cv2.imshow('window', frame)

        # cv2.putText(frame,\
        #             classes[labels[i]], \
        #             (x1, y1), \
        #             label_font, 0.9, bgr, 2) #Put a label over box.

        
        cv2.putText(frame,\
                    str(labels[i]), \
                    (x1, y1), \
                    label_font, 0.9, bgr, 2) #Put a label over box.
        return frame

# def __call__(self):
#     player = self.get_video_stream() #Get your video stream.
#     assert player.isOpened() # Make sure that their is a stream. 
#     #Below code creates a new video writer object to write our
#     #output stream.
#     x_shape = int(player.get(cv2.CAP_PROP_FRAME_WIDTH))
#     y_shape = int(player.get(cv2.CAP_PROP_FRAME_HEIGHT))
#     four_cc = cv2.VideoWriter_fourcc(*"MJPG") #Using MJPEG codex
#     out = cv2.VideoWriter(out_file, four_cc, 20, \
#                           (x_shape, y_shape)) 
#     ret, frame = player.read() # Read the first frame.
#     while rect: # Run until stream is out of frames
#         start_time = time() # We would like to measure the FPS.
#         results = self.score_frame(frame) # Score the Frame
#         frame = self.plot_boxes(results, frame) # Plot the boxes.
#         end_time = time()
#         fps = 1/np.round(end_time - start_time, 3) #Measure the FPS.
#         print(f"Frames Per Second : {fps}")
#         out.write(frame) # Write the frame onto the output.
#         ret, frame = player.read() # Read next frame.


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
    # rospy.loginfo("receiving video frame")
    current_frame = br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    # convert bgr -> rgb for image detector to process
    frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)

    # cv2.imshow('window', current_frame)

    # score_frame(frame, model)
    start = timer()
    results = score_frame(frame, model)

    # print(results)
    frame = plot_boxes(results, frame, model)

    end = timer()
    print(end - start)

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
