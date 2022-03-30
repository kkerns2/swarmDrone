#!/usr/bin/env python
import cv2, cv_bridge
import os
import rospy
import sys
import torch
import threading
from sensor_msgs.msg import Image
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalID
import subprocess
import signal

lock_yolo = threading.Lock()
lock_frames = threading.Lock()

global child_1
global child_2


num_Drones = len(sys.argv) - 1
drone_names = []
frame_buffer = [None] * num_Drones
if num_Drones >= 1:
    for arg in sys.argv[1:]:
        drone_names.append(arg)
        
else:
    drone_names.append('drone1')
    frame_buffer = [None] * 1

if(drone_names[0] == 'drone1'): #if drone1 or drone2
    child_1 = subprocess.Popen(["roslaunch","swarm","start_move_1.launch"])

elif(drone_names[0] == 'drone3'):
    child_2 = subprocess.Popen(["roslaunch","swarm","start_move_2.launch"])

# PREFERENCES
box_thickness = 1       
bgr = (0, 255, 0)   # color of box and label
label_font = cv2.FONT_HERSHEY_DUPLEX
label_scale = 0.5
label_thickness = 1

# load model
dirname = os.path.dirname(__file__)
model_path = os.path.join(dirname, 'custom.pt')
model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)
device = 'cuda' if torch.cuda.is_available() else 'cpu'
model.to(device)
# model.conf = 0.9



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


def plot_boxes(results, frame):
    cords, labels = results
    # x_shape, y_shape = frame.shape[1], frame.shape[0]
    for i, label in enumerate(labels):
        row = cords[i]

        # If score is less than 0.4, do not predict
        if row[4] < 0.7:
            continue
        x1 = int(row[0])
        y1 = int(row[1])
        x2 = int(row[2])
        y2 = int(row[3])

        # draw bounding box
        frame = cv2.rectangle(frame, \
                      (x1, y1), (x2, y2), \
                       bgr, box_thickness)

        # apply label over bounding box
        name = label[0]
        conf = str(round(row[4], 2))
        label = f'{name} {conf}'
        # print(label)

        # draw black outline
        cv2.putText(frame,\
                    label, \
                    (x1, y1-2), \
                    label_font, label_scale, (0, 0, 0), label_thickness+2)
        # draw text over outline
        cv2.putText(frame,\
                    label, \
                    (x1, y1-2), \
                    label_font, label_scale, bgr, label_thickness)

        # SEND A MESSAGE/TOPIC TO THE DRONE THAT SPOTS BB8 WITH A 96% AND STOP MOVEMENT/MOVEBASE
        # MAYBE A CANCEL GOAL? https://answers.ros.org/question/46214/move_base-stop-navigation/
        # Or kill the node 

        

        # os.system("rosnode kill /drone1_client_py")
        # os.system("rosnode kill /drone2_client_py")
        # os.system("rosnode kill /drone3_client_py")
        # os.system("rosnode kill /drone4_client_py")

        #rospy.sleep(15)
        if row[4] > 0.95:
            
            if(drone_names[0] == 'drone1'): #if drone1 or drone2
                child_1.send_signal(signal.SIGINT)

                # client1 = actionlib.SimpleActionClient('/drone1/move_base', MoveBaseAction)
                # client2 = actionlib.SimpleActionClient('/drone2/move_base', MoveBaseAction)
                
                # wait1 = client1.wait_for_server()
                # wait2 = client2.wait_for_server()

                # print("wait 1: ", wait1)
                # print("wait 2: ", wait2)

                # if wait1 == True:
                cancel_pub_1 = rospy.Publisher("/drone1/move_base/cancel", GoalID, queue_size=1)
                cancel_msg_1 = GoalID()
                cancel_pub_1.publish(cancel_msg_1)

                cancel_pub_2 = rospy.Publisher("/drone2/move_base/cancel", GoalID, queue_size=1)
                cancel_msg_2 = GoalID()
                cancel_pub_2.publish(cancel_msg_2)

                #     # cancels the next goal
                #     client2.cancel_goal() 
                #     client1.cancel_goal()
                # else:
                #     rospy.logerr("Drone 1 not reached") 


            elif(drone_names[0] == 'drone3'):
                child_2.send_signal(signal.SIGINT)

                # client3 = actionlib.SimpleActionClient('/drone3/move_base', MoveBaseAction)
                # client4 = actionlib.SimpleActionClient('/drone4/move_base', MoveBaseAction)
                
                # wait3 = client3.wait_for_server()
                # wait4 = client4.wait_for_server()

                # print("wait 3: ", wait3)
                # print("wait 4: ", wait4)


                # if wait3 == True:
                cancel_pub_3 = rospy.Publisher("/drone3/move_base/cancel", GoalID, queue_size=1)
                cancel_msg_3 = GoalID()
                cancel_pub_3.publish(cancel_msg_3)

                cancel_pub_4 = rospy.Publisher("/drone4/move_base/cancel", GoalID, queue_size=1)
                cancel_msg_4 = GoalID()
                cancel_pub_4.publish(cancel_msg_4)

                #     client4.cancel_goal()
                #     client3.cancel_goal()

                # else:
                #     rospy.logerr("Drone 3 not reached") 
    return frame


def callback(msg, args):
    i = args

    br = cv_bridge.CvBridge()
    current_frame = br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    # convert bgr -> rgb for image detector to process
    frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)

    # if lock_yolo.acquire() == False: return
    results = inference(frame, model)
    # lock_yolo.release()

    # print(results)
    frame = plot_boxes(results, frame)

    # convert back rgb -> to brg for cv2 to display
    lock_frames.acquire()
    frame_buffer[i] = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    lock_frames.release()


def thread_imshow():
    while(any(frame is None for frame in frame_buffer)):
        rospy.sleep(1)

    while(any(frame is not None for frame in frame_buffer)):
        lock_frames.acquire()
        for i, drone_name in enumerate(drone_names):
            cv2.imshow(drone_name, frame_buffer[i])
            cv2.waitKey(1)
        lock_frames.release()
        rospy.sleep(0.01)


def receive_message():
    rospy.init_node('follower', anonymous=True)
    # subscribe to all the drones provided in argv


    for i, drone_name in enumerate(drone_names):
    # for drone_name in drone_names:
        topic = f'{drone_name}/front_cam/camera/image'
        rospy.Subscriber(topic, Image, callback, (i))


    t = threading.Thread(target=thread_imshow)
    t.start()

    rospy.spin()
    lock_frames.acquire()
    frame_buffer[0] = None
    lock_frames.release()
    cv2.destroyAllWindows()


if __name__=='__main__':
    receive_message()
