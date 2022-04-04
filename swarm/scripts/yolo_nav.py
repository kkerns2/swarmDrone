#!/usr/bin/env python
import cv2, cv_bridge
import os
import rospy
import sys
import torch
import threading
from sensor_msgs.msg import Image
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from actionlib_msgs.msg import GoalID
import subprocess
import signal
from geometry_msgs.msg import PoseWithCovarianceStamped   

lock_yolo = threading.Lock()
lock_frames = threading.Lock()

drone_paused = False 

num_Drones = len(sys.argv) - 1
drone_names = []
frame_buffer = [None] * num_Drones
if num_Drones >= 1:
    for arg in sys.argv[1:]:
        drone_names.append(arg)
        
else:
    drone_names.append('drone1')
    frame_buffer = [None] * 1

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

drone_x = 0 
drone_y = 0
drone_z = 0

drone_orien_x = 0
drone_orien_y = 0
drone_orien_z = 0
drone_orien_w = 0 

drone_flag = False

leader_d1 = False
leader_d2 = False
leader_d3 = False 
leader_d4 = False

firstTime = 0 

def retrieve_positions(msg):
    global drone_x
    global drone_y
    global drone_z 

    global drone_orien_x 
    global drone_orien_y
    global drone_orien_z 
    global drone_orien_w

     
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z

    d_x = msg.pose.pose.orientation.x
    d_y = msg.pose.pose.orientation.y
    d_z = msg.pose.pose.orientation.z
    d_w = msg.pose.pose.orientation.w
    
    drone_x = x 
    drone_y = y
    drone_z = z 

    drone_orien_x = d_x
    drone_orien_y = d_y 
    drone_orien_z = d_z  
    drone_orien_w = d_w 

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

def plot_boxes(results, frame, drone_name):
    global child_1
    global child_2
    global drone_paused 

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

        if row[4] > 0.96:
            
            global drone_x
            global drone_y
            global drone_z 
            
            global drone_orien_x 
            global drone_orien_y
            global drone_orien_z
            global drone_orien_w
            
            global drone_flag
            
            global leader_d1 
            global leader_d2 
            global leader_d3  
            global leader_d4 
            global firstTime  

           # if(drone_flag == False):

            if((drone_name == 'drone1' and firstTime == 0) or (leader_d1 == True)):
                print('Drone 1 Spotted BB8!')
                print('ld1', leader_d1)
                firstTime = 1
                leader_d1 = True

                # Set all 4 drones to cancel their goals: 
                
                # Drone 1 goal cancel
                cancel_pub_1 = rospy.Publisher("/drone1/move_base/cancel", GoalID, queue_size=1)
                cancel_msg_1 = GoalID()
                cancel_pub_1.publish(cancel_msg_1)

                # Drone 2 goal cancel
                cancel_pub_2 = rospy.Publisher("/drone2/move_base/cancel", GoalID, queue_size=1)
                cancel_msg_2 = GoalID()
                cancel_pub_2.publish(cancel_msg_2)

                # Drone 3 goal cancel
                cancel_pub_3 = rospy.Publisher("/drone3/move_base/cancel", GoalID, queue_size=1)
                cancel_msg_3 = GoalID()
                cancel_pub_3.publish(cancel_msg_3)

                # Drone 4 goal cancel
                cancel_pub_4 = rospy.Publisher("/drone4/move_base/cancel", GoalID, queue_size=1)
                cancel_msg_4 = GoalID()
                cancel_pub_4.publish(cancel_msg_4)
        
                # Make sure the other 3 drones goals are cancel 
                # client_1 = actionlib.SimpleActionClient('/drone1/move_base', MoveBaseAction)
                client_2 = actionlib.SimpleActionClient('/drone2/move_base', MoveBaseAction)
                client_3 = actionlib.SimpleActionClient('/drone3/move_base', MoveBaseAction)
                client_4 = actionlib.SimpleActionClient('/drone4/move_base', MoveBaseAction)

                # Cancel all 3 drones                    
                # client_1.cancel_goal()
                client_2.cancel_goal()
                client_3.cancel_goal()
                client_4.cancel_goal()

                # Receive drone 1 position
                pose_sub_1 = rospy.Subscriber('/drone1/amcl_pose', PoseWithCovarianceStamped, retrieve_positions)

                # Get the action state for goals
                d2_action = actionlib.SimpleActionClient('/drone2/move_base', MoveBaseAction)
                d3_action = actionlib.SimpleActionClient('/drone3/move_base', MoveBaseAction)
                d4_action = actionlib.SimpleActionClient('/drone4/move_base', MoveBaseAction)

                goal_2 = MoveBaseGoal()
                goal_3 = MoveBaseGoal()
                goal_4 = MoveBaseGoal()

                goal_2.target_pose.header.frame_id = 'map' 
                goal_2.target_pose.pose.position.x = drone_x +  5.0
                goal_2.target_pose.pose.position.y = drone_y + 5.0
                goal_2.target_pose.pose.position.z = drone_z 
                goal_2.target_pose.pose.orientation.x = drone_orien_x
                goal_2.target_pose.pose.orientation.y = drone_orien_y
                goal_2.target_pose.pose.orientation.z = drone_orien_z
                goal_2.target_pose.pose.orientation.w = drone_orien_w

                goal_3.target_pose.header.frame_id = 'map' 
                goal_3.target_pose.pose.position.x = drone_x -  5.0
                goal_3.target_pose.pose.position.y = drone_y -  5.0
                goal_3.target_pose.pose.position.z = drone_z 
                goal_3.target_pose.pose.orientation.x = drone_orien_x
                goal_3.target_pose.pose.orientation.y = drone_orien_y
                goal_3.target_pose.pose.orientation.z = drone_orien_z 
                goal_3.target_pose.pose.orientation.w = drone_orien_w

                goal_4.target_pose.header.frame_id = 'map' 
                goal_4.target_pose.pose.position.x = drone_x - 7.0
                goal_4.target_pose.pose.position.y = drone_y - 7.0
                goal_4.target_pose.pose.position.z = drone_z 
                goal_4.target_pose.pose.orientation.x = drone_orien_x
                goal_4.target_pose.pose.orientation.y = drone_orien_y
                goal_4.target_pose.pose.orientation.z = drone_orien_z 
                goal_4.target_pose.pose.orientation.w = drone_orien_w

                d2_action.send_goal(goal_2) 
                d3_action.send_goal(goal_3) 
                d4_action.send_goal(goal_4) 
                

            elif((drone_name == 'drone2' and firstTime == 0) or (leader_d2 == True)):
                print('Drone 2 Spotted BB8!')
                print('ld2', leader_d2)

                firstTime = 1
                leader_d2 = True

                # Set all 4 drones to cancel their goals: 
                
                # Drone 1 goal cancel
                cancel_pub_1 = rospy.Publisher("/drone1/move_base/cancel", GoalID, queue_size=1)
                cancel_msg_1 = GoalID()
                cancel_pub_1.publish(cancel_msg_1)

                # Drone 2 goal cancel
                cancel_pub_2 = rospy.Publisher("/drone2/move_base/cancel", GoalID, queue_size=1)
                cancel_msg_2 = GoalID()
                cancel_pub_2.publish(cancel_msg_2)

                # Drone 3 goal cancel
                cancel_pub_3 = rospy.Publisher("/drone3/move_base/cancel", GoalID, queue_size=1)
                cancel_msg_3 = GoalID()
                cancel_pub_3.publish(cancel_msg_3)

                # Drone 4 goal cancel
                cancel_pub_4 = rospy.Publisher("/drone4/move_base/cancel", GoalID, queue_size=1)
                cancel_msg_4 = GoalID()
                cancel_pub_4.publish(cancel_msg_4)
        
                # Make sure the other 3 drones goals are cancel 
                client_1 = actionlib.SimpleActionClient('/drone1/move_base', MoveBaseAction)
                client_2 = actionlib.SimpleActionClient('/drone2/move_base', MoveBaseAction)
                client_3 = actionlib.SimpleActionClient('/drone3/move_base', MoveBaseAction)
                client_4 = actionlib.SimpleActionClient('/drone4/move_base', MoveBaseAction)

                # Cancel all 3 drones                    
                client_1.cancel_goal()
                client_2.cancel_goal()
                client_3.cancel_goal()
                client_4.cancel_goal()

                # Receive drone 2 position
                pose_sub_2 = rospy.Subscriber('/drone2/amcl_pose', PoseWithCovarianceStamped, retrieve_positions)

                # Get the action state for goals
                d1_action = actionlib.SimpleActionClient('/drone1/move_base', MoveBaseAction)
                d3_action = actionlib.SimpleActionClient('/drone3/move_base', MoveBaseAction)
                d4_action = actionlib.SimpleActionClient('/drone4/move_base', MoveBaseAction)

                goal_1 = MoveBaseGoal()
                goal_3 = MoveBaseGoal()
                goal_4 = MoveBaseGoal()

                goal_1.target_pose.header.frame_id = 'map' 
                goal_1.target_pose.pose.position.x = drone_x +  5.0
                goal_1.target_pose.pose.position.y = drone_y +  5.0
                goal_1.target_pose.pose.position.z = drone_z 
                goal_1.target_pose.pose.orientation.x = drone_orien_x
                goal_1.target_pose.pose.orientation.y = drone_orien_y
                goal_1.target_pose.pose.orientation.z = drone_orien_z 
                goal_1.target_pose.pose.orientation.w = drone_orien_w

                goal_3.target_pose.header.frame_id = 'map' 
                goal_3.target_pose.pose.position.x = drone_x -  5.0
                goal_3.target_pose.pose.position.y = drone_y -  5.0
                goal_3.target_pose.pose.position.z = drone_z 
                goal_3.target_pose.pose.orientation.x = drone_orien_x
                goal_3.target_pose.pose.orientation.y = drone_orien_y
                goal_3.target_pose.pose.orientation.z = drone_orien_z 
                goal_3.target_pose.pose.orientation.w = drone_orien_w

                goal_4.target_pose.header.frame_id = 'map' 
                goal_4.target_pose.pose.position.x = drone_x - 7.0
                goal_4.target_pose.pose.position.y = drone_y - 7.0
                goal_4.target_pose.pose.position.z = drone_z 
                goal_4.target_pose.pose.orientation.x = drone_orien_x
                goal_4.target_pose.pose.orientation.y = drone_orien_y
                goal_4.target_pose.pose.orientation.z = drone_orien_z 
                goal_4.target_pose.pose.orientation.w = drone_orien_w

                d1_action.send_goal(goal_1) 
                d3_action.send_goal(goal_3) 
                d4_action.send_goal(goal_4) 
            
            
            elif((drone_name == 'drone3' and firstTime == 0) or (leader_d3 == True)):
                print('Drone 3 Spotted BB8!')
                print('ld1', leader_d3)

                firstTime = 1
                leader_d3 = True
                # Set all 4 drones to cancel their goals: 
                
                # Drone 1 goal cancel
                cancel_pub_1 = rospy.Publisher("/drone1/move_base/cancel", GoalID, queue_size=1)
                cancel_msg_1 = GoalID()
                cancel_pub_1.publish(cancel_msg_1)

                # Drone 2 goal cancel
                cancel_pub_2 = rospy.Publisher("/drone2/move_base/cancel", GoalID, queue_size=1)
                cancel_msg_2 = GoalID()
                cancel_pub_2.publish(cancel_msg_2)

                # Drone 3 goal cancel
                cancel_pub_3 = rospy.Publisher("/drone3/move_base/cancel", GoalID, queue_size=1)
                cancel_msg_3 = GoalID()
                cancel_pub_3.publish(cancel_msg_3)

                # Drone 4 goal cancel
                cancel_pub_4 = rospy.Publisher("/drone4/move_base/cancel", GoalID, queue_size=1)
                cancel_msg_4 = GoalID()
                cancel_pub_4.publish(cancel_msg_4)
        
                # Make sure the other 3 drones goals are cancel 
                client_1 = actionlib.SimpleActionClient('/drone1/move_base', MoveBaseAction)
                client_2 = actionlib.SimpleActionClient('/drone2/move_base', MoveBaseAction)
                client_3 = actionlib.SimpleActionClient('/drone3/move_base', MoveBaseAction)
                client_4 = actionlib.SimpleActionClient('/drone4/move_base', MoveBaseAction)

                # Cancel all 3 drones                    
                client_1.cancel_goal()
                client_2.cancel_goal()
                client_3.cancel_goal()
                client_4.cancel_goal()

                # Receive drone 3 position
                pose_sub_3 = rospy.Subscriber('/drone3/amcl_pose', PoseWithCovarianceStamped, retrieve_positions)

                # Get the action state for goals
                d1_action = actionlib.SimpleActionClient('/drone1/move_base', MoveBaseAction)
                d2_action = actionlib.SimpleActionClient('/drone2/move_base', MoveBaseAction)
                d4_action = actionlib.SimpleActionClient('/drone4/move_base', MoveBaseAction)

                goal_1 = MoveBaseGoal()
                goal_2 = MoveBaseGoal()
                goal_4 = MoveBaseGoal()

                goal_1.target_pose.header.frame_id = 'map' 
                goal_1.target_pose.pose.position.x = drone_x + 5.0
                goal_1.target_pose.pose.position.y = drone_y +  5.0
                goal_1.target_pose.pose.position.z = drone_z 
                goal_1.target_pose.pose.orientation.x = drone_orien_x
                goal_1.target_pose.pose.orientation.y = drone_orien_y
                goal_1.target_pose.pose.orientation.z = drone_orien_z 
                goal_1.target_pose.pose.orientation.w = drone_orien_w

                goal_2.target_pose.header.frame_id = 'map' 
                goal_2.target_pose.pose.position.x = drone_x -  5.0
                goal_2.target_pose.pose.position.y = drone_y -  5.0
                goal_2.target_pose.pose.position.z = drone_z 
                goal_2.target_pose.pose.orientation.x = drone_orien_x
                goal_2.target_pose.pose.orientation.y = drone_orien_y
                goal_2.target_pose.pose.orientation.z = drone_orien_z 
                goal_2.target_pose.pose.orientation.w = drone_orien_w

                goal_4.target_pose.header.frame_id = 'map' 
                goal_4.target_pose.pose.position.x = drone_x - 7.0
                goal_4.target_pose.pose.position.y = drone_y - 7.0
                goal_4.target_pose.pose.position.z = drone_z 
                goal_4.target_pose.pose.orientation.x = drone_orien_x
                goal_4.target_pose.pose.orientation.y = drone_orien_y
                goal_4.target_pose.pose.orientation.z = drone_orien_z
                goal_4.target_pose.pose.orientation.w = drone_orien_w

                d1_action.send_goal(goal_1) 
                d2_action.send_goal(goal_2) 
                d4_action.send_goal(goal_4) 
                

            elif((drone_name == 'drone4' and firstTime == 0) or (leader_d4 == True)):
                print('Drone 4 Spooted BB8!')
                print('ld4', leader_d4)

                # Flag for true                  
                firstTime = 1
                leader_d4 = True

                # Set all 4 drones to cancel their goals: 
                
                # Drone 1 goal cancel
                cancel_pub_1 = rospy.Publisher("/drone1/move_base/cancel", GoalID, queue_size=1)
                cancel_msg_1 = GoalID()
                cancel_pub_1.publish(cancel_msg_1)

                # Drone 2 goal cancel
                cancel_pub_2 = rospy.Publisher("/drone2/move_base/cancel", GoalID, queue_size=1)
                cancel_msg_2 = GoalID()
                cancel_pub_2.publish(cancel_msg_2)

                # Drone 3 goal cancel
                cancel_pub_3 = rospy.Publisher("/drone3/move_base/cancel", GoalID, queue_size=1)
                cancel_msg_3 = GoalID()
                cancel_pub_3.publish(cancel_msg_3)

                # Drone 4 goal cancel
                cancel_pub_4 = rospy.Publisher("/drone4/move_base/cancel", GoalID, queue_size=1)
                cancel_msg_4 = GoalID()
                cancel_pub_4.publish(cancel_msg_4)
        
                # Make sure the other 3 drones goals are cancel 
                client_1 = actionlib.SimpleActionClient('/drone1/move_base', MoveBaseAction)
                client_2 = actionlib.SimpleActionClient('/drone2/move_base', MoveBaseAction)
                client_3 = actionlib.SimpleActionClient('/drone3/move_base', MoveBaseAction)
                client_4 = actionlib.SimpleActionClient('/drone4/move_base', MoveBaseAction)

                # Cancel all 3 drones                    
                client_1.cancel_goal()
                client_2.cancel_goal()
                client_3.cancel_goal()
                client_4.cancel_goal()

                # Receive drone 3 position
                pose_sub_4 = rospy.Subscriber('/drone4/amcl_pose', PoseWithCovarianceStamped, retrieve_positions)

                # Get the action state for goals
                d1_action = actionlib.SimpleActionClient('/drone1/move_base', MoveBaseAction)
                d2_action = actionlib.SimpleActionClient('/drone2/move_base', MoveBaseAction)
                d3_action = actionlib.SimpleActionClient('/drone3/move_base', MoveBaseAction)

                goal_1 = MoveBaseGoal()
                goal_2 = MoveBaseGoal()
                goal_3 = MoveBaseGoal()
                
                goal_1.target_pose.header.frame_id = 'map' 
                goal_1.target_pose.pose.position.x = drone_x + 5.0
                goal_1.target_pose.pose.position.y = drone_y + 5.0
                goal_1.target_pose.pose.position.z = drone_z 
                goal_1.target_pose.pose.orientation.x = drone_orien_x
                goal_1.target_pose.pose.orientation.y = drone_orien_y
                goal_1.target_pose.pose.orientation.z = drone_orien_z 
                goal_1.target_pose.pose.orientation.w = drone_orien_w

                goal_2.target_pose.header.frame_id = 'map' 
                goal_2.target_pose.pose.position.x = drone_x - 5.0
                goal_2.target_pose.pose.position.y = drone_y - 5.0
                goal_2.target_pose.pose.position.z = drone_z 
                goal_2.target_pose.pose.orientation.x = drone_orien_x
                goal_2.target_pose.pose.orientation.y = drone_orien_y
                goal_2.target_pose.pose.orientation.z = drone_orien_z 
                goal_2.target_pose.pose.orientation.w = drone_orien_w

                goal_3.target_pose.header.frame_id = 'map' 
                goal_3.target_pose.pose.position.x = drone_x - 7.0
                goal_3.target_pose.pose.position.y = drone_y - 7.0
                goal_3.target_pose.pose.position.z = drone_z 
                goal_3.target_pose.pose.orientation.x = drone_orien_x
                goal_3.target_pose.pose.orientation.y = drone_orien_y
                goal_3.target_pose.pose.orientation.z = drone_orien_z 
                goal_3.target_pose.pose.orientation.w = drone_orien_w

                d1_action.send_goal(goal_1) 
                d2_action.send_goal(goal_2) 
                d3_action.send_goal(goal_3) 

    return frame


def callback(msg, args):
    i = args[0] # msg - image args = drone count
    topic = args[1]

    drone_name = topic.replace("/front_cam/camera/image", "")

    br = cv_bridge.CvBridge()
    current_frame = br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    # convert bgr -> rgb for image detector to process
    frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)

    # if lock_yolo.acquire() == False: return
    results = inference(frame, model)
    # lock_yolo.release()

    # print(results)
    frame = plot_boxes(results, frame, drone_name)

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
        rospy.Subscriber(topic, Image, callback, (i, topic))


    t = threading.Thread(target=thread_imshow)
    t.start()

    rospy.spin()
    lock_frames.acquire()
    frame_buffer[0] = None
    lock_frames.release()
    cv2.destroyAllWindows()


if __name__=='__main__':
    receive_message()