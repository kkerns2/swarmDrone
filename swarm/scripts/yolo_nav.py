#!/usr/bin/env python
import cv2, cv_bridge
import os
import rospy
import sys
import torch
import math 
import threading
from sensor_msgs.msg import Image
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist   
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

lock_yolo = threading.Lock()
lock_frames = threading.Lock()


num_Drones = len(sys.argv) - 1
drone_names = []
if num_Drones >= 1:
    frame_buffer = [None] * num_Drones
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
model_path = os.path.join(dirname, 'YOLOv5.pt')
model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)
device = 'cuda' if torch.cuda.is_available() else 'cpu'
model.to(device)

drone_x = 0 
drone_y = 0
drone_z = 0

kP = 0.5 

drone_orien_x = 0
drone_orien_y = 0
drone_orien_z = 0
drone_orien_w = 0 

drone_flag = False

leader_d1 = False
leader_d2 = False
leader_d3 = False 
leader_d4 = False

mid = False

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

def getD1Height(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    print("getD1Height: ", x, y, z)

def getD2Height(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    print("getD2Height: ", x, y, z)

def getD4Height(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    print("getD4Height: ", x, y, z)
   

def calculate_midpoint(x1, x2):
    return (x1 + x2) / 2

def stop_condition(drone_name, cords):
    global drone_x
    global drone_y
    global drone_z 
    
    global drone_orien_x 
    global drone_orien_y
    global drone_orien_z
    global drone_orien_w
    
    global buffer1
    global buffer2
    global buffer3
    global buffer4
    
    global drone_flag
    
    global leader_d1 
    global leader_d2 
    global leader_d3  
    global leader_d4 
    global firstTime  

    global mid 

    if((drone_name == 'drone1' and firstTime == 0) or (leader_d1 == True)):
        print('Drone 1 Spotted BB8!')
        print('ld1', leader_d1)
        firstTime = 1
        leader_d1 = True

        if(firstTime):
            os.system('rosnode kill /drone1_client_py /drone2_client_py /drone3_client_py /drone4_client_py')
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

        # Receive drone 1 position
        pose_sub_1 = rospy.Subscriber('/drone1/amcl_pose', PoseWithCovarianceStamped, retrieve_positions)
        velocity_publisher = rospy.Publisher('/drone1/cmd_vel', Twist, queue_size=1)

        vel_msg = Twist()

        x1 = cords[0]
        x2 = cords[2]

        midpoint = calculate_midpoint(x1, x2)
        
        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        
        speed = 5
        
        angular_speed = speed*2*math.pi/360
        
        if(mid != True):
            if(midpoint > 500):
                print('rotate right', midpoint)
                vel_msg.angular.z = -abs(angular_speed)
                velocity_publisher.publish(vel_msg)
            elif(midpoint < 260):
                print('rotate left', midpoint)
                vel_msg.angular.z = abs(angular_speed)
                velocity_publisher.publish(vel_msg)
            else:
                print('stop rotating', midpoint)
                vel_msg.angular.z = 0
                mid = True 
                velocity_publisher.publish(vel_msg)
        # Get the action state for goals
        d2_action = actionlib.SimpleActionClient('/drone2/move_base', MoveBaseAction)
        d3_action = actionlib.SimpleActionClient('/drone3/move_base', MoveBaseAction)
        d4_action = actionlib.SimpleActionClient('/drone4/move_base', MoveBaseAction)

        goal_2 = MoveBaseGoal()
        goal_3 = MoveBaseGoal()
        goal_4 = MoveBaseGoal()

        goal_2.target_pose.header.frame_id = 'map' 
        goal_2.target_pose.pose.position.x = drone_x  
        goal_2.target_pose.pose.position.y = drone_y  
        goal_2.target_pose.pose.position.z = drone_z 
        goal_2.target_pose.pose.orientation.x = drone_orien_x
        goal_2.target_pose.pose.orientation.y = drone_orien_y
        goal_2.target_pose.pose.orientation.z = drone_orien_z
        goal_2.target_pose.pose.orientation.w = drone_orien_w

        goal_3.target_pose.header.frame_id = 'map' 
        goal_3.target_pose.pose.position.x = drone_x  
        goal_3.target_pose.pose.position.y = drone_y 
        goal_3.target_pose.pose.position.z = drone_z 
        goal_3.target_pose.pose.orientation.x = drone_orien_x
        goal_3.target_pose.pose.orientation.y = drone_orien_y
        goal_3.target_pose.pose.orientation.z = drone_orien_z 
        goal_3.target_pose.pose.orientation.w = drone_orien_w 

        goal_4.target_pose.header.frame_id = 'map' 
        goal_4.target_pose.pose.position.x = drone_x 
        goal_4.target_pose.pose.position.y = drone_y 
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

        if(firstTime):
            os.system('rosnode kill /drone1_client_py /drone2_client_py /drone3_client_py /drone4_client_py')

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
        rospy.Subscriber('/drone2/amcl_pose', PoseWithCovarianceStamped, retrieve_positions)
        velocity_publisher = rospy.Publisher('/drone2/cmd_vel', Twist, queue_size=1)

        vel_msg = Twist()

        x1 = cords[0]
        x2 = cords[2]

        midpoint = calculate_midpoint(x1, x2)
        
        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        
        speed = 5
        
        angular_speed = speed*2*math.pi/360
        
        if(mid != True):
            if(midpoint > 500):
                print('rotate right', midpoint)
                vel_msg.angular.z = -abs(angular_speed)
                velocity_publisher.publish(vel_msg)
            elif(midpoint < 260):
                print('rotate left', midpoint)
                vel_msg.angular.z = abs(angular_speed)
                velocity_publisher.publish(vel_msg)
            else:
                print('stop rotating', midpoint)
                vel_msg.angular.z = 0
                mid = True 
                velocity_publisher.publish(vel_msg)

        # Get the action state for goals
        d1_action = actionlib.SimpleActionClient('/drone1/move_base', MoveBaseAction)
        d3_action = actionlib.SimpleActionClient('/drone3/move_base', MoveBaseAction)
        d4_action = actionlib.SimpleActionClient('/drone4/move_base', MoveBaseAction)

        goal_1 = MoveBaseGoal()
        goal_3 = MoveBaseGoal()
        goal_4 = MoveBaseGoal()

        goal_1.target_pose.header.frame_id = 'map' 
        goal_1.target_pose.pose.position.x = drone_x 
        goal_1.target_pose.pose.position.y = drone_y 
        goal_1.target_pose.pose.position.z = drone_z 
        goal_1.target_pose.pose.orientation.x = drone_orien_x
        goal_1.target_pose.pose.orientation.y = drone_orien_y
        goal_1.target_pose.pose.orientation.z = drone_orien_z 
        goal_1.target_pose.pose.orientation.w = drone_orien_w

        goal_3.target_pose.header.frame_id = 'map' 
        goal_3.target_pose.pose.position.x = drone_x 
        goal_3.target_pose.pose.position.y = drone_y 
        goal_3.target_pose.pose.position.z = drone_z 
        goal_3.target_pose.pose.orientation.x = drone_orien_x
        goal_3.target_pose.pose.orientation.y = drone_orien_y
        goal_3.target_pose.pose.orientation.z = drone_orien_z 
        goal_3.target_pose.pose.orientation.w = drone_orien_w 

        goal_4.target_pose.header.frame_id = 'map' 
        goal_4.target_pose.pose.position.x = drone_x 
        goal_4.target_pose.pose.position.y = drone_y 
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
        
        if(firstTime):
            os.system('rosnode kill /drone1_client_py /drone2_client_py /drone3_client_py /drone4_client_py')
        
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
        rospy.Subscriber('/drone3/amcl_pose', PoseWithCovarianceStamped, retrieve_positions)
        velocity_publisher = rospy.Publisher('/drone3/cmd_vel', Twist, queue_size=1)

        vel_msg = Twist()

        x1 = cords[0]
        x2 = cords[2]

        midpoint = calculate_midpoint(x1, x2)
        
        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        
        speed = 5
        
        angular_speed = speed*2*math.pi/360
        
        if(mid != True):
            if(midpoint > 500):
                print('rotate right', midpoint)
                vel_msg.angular.z = -abs(angular_speed)
                velocity_publisher.publish(vel_msg)
            elif(midpoint < 260):
                print('rotate left', midpoint)
                vel_msg.angular.z = abs(angular_speed)
                velocity_publisher.publish(vel_msg)
            else:
                print('stop rotating', midpoint)
                vel_msg.angular.z = 0
                mid = True 
                velocity_publisher.publish(vel_msg)

        # Get the action state for goals
        d1_action = actionlib.SimpleActionClient('/drone1/move_base', MoveBaseAction)
        d2_action = actionlib.SimpleActionClient('/drone2/move_base', MoveBaseAction)
        d4_action = actionlib.SimpleActionClient('/drone4/move_base', MoveBaseAction)

        goal_1 = MoveBaseGoal()
        goal_2 = MoveBaseGoal()
        goal_4 = MoveBaseGoal()
        
        # goal header set to map 
        goal_1.target_pose.header.frame_id = 'map'
        goal_1.target_pose.pose.position.x = drone_x 
        goal_1.target_pose.pose.position.y = drone_y 
        goal_1.target_pose.pose.position.z = drone_z

        goal_1.target_pose.pose.orientation.x = drone_orien_x
        goal_1.target_pose.pose.orientation.y = drone_orien_y
        goal_1.target_pose.pose.orientation.z = drone_orien_z 
        goal_1.target_pose.pose.orientation.w = drone_orien_w

        goal_2.target_pose.header.frame_id = 'map' 
        goal_2.target_pose.pose.position.x = drone_x 
        goal_2.target_pose.pose.position.y = drone_y 
        goal_2.target_pose.pose.position.z = drone_z

        goal_2.target_pose.pose.orientation.x = drone_orien_x
        goal_2.target_pose.pose.orientation.y = drone_orien_y
        goal_2.target_pose.pose.orientation.z = drone_orien_z 
        goal_2.target_pose.pose.orientation.w = drone_orien_w 

        goal_4.target_pose.header.frame_id = 'map' 
        goal_4.target_pose.pose.position.x = drone_x 
        goal_4.target_pose.pose.position.y = drone_y 
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

        if(firstTime):
            os.system('rosnode kill /drone1_client_py /drone2_client_py /drone3_client_py /drone4_client_py')

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

        # print(cords)
        
        # Receive drone 3 position
        rospy.Subscriber('/drone4/amcl_pose', PoseWithCovarianceStamped, retrieve_positions)
        
        velocity_publisher = rospy.Publisher('/drone4/cmd_vel', Twist, queue_size=1)
        velocity_publisher = rospy.Publisher('/drone4/cmd_vel', Twist, queue_size=1)

        vel_msg = Twist()

        x1 = cords[0]
        x2 = cords[2]

        midpoint = calculate_midpoint(x1, x2)
        
        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        
        speed = 5
        
        angular_speed = speed*2*math.pi/360
        
        if(mid != True):
            if(midpoint > 500):
                print('rotate right', midpoint)
                vel_msg.angular.z = -abs(angular_speed)
                velocity_publisher.publish(vel_msg)
            elif(midpoint < 260):
                print('rotate left', midpoint)
                vel_msg.angular.z = abs(angular_speed)
                velocity_publisher.publish(vel_msg)
            else:
                print('stop rotating', midpoint)
                vel_msg.angular.z = 0
                mid = True 
                velocity_publisher.publish(vel_msg)

             
        

        # Get the action state for goals
        d1_action = actionlib.SimpleActionClient('/drone1/move_base', MoveBaseAction)
        d2_action = actionlib.SimpleActionClient('/drone2/move_base', MoveBaseAction)
        d3_action = actionlib.SimpleActionClient('/drone3/move_base', MoveBaseAction)

        goal_1 = MoveBaseGoal()
        goal_2 = MoveBaseGoal()
        goal_3 = MoveBaseGoal()
        
        goal_1.target_pose.header.frame_id = 'map' 
        goal_1.target_pose.pose.position.x = drone_x 
        goal_1.target_pose.pose.position.y = drone_y 
        goal_1.target_pose.pose.position.z = drone_z 
        goal_1.target_pose.pose.orientation.x = drone_orien_x
        goal_1.target_pose.pose.orientation.y = drone_orien_y
        goal_1.target_pose.pose.orientation.z = drone_orien_z 
        goal_1.target_pose.pose.orientation.w = drone_orien_w

        goal_2.target_pose.header.frame_id = 'map' 
        goal_2.target_pose.pose.position.x = drone_x 
        goal_2.target_pose.pose.position.y = drone_y 
        goal_2.target_pose.pose.position.z = drone_z 
        goal_2.target_pose.pose.orientation.x = drone_orien_x
        goal_2.target_pose.pose.orientation.y = drone_orien_y
        goal_2.target_pose.pose.orientation.z = drone_orien_z 
        goal_2.target_pose.pose.orientation.w = drone_orien_w 

        goal_3.target_pose.header.frame_id = 'map' 
        goal_3.target_pose.pose.position.x = drone_x 
        goal_3.target_pose.pose.position.y = drone_y 
        goal_3.target_pose.pose.position.z = drone_z 
        goal_3.target_pose.pose.orientation.x = drone_orien_x
        goal_3.target_pose.pose.orientation.y = drone_orien_y
        goal_3.target_pose.pose.orientation.z = drone_orien_z 
        goal_3.target_pose.pose.orientation.w = drone_orien_w 

        d1_action.send_goal(goal_1) 
        d2_action.send_goal(goal_2) 
        d3_action.send_goal(goal_3)

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

    return frame


def callback(msg, args):
    i = args[0] # msg - image args = drone count
    drone_name = args[1]     # current drone

    br = cv_bridge.CvBridge()
    current_frame = br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    # convert bgr -> rgb for image detector to process
    frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)

    results = inference(frame, model)
    # print(results)
    frame = plot_boxes(results, frame)

    # convert back rgb -> to brg for cv2 to display
    lock_frames.acquire()
    frame_buffer[i] = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    lock_frames.release()

    cords, labels = results
    for i, label in enumerate(labels):
        row = cords[i]
        if row[4] > 0.96:
            stop_condition(drone_name, row)



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
        rospy.Subscriber(topic, Image, callback, (i, drone_name))


    t = threading.Thread(target=thread_imshow)
    t.start()

    rospy.spin()
    lock_frames.acquire()
    frame_buffer[0] = None
    lock_frames.release()
    cv2.destroyAllWindows()


if __name__=='__main__':
    receive_message()
