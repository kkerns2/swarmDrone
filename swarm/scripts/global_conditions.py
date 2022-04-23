#!/usr/bin/env python
import rospy

import global_drones

import os
import random
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseWithCovarianceStamped

drone_x = 0 
drone_y = 0
drone_z = 0

drone_orien_x = 0
drone_orien_y = 0
drone_orien_z = 0
drone_orien_w = 0 

drone_flag = False

g_num_drones = global_drones.num_drones()
leader = [False] * (g_num_drones + 1) # over allocate for lazy indexing

firstTime = 0 

rand_list = [-6, -5, -4 ,-3, -2, 2, 3, 4, 5, 6]

buffer1 =random.choice(rand_list)

rand_list.remove(buffer1)

buffer2 = random.choice(rand_list)

rand_list.remove(buffer2)

buffer3 = random.choice(rand_list)

rand_list.remove(buffer3)

buffer4 = random.choice(rand_list)

rand_list.remove(buffer4)



def cancel_goals():
    for i in range(1, g_num_drones + 1):
        # cancel n drones
        cancel_pub = rospy.Publisher(f"/drone{i}/move_base/cancel", GoalID, queue_size=1)
        cancel_msg = GoalID()
        cancel_pub.publish(cancel_msg)

        # double check drones goals are canceled
        client = actionlib.SimpleActionClient(f'/drone{i}/move_base', MoveBaseAction)
        client.cancel_goal()


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

def stop_condition(drone_name):
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
    global leader
    global firstTime  

    drone_number = int(drone_name.replace('drone', ''))


    # everything here is modularized. comment this block for other testing
    if((firstTime == 0) or (leader[drone_number] == True)):
        print(f'Drone {drone_number} spotted BB8!')
        print(f'Leader is Drone {drone_number}')

        firstTime = 1
        leader[drone_number] = True

        if(firstTime):
            os.system('rosnode kill /drone1_client_py /drone2_client_py /drone3_client_py /drone4_client_py')
        # Set all 4 drones to cancel their goals: 
        cancel_goals()

        rospy.Subscriber(f'/{drone_name}/amcl_pose', PoseWithCovarianceStamped, retrieve_positions)

        for i in range(1, g_num_drones + 1):
            if i == drone_number:
                continue

            # Get the action state for goals
            action = actionlib.SimpleActionClient(f'/drone{i}/move_base', MoveBaseAction)
            goal = MoveBaseGoal()

            goal.target_pose.header.frame_id = 'map' 
            goal.target_pose.pose.position.x = drone_x + (2.0 * i) #param here
            goal.target_pose.pose.position.y = drone_y  
            goal.target_pose.pose.position.z = drone_z 
            goal.target_pose.pose.orientation.x = drone_orien_x
            goal.target_pose.pose.orientation.y = drone_orien_y
            goal.target_pose.pose.orientation.z = drone_orien_z
            goal.target_pose.pose.orientation.w = drone_orien_w
            if (i > g_num_drones / 2):
                goal.target_pose.pose.orientation.w = drone_orien_w + 180

            action.send_goal(goal)









    # if((drone_name == 'drone1' and firstTime == 0) or (leader[1] == True)):
    #     print('Drone 1 Spotted BB8!')
    #     print('ld1', leader[1])
    #     firstTime = 1
    #     leader[1] = True

    #     if(firstTime):
    #         os.system('rosnode kill /drone1_client_py /drone2_client_py /drone3_client_py /drone4_client_py')
    #     # Set all 4 drones to cancel their goals: 
    #     cancel_goals()
        

    #     # Receive drone 1 position
    #     pose_sub_1 = rospy.Subscriber('/drone1/amcl_pose', PoseWithCovarianceStamped, retrieve_positions)

    #     # Get the action state for goals
    #     d2_action = actionlib.SimpleActionClient('/drone2/move_base', MoveBaseAction)
    #     d3_action = actionlib.SimpleActionClient('/drone3/move_base', MoveBaseAction)
    #     d4_action = actionlib.SimpleActionClient('/drone4/move_base', MoveBaseAction)

    #     goal_2 = MoveBaseGoal()
    #     goal_3 = MoveBaseGoal()
    #     goal_4 = MoveBaseGoal()

    #     goal_2.target_pose.header.frame_id = 'map' 
    #     goal_2.target_pose.pose.position.x = drone_x + 2.0 
    #     goal_2.target_pose.pose.position.y = drone_y  
    #     goal_2.target_pose.pose.position.z = drone_z 
    #     goal_2.target_pose.pose.orientation.x = drone_orien_x
    #     goal_2.target_pose.pose.orientation.y = drone_orien_y
    #     goal_2.target_pose.pose.orientation.z = drone_orien_z
    #     goal_2.target_pose.pose.orientation.w = drone_orien_w

    #     goal_3.target_pose.header.frame_id = 'map' 
    #     goal_3.target_pose.pose.position.x = drone_x + 4.0 
    #     goal_3.target_pose.pose.position.y = drone_y 
    #     goal_3.target_pose.pose.position.z = drone_z 
    #     goal_3.target_pose.pose.orientation.x = drone_orien_x
    #     goal_3.target_pose.pose.orientation.y = drone_orien_y
    #     goal_3.target_pose.pose.orientation.z = drone_orien_z 
    #     goal_3.target_pose.pose.orientation.w = drone_orien_w + 180

    #     goal_4.target_pose.header.frame_id = 'map' 
    #     goal_4.target_pose.pose.position.x = drone_x + 6.0
    #     goal_4.target_pose.pose.position.y = drone_y 
    #     goal_4.target_pose.pose.position.z = drone_z 
    #     goal_4.target_pose.pose.orientation.x = drone_orien_x
    #     goal_4.target_pose.pose.orientation.y = drone_orien_y
    #     goal_4.target_pose.pose.orientation.z = drone_orien_z 
    #     goal_4.target_pose.pose.orientation.w = drone_orien_w + 180

    #     d2_action.send_goal(goal_2) 
    #     d3_action.send_goal(goal_3) 
    #     d4_action.send_goal(goal_4) 
        

    # elif((drone_name == 'drone2' and firstTime == 0) or (leader[2] == True)):
    #     print('Drone 2 Spotted BB8!')
    #     print('ld2', leader[2])

    #     firstTime = 1
    #     leader[2] = True

    #     if(firstTime):
    #         os.system('rosnode kill /drone1_client_py /drone2_client_py /drone3_client_py /drone4_client_py')

    #     # Set all 4 drones to cancel their goals: 
    #     cancel_goals()
        

    #     # Receive drone 2 position
    #     pose_sub_2 = rospy.Subscriber('/drone2/amcl_pose', PoseWithCovarianceStamped, retrieve_positions)

    #     # Get the action state for goals
    #     d1_action = actionlib.SimpleActionClient('/drone1/move_base', MoveBaseAction)
    #     d3_action = actionlib.SimpleActionClient('/drone3/move_base', MoveBaseAction)
    #     d4_action = actionlib.SimpleActionClient('/drone4/move_base', MoveBaseAction)

    #     goal_1 = MoveBaseGoal()
    #     goal_3 = MoveBaseGoal()
    #     goal_4 = MoveBaseGoal()

    #     goal_1.target_pose.header.frame_id = 'map' 
    #     goal_1.target_pose.pose.position.x = drone_x + 2.0
    #     goal_1.target_pose.pose.position.y = drone_y 
    #     goal_1.target_pose.pose.position.z = drone_z 
    #     goal_1.target_pose.pose.orientation.x = drone_orien_x
    #     goal_1.target_pose.pose.orientation.y = drone_orien_y
    #     goal_1.target_pose.pose.orientation.z = drone_orien_z 
    #     goal_1.target_pose.pose.orientation.w = drone_orien_w

    #     goal_3.target_pose.header.frame_id = 'map' 
    #     goal_3.target_pose.pose.position.x = drone_x + 4.0
    #     goal_3.target_pose.pose.position.y = drone_y 
    #     goal_3.target_pose.pose.position.z = drone_z 
    #     goal_3.target_pose.pose.orientation.x = drone_orien_x
    #     goal_3.target_pose.pose.orientation.y = drone_orien_y
    #     goal_3.target_pose.pose.orientation.z = drone_orien_z 
    #     goal_3.target_pose.pose.orientation.w = drone_orien_w + 180

    #     goal_4.target_pose.header.frame_id = 'map' 
    #     goal_4.target_pose.pose.position.x = drone_x + 6.0
    #     goal_4.target_pose.pose.position.y = drone_y 
    #     goal_4.target_pose.pose.position.z = drone_z 
    #     goal_4.target_pose.pose.orientation.x = drone_orien_x
    #     goal_4.target_pose.pose.orientation.y = drone_orien_y
    #     goal_4.target_pose.pose.orientation.z = drone_orien_z 
    #     goal_4.target_pose.pose.orientation.w = drone_orien_w + 180

    #     d1_action.send_goal(goal_1) 
    #     d3_action.send_goal(goal_3) 
    #     d4_action.send_goal(goal_4) 
    
    
    # elif((drone_name == 'drone3' and firstTime == 0) or (leader[3] == True)):
    #     print('Drone 3 Spotted BB8!')
    #     print('ld1', leader[3])

    #     firstTime = 1
    #     leader[3] = True
    #     # Set all 4 drones to cancel their goals: 
        
    #     if(firstTime):
    #         os.system('rosnode kill /drone1_client_py /drone2_client_py /drone3_client_py /drone4_client_py')
        
    #     cancel_goals()
       

    #     # Receive drone 3 position
    #     pose_sub_3 = rospy.Subscriber('/drone3/amcl_pose', PoseWithCovarianceStamped, retrieve_positions)

    #     # Get the action state for goals
    #     d1_action = actionlib.SimpleActionClient('/drone1/move_base', MoveBaseAction)
    #     d2_action = actionlib.SimpleActionClient('/drone2/move_base', MoveBaseAction)
    #     d4_action = actionlib.SimpleActionClient('/drone4/move_base', MoveBaseAction)

    #     goal_1 = MoveBaseGoal()
    #     goal_2 = MoveBaseGoal()
    #     goal_4 = MoveBaseGoal()
        
    #         # goal header set to map 
    #     goal_1.target_pose.header.frame_id = 'map'
    #     goal_1.target_pose.pose.position.x = drone_x + 2.0
    #     goal_1.target_pose.pose.position.y = drone_y 
    #     goal_1.target_pose.pose.position.z = drone_z 
    #     goal_1.target_pose.pose.orientation.x = drone_orien_x
    #     goal_1.target_pose.pose.orientation.y = drone_orien_y
    #     goal_1.target_pose.pose.orientation.z = drone_orien_z 
    #     goal_1.target_pose.pose.orientation.w = drone_orien_w

    #     goal_2.target_pose.header.frame_id = 'map' 
    #     goal_2.target_pose.pose.position.x = drone_x + 4.0
    #     goal_2.target_pose.pose.position.y = drone_y 
    #     goal_2.target_pose.pose.position.z = drone_z 
    #     goal_2.target_pose.pose.orientation.x = drone_orien_x
    #     goal_2.target_pose.pose.orientation.y = drone_orien_y
    #     goal_2.target_pose.pose.orientation.z = drone_orien_z 
    #     goal_2.target_pose.pose.orientation.w = drone_orien_w + 180

    #     goal_4.target_pose.header.frame_id = 'map' 
    #     goal_4.target_pose.pose.position.x = drone_x + 6.0
    #     goal_4.target_pose.pose.position.y = drone_y 
    #     goal_4.target_pose.pose.position.z = drone_z 
    #     goal_4.target_pose.pose.orientation.x = drone_orien_x
    #     goal_4.target_pose.pose.orientation.y = drone_orien_y
    #     goal_4.target_pose.pose.orientation.z = drone_orien_z
    #     goal_4.target_pose.pose.orientation.w = drone_orien_w + 180

    #     d1_action.send_goal(goal_1) 
    #     d2_action.send_goal(goal_2) 
    #     d4_action.send_goal(goal_4) 
        

    # elif((drone_name == 'drone4' and firstTime == 0) or (leader[4] == True)):
    #     print('Drone 4 Spooted BB8!')
    #     print('ld4', leader[4])

    #     if(firstTime):
    #         os.system('rosnode kill /drone1_client_py /drone2_client_py /drone3_client_py /drone4_client_py')

    #     # Flag for true                  
    #     firstTime = 1
    #     leader[4] = True

    #     # Set all 4 drones to cancel their goals: 
    #     cancel_goals()


    #     # Receive drone 3 position
    #     pose_sub_4 = rospy.Subscriber('/drone4/amcl_pose', PoseWithCovarianceStamped, retrieve_positions)

    #     # Get the action state for goals
    #     d1_action = actionlib.SimpleActionClient('/drone1/move_base', MoveBaseAction)
    #     d2_action = actionlib.SimpleActionClient('/drone2/move_base', MoveBaseAction)
    #     d3_action = actionlib.SimpleActionClient('/drone3/move_base', MoveBaseAction)

    #     goal_1 = MoveBaseGoal()
    #     goal_2 = MoveBaseGoal()
    #     goal_3 = MoveBaseGoal()
        
    #     goal_1.target_pose.header.frame_id = 'map' 
    #     goal_1.target_pose.pose.position.x = drone_x + 2.0
    #     goal_1.target_pose.pose.position.y = drone_y 
    #     goal_1.target_pose.pose.position.z = drone_z 
    #     goal_1.target_pose.pose.orientation.x = drone_orien_x
    #     goal_1.target_pose.pose.orientation.y = drone_orien_y
    #     goal_1.target_pose.pose.orientation.z = drone_orien_z 
    #     goal_1.target_pose.pose.orientation.w = drone_orien_w

    #     goal_2.target_pose.header.frame_id = 'map' 
    #     goal_2.target_pose.pose.position.x = drone_x + 4.0
    #     goal_2.target_pose.pose.position.y = drone_y 
    #     goal_2.target_pose.pose.position.z = drone_z 
    #     goal_2.target_pose.pose.orientation.x = drone_orien_x
    #     goal_2.target_pose.pose.orientation.y = drone_orien_y
    #     goal_2.target_pose.pose.orientation.z = drone_orien_z 
    #     goal_2.target_pose.pose.orientation.w = drone_orien_w + 180

    #     goal_3.target_pose.header.frame_id = 'map' 
    #     goal_3.target_pose.pose.position.x = drone_x + 6.0
    #     goal_3.target_pose.pose.position.y = drone_y 
    #     goal_3.target_pose.pose.position.z = drone_z 
    #     goal_3.target_pose.pose.orientation.x = drone_orien_x
    #     goal_3.target_pose.pose.orientation.y = drone_orien_y
    #     goal_3.target_pose.pose.orientation.z = drone_orien_z 
    #     goal_3.target_pose.pose.orientation.w = drone_orien_w + 180

    #     d1_action.send_goal(goal_1) 
    #     d2_action.send_goal(goal_2) 
    #     d3_action.send_goal(goal_3)





# if __name__=='__main__':
    # print(num_drones())
    