#!/usr/bin/env python

from dataclasses import replace
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_drone1_client(x, y):
    client1 = actionlib.SimpleActionClient('/drone1/move_base', MoveBaseAction)
    
    client1.wait_for_server()
    
    #client1.cancel_goal()
   
    goal1 = MoveBaseGoal()
   
    goal1.target_pose.header.frame_id = "map"
   
    goal1.target_pose.header.stamp = rospy.Time.now()

    goal1.target_pose.pose.position.x = x
    
    goal1.target_pose.pose.position.y = y
   
    goal1.target_pose.pose.orientation.w = 1.0
    
    client1.send_goal(goal1)

    wait = client1.wait_for_result()

    if not wait:
        rospy.logerr("Action server is not available")
        rospy.signal_shutdown("Action server is not available")
    else:
        return client1.get_result()

if __name__=='__main__':

    with open('/home/mcp/catkin_ws/src/nav_point_publisher/waypoints_d1.txt', 'r') as f:
        lines_1 = f.readlines()

    drone1_coordinates = []

    for i in range(len(lines_1)):
        lines_1[i] = lines_1[i].replace('\n', '')
        drone1_coordinates.append(lines_1[i].split(" "))
        for j in range(2): 
            drone1_coordinates[i][j] = float(drone1_coordinates[i][j])

    try:
        #add coordinate logic here
        rospy.init_node('drone1_client_py')
        
        drone1_count = 0
        
        while drone1_count < len(drone1_coordinates):
    
            drone1_result = movebase_drone1_client(drone1_coordinates[drone1_count][0], drone1_coordinates[drone1_count][1])
    
            if drone1_result:
                rospy.loginfo("Drone 1: Goal executed")
                drone1_count +=1
            else:
                rospy.loginfo("Drone 1: Something went wrong")
                break

    except rospy.ROSInterruptException:
        rospy.loginfo("Swarm Drone Navigation Broadcast Finished")

