#!/usr/bin/env python

from dataclasses import replace
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_drone3_client(x, y):
    client3 = actionlib.SimpleActionClient('/drone3/move_base', MoveBaseAction)
    
    client3.wait_for_server()
    
    #client3.cancel_goal()
   
    goal3 = MoveBaseGoal()
   
    goal3.target_pose.header.frame_id = "map"
   
    goal3.target_pose.header.stamp = rospy.Time.now()

    goal3.target_pose.pose.position.x = x
    
    goal3.target_pose.pose.position.y = y
   
    goal3.target_pose.pose.orientation.w = 1.0
    
    client3.send_goal(goal3)

    wait = client3.wait_for_result()

    if not wait:
        rospy.logerr("Action server is not available")
        rospy.signal_shutdown("Action server is not available")
    else:
        return client3.get_result()

if __name__=='__main__':

    with open('/home/mcp/catkin_ws/src/swarm/waypoints/waypoints_d3.txt', 'r') as f:
        lines_3 = f.readlines()

    drone3_coordinates = []

    for i in range(len(lines_3)):
        lines_3[i] = lines_3[i].replace('\n', '')
        drone3_coordinates.append(lines_3[i].split(" "))
        for j in range(2): 
            drone3_coordinates[i][j] = float(drone3_coordinates[i][j])

    try:
        #add coordinate logic here
        rospy.init_node('drone3_client_py')
        
        drone3_count = 0
        
        while drone3_count < len(drone3_coordinates):
    
            drone3_result = movebase_drone3_client(drone3_coordinates[drone3_count][0], drone3_coordinates[drone3_count][1])
    
            if drone3_result:
                rospy.loginfo("Drone 3: Goal executed")
                drone3_count +=1
            else:
                rospy.loginfo("Drone 3: Something went wrong")
                break

    except rospy.ROSInterruptException:
        rospy.loginfo("Swarm Drone Navigation Broadcast Finished")

