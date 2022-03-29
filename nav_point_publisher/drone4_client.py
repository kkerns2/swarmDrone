#!/usr/bin/env python

from dataclasses import replace
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_drone4_client(x, y):
    client4 = actionlib.SimpleActionClient('/drone4/move_base', MoveBaseAction)
    
    client4.wait_for_server()
    
    #client4.cancel_goal()
   
    goal4 = MoveBaseGoal()
   
    goal4.target_pose.header.frame_id = "map"
   
    goal4.target_pose.header.stamp = rospy.Time.now()

    goal4.target_pose.pose.position.x = x
    
    goal4.target_pose.pose.position.y = y
   
    goal4.target_pose.pose.orientation.w = 1.0
    
    client4.send_goal(goal4)

    wait = client4.wait_for_result()

    if not wait:
        rospy.logerr("Action server is not available")
        rospy.signal_shutdown("Action server is not available")
    else:
        return client4.get_result()

if __name__=='__main__':

    with open('/home/mcp/catkin_ws/src/nav_point_publisher/waypoints_d4.txt', 'r') as f:
        lines_4 = f.readlines()

    drone4_coordinates = []

    for i in range(len(lines_4)):
        lines_4[i] = lines_4[i].replace('\n', '')
        drone4_coordinates.append(lines_4[i].split(" "))
        for j in range(2): 
            drone4_coordinates[i][j] = float(drone4_coordinates[i][j])

    try:
        #add coordinate logic here
        rospy.init_node('drone4_client_py')
        
        drone4_count = 0
        
        while drone4_count < len(drone4_coordinates):
    
            drone4_result = movebase_drone4_client(drone4_coordinates[drone4_count][0], drone4_coordinates[drone4_count][1])
    
            if drone4_result:
                rospy.loginfo("Drone 4: Goal executed")
                drone4_count +=1
            else:
                rospy.loginfo("Drone 4: Something went wrong")
                break

    except rospy.ROSInterruptException:
        rospy.loginfo("Swarm Drone Navigation Broadcast Finished")

