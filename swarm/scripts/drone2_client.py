#!/usr/bin/env python

from dataclasses import replace
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_drone2_client(x, y):
    ''' 
    Grabs Drone Movebase and assign the reference to Map, 
    and feed the generated waypoints to navigate to. 
    '''

    client2 = actionlib.SimpleActionClient('/drone2/move_base', MoveBaseAction)
    
    client2.wait_for_server()
   
    goal2 = MoveBaseGoal()
   
    goal2.target_pose.header.frame_id = "map"
   
    goal2.target_pose.header.stamp = rospy.Time.now()

    goal2.target_pose.pose.position.x = x
    
    goal2.target_pose.pose.position.y = y
   
    goal2.target_pose.pose.orientation.w = 1.0
    
    client2.send_goal(goal2)

    wait = client2.wait_for_result()

    if not wait:
        rospy.logerr("Action server is not available")
        rospy.signal_shutdown("Action server is not available")
    else:
        return client2.get_result()

if __name__=='__main__':
    # If Launching on different environment, change workspace name and user name 
    with open('/home/mcp/catkin_ws/src/swarm/waypoints/waypoints_d2.txt', 'r') as f:
        lines_2 = f.readlines()

    drone2_coordinates = []

    for i in range(len(lines_2)):
        lines_2[i] = lines_2[i].replace('\n', '')
        drone2_coordinates.append(lines_2[i].split(" "))
        for j in range(2): 
            drone2_coordinates[i][j] = float(drone2_coordinates[i][j])

    try:
        #add coordinate logic here
        rospy.init_node('drone2_client_py')
        
        drone2_count = 0
        
        while drone2_count < len(drone2_coordinates):
    
            drone2_result = movebase_drone2_client(drone2_coordinates[drone2_count][0], drone2_coordinates[drone2_count][1])
    
            if drone2_result:
                rospy.loginfo("Drone 2: Goal executed")
                drone2_count +=1
            else:
                rospy.loginfo("Drone 2: Something went wrong")
                break

    except rospy.ROSInterruptException:
        rospy.loginfo("Swarm Drone Navigation Broadcast Finished")

