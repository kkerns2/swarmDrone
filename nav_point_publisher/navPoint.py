#!/usr/bin/env python

from dataclasses import replace
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client(x, y):
    #could make multiple clients to broadcast to multiple drones
    client = actionlib.SimpleActionClient('/drone1/move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0


    client.send_goal(goal)
    rospy.spin()
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server is not available")
        rospy.signal_shutdown("Action server is not available")
    else:
        return client.get_result()

if __name__=='__main__':
    #import the text files (x,y) coordinates.
    with open('/home/mcp/catkin_ws/src/nav_point_publisher/waypoints.txt', 'r') as f:
        lines = f.readlines()

    #convert the strings to floats
    coordinates = []
    for i in range(len(lines)):
        lines[i] = lines[i].replace('\n', '')
        coordinates.append(lines[i].split(" "))
        for j in range(2): 
            coordinates[i][j] = float(coordinates[i][j])

    try:
        #add coordinate logic here
        rospy.init_node('navPoint_py')
        count = 0
        while count < coordinates:
            result = movebase_client(coordinates[count][0], coordinates[count][1])
            if result:
                rospy.loginfo("Goal executed")
                count +=1
            else:
                rospy.loginfo("Something went wrong")
                break

    except rospy.ROSInterruptException:
        rospy.loginfo("Nav Broadcast finished")

    #pass the coordinates to the drones. 