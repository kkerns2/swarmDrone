#! /usr/bin/env python
from turtle import position
import rospy
import std_msgs
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseWithCovarianceStamped
from p5 import Vector
from boid import Boid
import sys


x = 0
y = 0
z = 0
positions = list()

def retrieve_positions(msg):

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z

    positions.append(x)
    positions.append(y)
    positions.append(z)



def boid_startup(num_drones):

    rospy.init_node('boid_main')

    # Store all subscribers into the dictionary for easy reference
    drone_subscribers = {}
    j = 0
    for i in range(num_drones):
        j = i+1
        drone_subscribers[j] = rospy.Subscriber(f'/drone{j}/amcl_pose', PoseWithCovarianceStamped, retrieve_positions)

    drones = {}
    # Declare a single boid instance for each drone
    for i in range(num_drones):
        j = i + 1
        drones[j] = Boid(positions[0], positions[1])

    print(drones)


if __name__=='__main__':

    # Standard swarm size is 4
    num_drones = int(sys.argv[1])

    boid_startup(num_drones)

    rospy.spin()