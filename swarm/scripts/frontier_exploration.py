#!/usr/bin/env python

from cProfile import label
import rospy
import actionlib
from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import ColorRGBA
import time
import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt
import matplotlib.markers as markers
from skimage.measure import find_contours
from sklearn.cluster import AgglomerativeClustering
import random
import threading
import sys
import signal

map_data = OccupancyGrid()
odom = Odometry
bot_x = 0
bot_y = 0
resolution = 0
ori_x = 0
ori_y = 0
target = np.array([0, 0])
prev_target = np.array([0, 0])
rviz_id = 0
bot_idx = np.zeros([1,2])

# read information from the map
def map_callback(data):
    global map_data
    global resolution, ori_x, ori_y, w, h
    map_data = data
    resolution = map_data.info.resolution
    ori_x = map_data.info.origin.position.x
    ori_y = map_data.info.origin.position.y

# callback of /odom subscriber
def update_pose(data):
    global odom, bot_x, bot_y
    odom = data
    bot_x = odom.pose.pose.position.x
    bot_y = odom.pose.pose.position.y

# update the position of the robot (the returned coordinate is only for visualization)
def bot_index(x, y):
    return np.array([int((x - ori_x)/resolution), int((y - ori_y)/resolution)])

# transform contour to real map coordinate
def to_map_coord(contour):
    for index in range(len(contour)):
        contour[index][0] = round(contour[index][0] * resolution + ori_x, 2)
        contour[index][1] = round(contour[index][1] * resolution + ori_y, 2)
    return contour

# thread to constantly update the position of the turtlebot
def get_bot_coord_thread(name):
    global bot_idx
    print(f"thread {name} starting")
    while True:
        time.sleep(1)
        bot_idx = np.array([bot_x, bot_y])
        print(f'bot coordinate {bot_idx}')

# calculate the frontiers and plot the position of the centroids, this function is only for visualization
def get_frontier():
    fig, ax = plt.subplots()
    while map_data.header.seq<1 or len(map_data.data)<1:
        pass
    data = map_data.data
    w = map_data.info.width
    h = map_data.info.height
    data_np = np.asarray(data)
    data_np = np.reshape(data_np, [h, w]);
    ## for visualizing ####
    plt.imshow(data_np, cmap='gray', vmin = -1, vmax = 100)
    plt.plot(bot_idx[0], bot_idx[1], 'bo--', linewidth=2, markersize=12)
    contour_m1 = find_contours(data_np, -1, fully_connected='high')
    # detect groups of potential frontiers by finding the contour of occupied
    # spots and unoccupied spots, then subtract them
    contour_0 = find_contours(data_np, 1, fully_connected='high')
    contour_m1 = np.concatenate(contour_m1, axis=0)
    contour_0 = np.concatenate(contour_0, axis=0)
    
    contour_0 = to_map_coord(contour_0)
    contour_m1 = to_map_coord(contour_m1)

    contour_0 = np.asarray(contour_0)
    contour_m1 = np.asarray(contour_m1)

    set_0 = set([tuple(x) for x in contour_0])
    set_m1 = set([tuple(x) for x in contour_m1])

    candidates = list(set_m1.difference(set_0))
    candidates = np.asarray(candidates)

    # group the frontiers using agglomerative (hierarchical clustering) 
    model = AgglomerativeClustering(n_clusters=3, affinity='euclidean', linkage='ward')
    model.fit(candidates)
    labels = model.labels_
    centroids = np.zeros([3, 2])
    dis = np.zeros([3, 1])
    for i in range(3):
        centroids[i, :] = np.mean(candidates[labels==i, :], axis = 0)
        dis[i] = norm(centroids[i, :] - bot_idx)
    print(dis)
    target = centroids[np.argmin(dis)]
    print(target)

    plt.scatter((candidates[labels==0, 1]-ori_x)/resolution, (candidates[labels==0, 0]-ori_y)/resolution, s=10, marker='x', color='red')
    plt.scatter((candidates[labels==1, 1]-ori_x)/resolution, (candidates[labels==1, 0]-ori_y)/resolution, s=10, marker='x', color='white')
    plt.scatter((candidates[labels==2, 1]-ori_x)/resolution, (candidates[labels==2, 0]-ori_y)/resolution, s=10, marker='x', color='green')
    plt.scatter((centroids[:, 1]-ori_x)/resolution, (centroids[:, 0]-ori_y)/resolution, s=50, marker='o', color='pink')


    plt.show()

# thread to find the frontiers and set target
def get_frontier_thread(name, lock):
    print(f'Thread {name} is starting')
    # these values can be changed
    n_cluster = 5
    knn = 7
    markerarray_publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
    global rviz_id
    rviz_id = 0

    while True:
        while map_data.header.seq<1 or len(map_data.data)<1:
            pass
        data = map_data.data
        w = map_data.info.width
        h = map_data.info.height
        data_np = np.asarray(data)
        data_np = np.reshape(data_np, [h, w]);
        lock.acquire()

        # detect groups of potential frontiers by finding the contour of occupied
        # spots and unoccupied spots, then subtract them
        contour_m1 = find_contours(data_np, -1, fully_connected='high')
        contour_0 = find_contours(data_np, 1, fully_connected='high')
        contour_m1 = np.concatenate(contour_m1, axis=0)
        contour_0 = np.concatenate(contour_0, axis=0)
        
        contour_0 = to_map_coord(contour_0)
        contour_m1 = to_map_coord(contour_m1)

        contour_0 = np.asarray(contour_0)
        contour_m1 = np.asarray(contour_m1)

        # convert contour np arrays into sets
        set_0 = set([tuple(x) for x in contour_0])
        set_m1 = set([tuple(x) for x in contour_m1])

        global target

        # stop condition
        candidates = list(set_m1.difference(set_0))
        if len(candidates) < 20:
            print('done')
            break

        # group the frontiers using agglomerative (hierarchical clustering) 
        # and choose the nearest group with knn classifier
        model = AgglomerativeClustering(n_clusters=n_cluster, affinity='euclidean', linkage='ward')
        model.fit(candidates)
        labels = model.labels_
        lbl_ign = []
        # ignore labels with too few candidates to avoid running into walls
        for i in range(n_cluster):
            if np.sum(labels==i) < 10:
                lbl_ign.append(i)
                print(f'ignore label {i}')
        dis = np.ones([len(candidates), 1])*10000000

        for i in range(len(candidates)):
            if labels[i] not in lbl_ign:
                dis[i] = norm(np.asarray(candidates[i]) - bot_idx)
        
        candidates = np.asarray(candidates)
        
        # implement knn classifier, choose the nearest cluster
        dis_sort = np.argsort(dis)
        nearest_idx = dis_sort[0:knn]
        potential_targets = labels[nearest_idx]
        target_label = np.argmax(np.bincount(potential_targets[:,0]))
        target = np.mean(candidates[labels==target_label, :], axis = 0)

        print(f'next target {target}')

        lock.release()

        # visualize the candidates in RVIZ as a MarkerArray object
        dim = resolution*2
        markerArray = MarkerArray()
        # loop throgh all instances of the array
        for index in range(len(candidates)):
            marker = Marker()
            marker.id = rviz_id
            marker.header.frame_id = "map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.lifetime = rospy.Duration(120.0)
            marker.scale.x = dim 
            marker.scale.y = dim
            marker.scale.z = dim
            marker.color = ColorRGBA(0.0, 0.0, 1.0, 0.75)
            # x and y are inverted due to nature of the map
            marker.pose.position.x = candidates[index, 1]
            marker.pose.position.y = candidates[index, 0]
            markerArray.markers.append(marker)

            rviz_id += 1
            if rviz_id == 1000:
                rviz_id = 0
        markerarray_publisher.publish(markerArray)

# thread client of move_base, target updated constantly
def movebase_thread(client, lock):
    # Define Marker object to visualize the next target
    global rviz_id
    marker_publisher = rospy.Publisher('target', Marker, queue_size=10)
    marker_goal = Marker()
    marker_goal.id = rviz_id
    marker_goal.header.frame_id = "map"
    marker_goal.type = marker_goal.SPHERE
    marker_goal.action = marker_goal.ADD
    marker_goal.scale.x = 0.5
    marker_goal.scale.y = 0.5
    marker_goal.scale.z = 0.5
    marker_goal.color.a = 1.0
    rviz_id += 1
    if rviz_id == 1000:
        rviz_id = 0
    print(f'Movebase thread starting')
    client.wait_for_server()
    print('connected to server')
    while True:
        lock.acquire()
        global prev_target

        # Sends the goal to the action server.
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(target[1])
        goal.target_pose.pose.position.y = float(target[0]) 
        goal.target_pose.pose.orientation.w = 1.0
        print(f'new target {target}')
        client.send_goal(goal)

        # Visualize the next goal as a Marker object in RVIZ
        marker_goal.pose.orientation.w = 1.0
        marker_goal.pose.position.x = target[1]
        marker_goal.pose.position.y = target[0]
        marker_goal.pose.position.z = 0
        marker_publisher.publish(marker_goal)

        lock.release()

# end the program with ctrl C
def signal_handler(sig, frame):
    print('Program ended by user')
    sys.exit()

if __name__ == '__main__':
    #Initializes a rospy node to let the SimpleActionClient publish and subscribe
    rospy.init_node('movebase_client_py')
    rate = rospy.Rate(10)

    movebase_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    odom_subscriber = rospy.Subscriber('/odom', Odometry, update_pose)
    map_client = rospy.Subscriber("/map", OccupancyGrid, map_callback)

    lock = threading.Lock()

    ## uncomment the line below for visualization
    # get_frontier()

    ## comment the part below to prevent robot from moving to visualize
    threadBotIdx = threading.Thread(target = get_bot_coord_thread, args = ('bot index', ))
    threadBotIdx.daemon = True
    threadBotIdx.start()

    threadMap = threading.Thread(target=get_frontier_thread, args=('random frontier', lock, ))
    threadMap.daemon = True
    threadMap.start()

    threadMove = threading.Thread(target=movebase_thread, args=(movebase_client, lock, ))
    threadMove.daemon = True
    threadMove.start()
    
    while threadMap.is_alive():
        signal.signal(signal.SIGINT, signal_handler)