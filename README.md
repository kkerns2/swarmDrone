# Swarm Package
Senior Design for University of Central Florida. Sponsored By Lockheed Martin
<br>
## Phase 1: Autonomously Map the environment (SLAM):

### Steps to run 
##### 1. Launch World Simulation with Four Drones into Environment with Hector SLAM enabled
```console
roslaunch swarm world.launch slam:=true
```
##### 2. Launch BB8 into the world (arg x and y is for the spawn location of BB8)
```console
roslaunch bb8 main.launch
```
##### 3. Launch Four Drones Mid-Air
```console
src/swarm/launch/bash\ script/./takeoff.sh
```
##### 4. Launch first instance of YOLOv5 for drone1 and drone2
```console
rosrun detector yolo_multi.py drone1 drone2
```
##### 5. Launch second instance of YOLOv5 for drone3 and drone4
```console
rosrun detector yolo_multi.py drone3 drone4
```
##### 6. Enable Movebase & Explore Frontiers to allow the Drones to Navigate around the Map autonomously 
```console
roslaunch swarm start_frontier.launch
```
##### 7. Reset Hector SLAM
```console
src/swarm/launch/bash\ script/./reset_scan.sh
```
##### 8. Once you're satisfied with the completed map, saved the map 
```console
rosrun map_server map_saver --occ 100 --free 10 -f map:=/drone1/map ~/catkin_ws/src/swarm/maps/drone1_map
```
```console
rosrun map_server map_saver --occ 100 --free 10 -f map:=/drone2/map ~/catkin_ws/src/swarm/maps/drone2_map
```
```console
rosrun map_server map_saver --occ 100 --free 10 -f map:=/drone3/map ~/catkin_ws/src/swarm/maps/drone3_map
```
```console
rosrun map_server map_saver --occ 100 --free 10 -f map:=/drone4/map ~/catkin_ws/src/swarm/maps/drone4_map
```

## Phase 2: Navigation 
### Steps to run 
##### 1. Launch World Simulation with Four Drones into Environment 
```console
roslaunch swarm world.launch slam:=false
```
##### 2. Launch BB8 into the world (arg x and y is for the spawn location of BB8)
```console
roslaunch bb8 main.launch
```
##### 3. Launch Four Drones Mid-Air
```console
src/swarm/bash\ script/./takeoff.sh
```
##### 4. Launch first instance of YOLOv5 for drone1 and drone2
```console
rosrun detector yolo_multi.py drone1 drone2
```
##### 5. Launch second instance of YOLOv5 for drone3 and drone4
```console
rosrun detector yolo_multi.py drone3 drone4
```
##### 6. Enable Movebase & AMCL to allow the Four Drones to Navigate the Map
```console
roslaunch swarm start_move.launch
```
<br/>
