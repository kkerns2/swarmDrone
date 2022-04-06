# Swarm Package
Senior Design for University of Central Florida. Sponsored By Lockheed Martin
<br>
## Phase 1: Autonomously Map the environment (SLAM):

### Steps to run 
##### 1. Launch World Simulation with Four Drones into Environment with Hector SLAM enabled
```console
roslaunch swarm world.launch slam:=true
```
##### 2. Launch bb8 into the world (arg x and y is for the spawn location of bb8)
```console
roslaunch bb8 main.launch
```
##### 3. Launch Four Drones Mid-Air
```console
cd bash_script/
```
```console
./takeoff.sh
```
##### 4. Reset Hector SLAM
```console
cd bash_script/
```
```console
./reset_scan.sh
```
##### 5. Launch YOLOv5 for object detection of bb8 for drone1, drone2, drone3 and drone4
```console
rosrun swarm yolo_slam.py drone1 drone2 drone3 drone4
```
##### 6. Enable Movebase & Explore Frontiers to allow the Drones to Navigate around the Map autonomously 
```console
roslaunch swarm start_frontier.launch
```
##### 7. Once you're satisfied with the completed map, saved the map 
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
##### 2. Launch bb8 into the world (arg x and y is for the spawn location of bb8)
```console
roslaunch bb8 main.launch
```
##### 3. Launch Four Drones Mid-Air
```console
cd bash_script/
```
```console
./takeoff.sh
```
##### 4. Launch YOLOv5 for object detection of bb8 for drone1, drone2, drone3 and drone4
```console
rosrun swarm yolo_nav.py drone1 drone2 drone3 drone4
```
##### 5. Enable Movebase & AMCL to allow the Four Drones to Navigate the Map
```console
roslaunch swarm start_move.launch
```
<br/>
