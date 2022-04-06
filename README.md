# SwarmDrone Package
Senior Design for University of Central Florida. Sponsored By Lockheed Martin

Implements a drone swarm consisting of 4-5 drones.  Incorporates path planning algorithms and object detection.  The swarm traverses an urban environment
simulated in Gazebo and implemented in ROS Noetic.  
<br>
# Steps to run the SwarmDrone Package

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

# Requirements
## Newcomers, running the SwarmDrone Package for the FIRST TIME: 

## Install ROS and Environment
```
# enable repo
sudo add-apt-repository universe
sudo add-apt-repository multiverse
sudo add-apt-repository restricted

# install dev libraries
sudo apt-get install libsdl-image1.2-dev

# setup source list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# setup keys
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# install
sudo apt update
sudo apt install ros-noetic-desktop-full

# install dependencies for building packages
source /opt/ros/noetic/setup.bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# auto source whenever bash starts
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/gazebo_models_worlds_collection/models" >> ~/.bashrc
source ~/.bashrc

# setup gazebo
. /usr/share/gazebo/setup.sh
```

## Set up workspace
```
# Create catkin_ws
cd ~/
mkdir catkin_ws
cd catkin_ws/
mkdir src
catkin build

# clone gazebo models and repo
cd ~/catkin_ws/src/
git clone https://github.com/ros-geographic-info/unique_identifier.git
git clone https://github.com/ros-geographic-info/geographic_info.git
git clone https://github.com/kkerns2/swarmDrone.git
```

