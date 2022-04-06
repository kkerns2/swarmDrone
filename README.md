# SwarmDrone Package
Senior Design for University of Central Florida. Sponsored By Lockheed Martin

- Implements a drone swarm consisting of 4-5 drones.  Incorporates path planning algorithms and object detection.  The swarm traverses an urban environment
simulated in Gazebo and implemented in ROS Noetic.  

# Requirements: 

## A. Install ROS and Environment:
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

## B. Set up workspace:
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

## C. Files that need to be changed: 
- If your workspace isn't named **catkin_ws**, changed the following: 

    ### 1. Go to swarmDrone/bash_script and modify the following files:
    <ul>
        <li>drone1_reset.sh</li>
        <li>drone2_reset.sh</li>
        <li>drone3_reset.sh</li>
        <li>drone4_reset.sh</li>
        <li>drone1_takeoff.sh</li>
        <li>drone2_takeoff.sh</li>
        <li>drone3_takeoff.sh</li>
        <li>drone4_takeoff.sh</li>
    </ul>

    #### Change this line with the name of your current workspace
    ```
    Before:                                       After:
    source ~/catkin_ws/devel/setup.bash     ->  source ~/name_of_your_workspace/devel/setup.bash 
    ```

    ### 2. Go to swarmDrone/swarm/scripts/ and modify the following files:
    <ul>
        <li>drone1_client.py</li>
        <li>drone2_client.py</li>
        <li>drone3_client.py</li>
        <li>drone4_client.py</li>
    </ul>

    #### Change this line with the name of your current workspace
    ```
    Before:                                                                                   After:
    with open('/home/mcp/catkin_ws/src/swarm/waypoints/waypoints_d1.txt', 'r') as f:    ->    with open('/home/mcp/name_of_your_workspace/src/swarm/waypoints/waypoints_d1.txt', 'r') as f:
    ```

## D. Move the bash_script folder where your src folder resides:

```
catkin_ws/
    -> bash_script
       build
       devel
       logs
       src
          swarmDrone
```
<br>

# Steps to run the SwarmDrone Package:

## Phase 1: Autonomously Map the environment (SLAM):

### Steps to run 
##### 1. Launch World Simulation with Four Drones into Environment with Hector SLAM enabled:
```console
roslaunch swarm world.launch slam:=true
```
##### 2. Launch bb8 into the world (arg x and y is for the spawn location of bb8):
```console
roslaunch bb8 main.launch
```
##### 3. Launch Four Drones Mid-Air:
```console
cd bash_script/
```
```console
./takeoff.sh
```
##### 4. Reset Hector SLAM:
```console
cd bash_script/
```
```console
./reset_scan.sh
```
##### 5. Launch YOLOv5 for object detection of bb8 for drone1, drone2, drone3 and drone4:
```console
rosrun swarm yolo_slam.py drone1 drone2 drone3 drone4
```
##### 6. Enable Movebase & Explore Frontiers to allow the Drones to Navigate around the Map autonomously: 
```console
roslaunch swarm start_frontier.launch
```
##### 7. Once you're satisfied with the completed map, saved the map: 
```console
rosrun map_server map_saver --occ 100 --free 10 -f map:=/drone1/map ~/catkin_ws/src/swarmDrone/swarm/maps/drone1_map
```
```console
rosrun map_server map_saver --occ 100 --free 10 -f map:=/drone2/map ~/catkin_ws/src/swarmDrone/swarm/maps/drone2_map
```
```console
rosrun map_server map_saver --occ 100 --free 10 -f map:=/drone3/map ~/catkin_ws/src/swarmDrone/swarm/maps/drone3_map
```
```console
rosrun map_server map_saver --occ 100 --free 10 -f map:=/drone4/map ~/catkin_ws/src/swarmDrone/swarm/maps/drone4_map
```

##### 8. Merge your Saved Maps into a Master Map:

##### Copy the following maps and insert it into: catkin_ws/src/swarmDrone/Map-Merge-Tool/build/figure/
  <ul>
        <li>drone1_map.pgm</li>
        <li>drone2_map.pgm</li>
        <li>drone3_map.pgm</li>
        <li>drone4_map.pgm</li>
  </ul>

```console
cd src/swarmDrone/Map-Merge-Tool/build/
```

```console
./DisplayImage figure/drone1_map.pgm figure/drone2_map.pgm figure/drone_a.pgm
```

```console
./DisplayImage figure/drone3_map.pgm figure/drone4_map.pgm figure/drone_b.pgm
```
```console
./DisplayImage figure/drone_a.pgm figure/drone_b.pgm result/urban_result.pgm
```
### Copy drone1_map.yaml and rename it to urban_result.yaml, and leave it in catkin_ws/src/swarmDrone/swarm/maps

#### Edit urban_result.yaml on line 1: 
```
Before:                     After
image: drone1_map.pgm       image: urban_result.pgm
```
### Copy urban_result.pgm and insert it in catkin_ws/src/swarmDrone/swarm/maps

## Phase 2: Navigation 
### Steps to run 
##### 1. Launch World Simulation with Four Drones into Environment: 
```console
roslaunch swarm world.launch slam:=false
```
##### 2. Launch bb8 into the world (arg x and y is for the spawn location of bb8):
```console
roslaunch bb8 main.launch
```
##### 3. Launch Four Drones Mid-Air:
```console
cd bash_script/
```
```console
./takeoff.sh
```
##### 4. Launch YOLOv5 for object detection of bb8 for drone1, drone2, drone3 and drone4:
```console
rosrun swarm yolo_nav.py drone1 drone2 drone3 drone4
```
##### 5. Enable Movebase & AMCL to allow the Four Drones to Navigate the Map:
```console
roslaunch swarm start_move.launch
```
<br/>
