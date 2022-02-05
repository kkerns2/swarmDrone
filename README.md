# Autonomous Drone Swarm in Gazebo and ROS Noetic

Implements a drone swarm consisting of 4-5 drones.  Incorporates path planning algorithms and object detection.  The swarm traverses an urban environment
simulated in gazebo and implemented in ROS Noetic.  

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
# make catkin_ws
cd ~/
mkdir catkin_ws
cd catkin_ws/
mkdir src
catkin_make

# clone gazebo models and repo
cd ~/catkin_ws/src/
git clone https://github.com/chaolmu/gazebo_models_worlds_collection.git
git clone https://github.com/kkerns2/swarmDrone.git
cd swarmDrone
mv * ..
mv .* ..

# clone submodules
cd ~/catkin_ws/src/swarmDrone/
git submodule update --init --recursive
```
