# RB-Y1 SLAM Simulation
A SLAM simulation of RB-Y1 robot using gmapping algorithm in warehouse environment

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 20.04.

ROS Noetic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **ROS Package**
```
sudo apt-get install ros-noetic-hector-trajectory-server ros-noetic-slam-gmapping ros-noetic-navigation
```

## 2. Build 
### 2.1 Clone repository:
```
    cd ~/catkin_ws/src
    git clone https://github.com/jihyun9777/RB-Y1_SLAM_Simulation.git
    cd ..
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

### 2.2 Launch ROS
```
    roslaunch SLAM_simulation SLAM_simulation.launch
```
Note that it takes a few minutes to load model upon first launch

### 2.3 Autonomous Navigation
You may set target points in RVIZ and the robot will navigate to the location in gazebo.
1. click 2d nav goal button on rviz
2. click any points you want on the map

## 3.Acknowledgements
Thanks for the great work from (https://github.com/wh200720041/warehouse_simulation_toolkit.git).


