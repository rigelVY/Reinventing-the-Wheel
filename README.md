# Autonomous driving software for Tsukuba Challenge

| *master* | *develop* |
|----------|-----------|
|[![Build Status](https://travis-ci.org/rigelVY/Reinventing-the-Wheel.svg?branch=master)](https://travis-ci.org/rigelVY/Reinventing-the-Wheel)|[![Build Status](https://travis-ci.org/rigelVY/Reinventing-the-Wheel.svg?branch=develop)](https://travis-ci.org/rigelVY/Reinventing-the-Wheel)|

<div align="center">
    <img src=img/rtw_overview.png width=90%>
</div>

## Requirements
Ubuntu: 18.04  
ROS: Melodic

## Build the Workpace
1. install dependencies
```
sudo apt install ros-$ROS_DISTRO-serial
sudo apt install ros-$ROS_DISTRO-velodyne*
sudo apt install ros-$ROS_DISTRO-hector-gazebo-plugins
sudo apt install ros-$ROS_DISTRO-grid-map
sudo apt install ros-$ROS_DISTRO-gmapping ros-$ROS_DISTRO-amcl ros-$ROS_DISTRO-map-server 
sudo apt install ros-$ROS_DISTRO-jsk-visualization
```

2. clone & build of Reinventing-the-Wheel package
```
cd ~
git clone https://github.com/rigelVY/Reinventing-the-Wheel.git
cd Reinventing-the-Wheel
catkin_make
```

## Launch RTW and manipulate WHILL

### Case 1: simulation mode
1. launch RTW nodes.
```
roslaunch rtw_bringup rtw_bringup.launch mode:=simulation
```

we can choose one of the following robot models by setting `model`.
- whill_modelc (*default*)
- dtw_robot
- roomba
```
roslaunch rtw_bringup rtw_bringup.launch mode:=simulation model:=whill_modelc
```

### Case 2: real environment mode
1. launch velodyne node.
```
roslaunch velodyne_pointcloud VLP16_points.launch
```

2. launch RTW nodes.
```
roslaunch rtw_bringup rtw_bringup.launch mode:=real
```
