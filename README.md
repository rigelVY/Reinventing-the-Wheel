# Autonomous driving software for Tsukuba Challenge

## Requirements
Ubuntu: 18.04  
ROS: Melodic

## Build the Workpace
install dependencies
```
sudo apt install ros-melodic-velodyne*
```

clone & build of Reinventing-the-Wheel package
```
cd ~
git clone https://github.com/rigelVY/Reinventing-the-Wheel.git
cd Reinventing-the-Wheel
catkin_make
```

## Launch RTW and manipulate WHILL
launch WHILL driver.
```
roslaunch ros_whill ros_whill.launch
```

launch Rviz.
```
rosrun rviz rviz
```

### Case 1: simulation mode
launch gazebo and Rviz
```
roslaunch rtw_gazebo rtw_gazebo.launch
```

launch the RTW node
```
roslaunch waypoint_loader waypoint_loader.launch
roslaunch dummy_localizer dummy_localizer.launch
roslaunch pure_pursuit pure_pursuit.launch twist_topic:=/roomba/diff_drive_controller/cmd_vel
```

### Case 2: real environment mode
launch the RTW node for manipulating WHILL.
```
roslaunch waypoint_loader waypoint_loader.launch
roslaunch dummy_localizer dummy_localizer.launch
roslaunch pure_pursuit pure_pursuit.launch
roslaunch whill_interface whill_interface.launch
```
