# Autonomous driving software for Tsukuba Challenge

## Requirements
Ubuntu: 18.04  
ROS: Melodic

## Build the Workpace
install dependencies
```
sudo apt install ros-melodic-velodyne*
sudo apt install ros-melodic-hector-gazebo-plugins
```

clone & build of Reinventing-the-Wheel package
```
cd ~
git clone https://github.com/rigelVY/Reinventing-the-Wheel.git
cd Reinventing-the-Wheel
catkin_make
```

## Launch RTW and manipulate WHILL

### Case 1: simulation mode
1. launch gazebo and Rviz.  
we can choose one of the following robot models.
- whill_modelc (*default*)
- dtw_robot
- roomba

```
roslaunch rtw_gazebo rtw_gazebo.launch model:=<robot model>
```

2. launch the RTW node.
```
roslaunch waypoint_loader waypoint_loader.launch
roslaunch dummy_localizer dummy_localizer.launch
roslaunch pure_pursuit pure_pursuit.launch twist_topic:=/<robot model>/diff_drive_controller/cmd_vel
```

### Case 2: real environment mode
1. launch WHILL driver.
```
roslaunch ros_whill ros_whill.launch
```

2. launch Rviz.
```
rosrun rviz rviz
```

3. launch the RTW node for manipulating WHILL.
```
roslaunch waypoint_loader waypoint_loader.launch
roslaunch dummy_localizer dummy_localizer.launch
roslaunch pure_pursuit pure_pursuit.launch
roslaunch whill_interface whill_interface.launch
```
