# Autonomous driving software for Tsukuba Challenge

## Requirements
Ubuntu: 18.04
ROS: Melodic

## Build the Workpace


## Launch RTW and manipulate WHILL
Launch WHILL driver.
```
roslaunch ros_whill ros_whill.launch
```

Launch Rviz.
```
rosrun rviz rviz
```

Launch the RTW node for manipulating WHILL.
```
roslaunch waypoint_loader waypoint_loader.launch
roslaunch dummy_localizer dummy_localizer.launch
roslaunch pure_pursuit pure_pursuit.launch
roslaunch whill_interface whill_interface.launch
```
