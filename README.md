# Autonomous driving software for Tsukuba Challenge

## Requirements
Ubuntu: 18.04
ROS: Melodic

## Build the Workpace
install dependencies
```
sudo apt install ros-melodic-velodyne*
```

make workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ../
catkin_make
```

clone & build of Reinventing-the-Wheel package
```
cd ~/catkin_ws/src
git clone https://github.com/rigelVY/Reinventing-the-Wheel.git
cd ../
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

launch the RTW node for manipulating WHILL.
```
roslaunch waypoint_loader waypoint_loader.launch
roslaunch dummy_localizer dummy_localizer.launch
roslaunch pure_pursuit pure_pursuit.launch
roslaunch whill_interface whill_interface.launch
```
