# Autonomous driving software for Tsukuba Challenge

## Requirements
Ubuntu: 18.04  
ROS: Melodic

## Build the Workpace
1. install dependencies
```
sudo apt install ros-melodic-velodyne*
sudo apt install ros-melodic-hector-gazebo-plugins
sudo apt install ros-$ROS_DISTRO-grid-map
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
1. launch gazebo and Rviz.  
we can choose one of the following robot models by changing roslaunch parameter `model`.
- whill_modelc (*default*)
- dtw_robot
- roomba
```
roslaunch rtw_gazebo rtw_gazebo.launch
```

2. launch joy node.
```
rosrun joy joy_node _autorepeat_rate:=30
```

3. launch the RTW node.
```
roslaunch localmap_2d localmap_2d.launch
roslaunch waypoint_loader waypoint_loader.launch
roslaunch dummy_localizer dummy_localizer.launch
roslaunch pure_pursuit pure_pursuit.launch
roslaunch rostate_machine control_state_machine.launch
roslaunch whill_interface whill_interface.launch mode:=simulation
```

### Case 2: real environment mode
1. launch WHILL driver.
```
roslaunch ros_whill ros_whill.launch
```

2. launch joy node.
```
rosrun joy joy_node _autorepeat_rate:=30
```

3. launch Rviz.
```
rosrun rviz rviz
```
you can use Rviz configuration file at "(path to RTW)/src/simulation/rtw_gazebo/rviz/rtw_default.rviz".

4. launch the RTW node for manipulating WHILL.
```
roslaunch localmap_2d localmap_2d.launch
roslaunch waypoint_loader waypoint_loader.launch
roslaunch dummy_localizer dummy_localizer.launch
roslaunch pure_pursuit pure_pursuit.launch
roslaunch rostate_machine control_state_machine.launch
roslaunch whill_interface whill_interface.launch mode:=real
```
