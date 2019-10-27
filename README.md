# Autonomous driving software for Tsukuba Challenge

## Requirements
Ubuntu: 18.04  
ROS: Melodic

## Build the Workpace
1. install dependencies
```
sudo apt install ros-$ROS_DISTRO-serial
sudo apt install ros-$ROS_DISTRO-velodyne*
sudo apt install ros-$ROS_DISTRO-hector-gazebo-plugins
sudo apt install ros-$ROS_DISTRO-gridmap*
sudo apt install ros-$ROS_DISTRO-gmapping ros-$ROS_DISTRO-amcl ros-$ROS_DISTRO-map-server 
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
1. launch gazebo, Rviz and `joy`.  
we can choose one of the following robot models by changing roslaunch parameter `model`.
- whill_modelc (*default*)
- dtw_robot
- roomba
```
roslaunch rtw_gazebo rtw_gazebo.launch
```

2. launch `amcl` and `map_server` nodes.
```
rosrun map_server map_server src/data/map/mymap.yaml
rosrun amcl amcl
```

3. launch the RTW nodes.
```
roslaunch rostate_machine control_state_machine.launch
roslaunch whill_interface whill_interface.launch mode:=simulation
roslaunch localmap_2d localmap_2d.launch
roslaunch waypoint_loader waypoint_loader.launch
roslaunch dummy_localizer dummy_localizer.launch
roslaunch dwa dwa.launch
```

### Case 2: real environment mode
1. launch whill driver, Rviz and `joy`.  
```
roslaunch rtw_gazebo rtw_startup.launch 
```

2. launch velodyne node.
```
roslaunch velodyne_pointcloud VLP16_points.launch
```

3. launch `amcl` and `map_server` nodes.
```
rosrun map_server map_server src/data/map/mymap.yaml
rosrun amcl amcl
```

4. launch the RTW nodes.
```
roslaunch rostate_machine control_state_machine.launch
roslaunch whill_interface whill_interface.launch mode:=real
roslaunch localmap_2d localmap_2d.launch
roslaunch waypoint_loader waypoint_loader.launch
roslaunch dummy_localizer dummy_localizer.launch
roslaunch dwa dwa.launch
```
