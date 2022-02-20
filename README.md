# First Assignment of the Research Track 2 course-ROS2 branch 

The package contains the nodes for controlling a mobile robot in the Gazebo simulation environment. The robot is controlled from both ROS and ROS2, using the ros2_bridge that allows the exchange of data between nodes of different workspaces.

## General  
The file: **my_mapping_rules.yaml** that contains the name of the package in ros and ros2 that we want to keep in connection. 
In this branch are present just the nodes that works in ROS2: **random_position_server** (file: position_service_ros2.cpp) and the node **state_machine** (file: state_machine_ros2.cpp). The other two nodes that we need to control the robot can be found in the branch **main**, they are: **go_to_point** (file:go_to_point.py) and **user_interface** (file: user_interface.py). The two nodes in ROS are written as components, because the bridge supports only composable nodes.

## How to run the code for controlling the robot?

Please follow this step below to run the code both in ros and ros2:

1. Open a terminal and **source ros.sh**
2. In that terminal type this command
```
roslaunch rt2_assignment1 sim.launch
```
3. Open one more terminal and **source ros12.sh** (the bridge)
4. In that terminal type this command:
```
ros2 run ros1_bridge dynamic_bridge --bridge
```
5. Open one more terminal and **source ros2.sh**
6. In this last one terminal type the command:
```
ros2 run rt2_assignment1 sim_container.py
```
If you want to run only the package in ROS2, please follow just the point 5 and 6 of the previous list. 

7. If you like to launch three steeps in one script you can first download package of gnome terminal.
8. After that you can run the script.
```
sudo apt update  
sudo apt install gnome-shell  

./script.sh
```

