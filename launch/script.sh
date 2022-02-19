#!/bin/bash
gnome-terminal --tab --title="gazebo_simulation" -- bash -c "cd /home; source ros.sh; roslaunch rt2_assignment1 ros2_sim.launch"
gnome-terminal --tab --title="bridge" -- bash -c "cd /home; sleep 3; source ros12.sh; ros2 run ros1_bridge dynamic_bridge"
gnome-terminal --tab --title="ros2_component_container" -- bash -c "cd /home; source ros2.sh; ros2 launch rt2_assignment1 container.py"
