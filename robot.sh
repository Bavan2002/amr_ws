#!/bin/bash

# First command in the first tab
gnome-terminal --tab -- bash -c "source install/setup.bash && ros2 launch my_bot launch_robot_sim.launch.py world:=./src/my_bot/worlds/room2.world; exec bash"

# Delay before running the next command
sleep 10

# Second command in the second tab
gnome-terminal --tab -- bash -c "source install/setup.bash && ros2 run rviz2 rviz2 -d src/my_bot/config/main.rviz --ros-args -p use_sim_time:=true; exec bash"

# Delay before running the next command
sleep 2

# Third command in the third tab
gnome-terminal --tab -- bash -c "source install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard; exec bash"

# Delay before running the next command
sleep 2

# Fourth command in the fourth tab
gnome-terminal --tab -- bash -c "source install/setup.bash && ros2 launch my_bot localization_launch.py map:=my_map1.yaml; exec bash"

# Delay before running the next command
sleep 10

# Fifth command in the fifth tab
gnome-terminal --tab -- bash -c "source install/setup.bash && ros2 launch my_bot navigation_launch.py map_subscribe_transient_local:=true; exec bash"

