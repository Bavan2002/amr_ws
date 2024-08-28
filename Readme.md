pyserial-miniterm -e /dev/ttyUSB0 57600

sudo usermod -aG dialout $USER

int Kp = 13;
int Kd = 7;
int Ki = 0;
int Ko = 1;

int Kp2 = 20;
int Kd2 = 9;
int Ki2 = 0;
int Ko2 = 1;


source install/setup.bash
ros2 launch my_bot launch_robot_sim.launch.py \
world:=./src/my_bot/worlds/room2.world

source install/setup.bash
ros2 launch my_bot launch_robot.launch.py

source install/setup.bash
ros2 run rviz2 rviz2 -d src/my_bot/config/main.rviz \
--ros-args -p use_sim_time:=true

source install/setup.bash
ros2 run tele teleop_twist_keyboard

source install/setup.bash
ros2 launch my_bot online_async_launch.py

source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f my_map

source install/setup.bash
ros2 launch my_bot localization_launch.py map:=my_map1.yaml

source install/setup.bash
ros2 launch my_bot navigation_launch.py map_subscribe_transient_local:=true

source install/setup.bash
ros2 run initial_pose_handler initial_pose_handler

225/20 = 11.25


Install These Packages

sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \ 
ros-humble-twist-mux ros-humble-turtlebot3* ros-humble-gazebo-ros-pkgs \
ros-humble-xacro ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control 
ros-humble-gazebo-ros2-pkgs
