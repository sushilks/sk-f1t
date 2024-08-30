.PHONY: init gym aeb kbd wall_following build jstest ros_stack rviz

#init:
#	source ~/work/sim_ws/install/setup.bash
#	source ~/work/skf1t/install/local_setup.bash

gym:
	ros2 launch f1tenth_gym_ros gym_bridge_launch.py
aeb:
	colcon build --packages-select  aeb && ros2 run aeb aeb_node 
	# --ros-args -pfull_break_threshold:=2.9
kbd:
	ros2 run teleop_twist_keyboard teleop_twist_keyboard

wall_following: 
	colcon build --packages-select wall_following && ros2 launch wall_following launch.py 

build:
	colcon build

jstest:
	jstest /dev/input/js0

ros_stack:
	ros2 launch f1tenth_stack bringup_launch.py

rviz:
	rviz2