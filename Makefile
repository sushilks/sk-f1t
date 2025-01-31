.PHONY: init gym aeb kbd wall_following build jstest ros_stack rviz
# PYTHONPATH=$(shell pwd)
# PYTHONPATH=${PYTHONPATH%"${PYTHONPATH##*[! ]}"}
# PYTHONPATH+=/../f1tenth_gym/gym:
# PYTHONPATH+=$(shell echo $$PYTHONPATH)
# export PYTHONPATH=$PWD/../f1tenth_gym/gym:$PYTHONPATH

export PYTHONPATH := $(PWD)/../f1tenth_gym/gym::$(PYTHONPATH)
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

gap_follow: 
	colcon build --packages-select gap_follow && ros2 launch gap_follow launch.py 

total_persuit_gym:
	colcon build --packages-select total_persuit && ros2 launch  total_persuit launch.py
total_persuit:
	colcon build --packages-select total_persuit && ros2 run total_persuit total_persuit --ros-args --params-file /home/sushil/work_sushil/skf1t/src/total_persuit/config/total_persuit_oncar.yaml
build:
	colcon build
test: 
	colcon test
test_result:
	colcon test-result --all 
	#--verbose
	# colcon test --ctest-args tests [package_selection_args]

gap_follow_test:
	colcon build --packages-select gap_follow  && ./build/gap_follow/gap_follow_gap_follow_test 

	#colcon test --packages-select gap_follow 
	#--ctest-args --rerun-failed --output-on-failure


jstest:
	jstest /dev/input/js0

ros_stack:
	ros2 launch f1tenth_stack bringup_launch.py

rviz:
	rviz2
slam:
	cd maps &&  ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/sushil/work_sushil/skf1t/maps/f1tenth_online_async.yaml
slam_localize:
	ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/sushil/work_sushil/skf1t/maps/f1tenth_localize.yaml
	

save_map:
	echo "to Save map with name sk-office use the following command"
	echo "to save map with pmg/yaml data"
	echo ros2 run nav2_map_server map_saver_cli -f sk-office
	echo "To save map with posegraph/data"
	echo ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename : 'sk-offic3'}"
