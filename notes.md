# Setup 
```
cd ~/work_sushil/f1tenth_ws; source install/setup.bash
```

# Setup Alias
```
alias skws='cd ~/work_sushil/f1tenth_ws; source install/setup.bash'
```

# To build 
```
colcon build
```

# to test joystick
```
jstest /dev/input/js0
```

# to launch ROS stack
```
ros2 launch f1tenth_stack bringup_launch.py

ROS_LOG_DIR="/home/sushil/work_sushil/f1tenth_ws/debug" ros2 launch f1tenth_stack bringup_launch.py
```


# To visualize
```
skws
rviz2
```

# to Visualize the network graph of topics and nodes
```
cd ~/work_sushil/viz_ws
source install/setup.bash
ros2 run ros_network_viz ros_network_viz 
```

# To just run teleop for debug
```
ros2 launch joy_teleop example.launch.py
ros2 run joy joy_node
ros2 run joy joy_node --ros-args --log-level DEBUG
ros2 run joy joy_enumerate_devices
```

## if joystick does not show change permissions on the events file
```
sudo chmod 666  /dev/input/event1
```

# go generate maps
## run the stack
```
ros2 launch f1tenth_stack bringup_launch.py
```

## run the slam tool
```
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/sushil/work_sushil/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml
```

## save map when done 
```
ros2 run nav2_map_server map_saver_cli -f office-1
```


# to start simulator
<!-- cd /home/sushil/work_sushil/f1tenth_simulator
source install/setup.bash
colcon build
export AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH:/home/sushil/work_sushil/f1tenth_simulator/install/f1tenth_simulator
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/sushil/work_sushil/f1tenth_simulator/install/f1tenth_simulator/lib
ros2 launch f1tenth_simulator simulator.launch -->
```
cd /home/sushil/work_sushil/sim_ws
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
# to start the teleop keyboard control
ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 run aeb aeb_node --ros-args -pfull_break_threshold:=2.9

colcon build --packages-select wall_following && ros2 launch wall_following launch.py | tee a


export PYTHONPATH=/home/sushil/work/f1t-labs/f1tenth_gym/gym:$PYTHONPATH


```

# ROS Concepts
## Nodes 
```
ros2 run <package_name> <executable_name>
ros2 node list
ros2 node into <node_name>
```

## Topics
Publish subscribe protocol is via a topic
```
rqt_graph
ros2 topic list
ros2 topic list -t
ros2 topic echo <topic_name>
ros2 topic info <topic_name>
ros2 interface show <msg_type>
ros2 topic pub <topic_name> <msg_type> '<args>'
ros2 topic hz <topic_name>
```
## Service
Request reponse protocol
```
ros2 service list
ros2 service type <service_name>
ros2 service list -t
ros2 service find <type_name>
ros2 interface show <type_name>.srv
ros2 service call <service_name> <service_type> <arguments>
```

## Action
action is like a job request and can collect status of the task later 
request is via a goal and status is via a result

## Paramters
configuration for nodes
```
ros2 param list
ros2 param get <node_name> <parameter_name>
ros2 param set <node_name> <parameter_name> <value>
```

## workspace
directory containing ros2 packages
overlay - a secondary workspace where you can add additional packages without interferaing with existing workspace "Underlay"
### creating workspace
```
mkdir -p <workspace_name>/src
cd <workspace_name>
rosdep install -i --from-path src --rosdistro humble -y
```

## BUILD tool
```
colcon build
userful arguments 
- --packages-up-to : Builds the packages you want plu all it's dependenciesbut not the whole workspace
- --symlink-install : for python node so we can change source and not have to change in install or rebuild 
- --event-handlers console_direct+: shows console while building
```

### underlay 
```
source /opt/ros/humble/setup.bash
```
### overlay 
```
cd <workspacename>
source install/local_setup.bash 

source install/setup.bash does both underlay and overlay
```

### packages creation 
package creation in ros2 useas ament as its build system and colcon as its build tool
use cmake or python based packages

ros2 pkg create --build-type ament_cmake wall_following


### to debug build process
colcon build --packages-select gap_follow  --event-handlers console_cohesion+ --cmake-args "--trace-source src/gap_follow/CMakeList.txt"
