# Copyright 2019 Canonical, Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch.actions
import launch_ros.actions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

# Please note that this is only an example!
# It is not guaranteed to work with your setup but can be used as a starting point.


def generate_launch_description():

    parameters_file = os.path.join(
        get_package_share_directory('wall_following'),
        'config', 'wall_following.yaml'
    )
    #/home/sushil/work/skf1t/install/wall_following/share/wall_following/launch
    #gym_ws_path = os.getcwd() + "/../sim_ws/install/f1tenth_gym_ros/share/f1tenth_gym_ros/launch"
    gym_ws_path = os.getcwd() + "/../f1tenth/install/f1tenth_gym_ros/share/f1tenth_gym_ros/launch"

    ld = LaunchDescription([
        launch.actions.DeclareLaunchArgument('wall_following_config', default_value=parameters_file),
        launch.actions.LogInfo(msg=[
            'Including launch file located at: ', gym_ws_path , '/gym_bridge_launch.py'
        ]),
        launch.actions.LogInfo(msg = [launch.substitutions.LaunchConfiguration('wall_following_config')]),
      launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gym_ws_path , '/gym_bridge_launch.py']),
            # launch_arguments={'node_name': 'bar'}.items(),
        ),  
    ])

    ld.add_action(launch_ros.actions.Node(
            package='wall_following', executable='wall_following_node',
            parameters=[launch.substitutions.LaunchConfiguration('wall_following_config')]))

    return ld
