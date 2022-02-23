# Copyright (c) 2018 Intel Corporation
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
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from rosidl_generator_py import import_type_support
import json


def generate_launch_description():
    f = open(os.path.join(os.getcwd(), 'task9/task9.json'),)
    task_params = json.load(f)
    target_params= task_params["Target"]

    return LaunchDescription([

        Node(
            package='nav2_planner',
            executable='goal_sender',
            name='goal_sender',
            output='screen',
            parameters=[target_params])
    ])
