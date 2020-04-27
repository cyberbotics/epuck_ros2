#!/usr/bin/env python

# Copyright 1996-2020 Cyberbotics Ltd.
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

"""Launch E-Puck controller."""

import launch
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    use_camera = LaunchConfiguration('camera', default=False)

    driver = Node(
        package='epuck_ros2_driver',
        node_executable='driver',
        output='screen'
    )
    camera = Node(
        package='epuck_ros2_camera',
        node_executable='camera',
        output='screen',
        condition=launch.conditions.IfCondition(use_camera)
    )

    return LaunchDescription([
        driver,
        camera
    ])
