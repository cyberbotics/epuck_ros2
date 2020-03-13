#!/usr/bin/env python

# Copyright 1996-2019 Cyberbotics Ltd.
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

"""Launch e-puck2 driver nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node
from webots_ros2_core.utils import ControllerLauncher


def generate_launch_description():
    driver = ControllerLauncher(package='epuck_ros2_driver',
                                node_executable='driver',
                                output='screen')

    laser_tf = Node(package='tf2_ros',
                    node_executable='static_transform_publisher',
                    output='screen',
                    arguments=[
                        '0.0', '0.0', '0.0',
                        '1.0', '0.0', '0.0', '0.0',
                        'base_link',
                        'laser_frame'
                    ])

    return LaunchDescription([driver, laser_tf])
