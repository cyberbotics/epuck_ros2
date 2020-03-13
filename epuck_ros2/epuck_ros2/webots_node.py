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

"""
Resolves high CPU usage of ROS2 timer.

More details is available at the following URL:
https://github.com/ros2/rclpy/issues/520.
"""

import threading
import time
from rclpy.node import Node


class TimerThread(threading.Thread):
    def __init__(self, period, callback, event):
        self.callback = callback
        self.event = event
        self.period = period
        super(TimerThread, self).__init__()

    def run(self):
        last_time = time.time()
        while not self.event.wait(self.period - (time.time() - last_time)):
            last_time = time.time()
            self.callback()


class WebotsNode(Node):
    def __init__(self, name, args=None):
        super().__init__(name)

    def create_timer(self, period, callback):
        timer_event = threading.Event()
        timer_thread = TimerThread(period, callback, timer_event)
        timer_thread.start()
