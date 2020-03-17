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

"""LED class."""

from .device import Device


class LED(Device):
    names = ['led0', 'led1', 'led2', 'led3', 'led4', 'led5', 'led6', 'led7', 'led8', 'led9',
             'pi-puck led 0', 'pi-puck led 1', 'pi-puck led 2']

    def __init__(self, name):
        Device.__init__(self, name)
        self.value = 0
        self.changed = True

    def set(self, value):
        newValue = min(value, 0xFFFFFF)
        self.changed = False if newValue == self.value else True
        self.value = min(value, 0xFFFFFF)

    def get(self):
        self.value = 0
