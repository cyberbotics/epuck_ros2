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

"""Motor class."""

from .device import Device


class Motor(Device):
    names = ['left wheel motor', 'right wheel motor']

    def __init__(self, name):
        Device.__init__(self, name)
        self.targetPosition = 0
        self.velocity = 0

    def setPosition(self, position):
        self.targetPosition = position

    def setVelocity(self, vel):
        if vel > self.getMaxVelocity():
            vel = self.getMaxVelocity()
        self.velocity = vel

    def getTargetPosition(self):
        return self.targetPosition

    def getMinPosition(self):
        return 0

    def getMaxPosition(self):
        return 0

    def getVelocity(self):
        return self.velocity

    def getMaxVelocity(self):
        return 7.536
