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

"""DistanceSensor class."""

from .device import Device


class DistanceSensor(Device):
    proximityNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
    groundNames = ['gs0', 'gs1', 'gs2']

    def __init__(self, name):
        Device.__init__(self, name)
        self.value = 0
        self.samplingPeriod = 0

    def enable(self, samplingPeriod):
        self.samplingPeriod = samplingPeriod

    def disable(self):
        self.samplingPeriod = 0

    def getSamplingPeriod(self):
        return self.samplingPeriod

    def getValue(self):
        return self.value

    def getMaxValue(self):
        if 'ds' in self.name:
            return 4095
        return 1000

    def getMinValue(self):
        if 'ds' in self.name:
            return 34
        return 300

    def getAperture(self):
        return 1.5708
