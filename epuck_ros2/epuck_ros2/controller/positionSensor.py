"""PositionSensor class."""

from .device import Device


class PositionSensor(Device):
    names = ['left wheel sensor', 'right wheel sensor']

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
