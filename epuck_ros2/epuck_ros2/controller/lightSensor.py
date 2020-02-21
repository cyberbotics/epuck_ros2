"""LightSensor class."""

from .device import Device


class LightSensor(Device):
    names = ['ls0', 'ls1', 'ls2', 'ls3', 'ls4', 'ls5', 'ls6', 'ls7']

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
