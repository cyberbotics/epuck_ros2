"""Gyro class."""

from .device import Device


class Gyro(Device):
    names = ['gyro']

    def __init__(self, name):
        Device.__init__(self, name)
        self.value = 0
        self.name = name
        self.samplingPeriod = 0

    def enable(self, samplingPeriod):
        self.samplingPeriod = samplingPeriod

    def disable(self):
        self.samplingPeriod = 0

    def getSamplingPeriod(self):
        return self.samplingPeriod

    def getValues(self):
        return self.value
