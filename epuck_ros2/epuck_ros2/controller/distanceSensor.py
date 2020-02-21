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
