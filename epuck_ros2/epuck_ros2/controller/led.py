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
