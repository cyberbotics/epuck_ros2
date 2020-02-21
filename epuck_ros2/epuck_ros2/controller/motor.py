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
