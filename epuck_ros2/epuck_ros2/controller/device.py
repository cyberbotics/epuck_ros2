"""Device class."""


class Device(object):
    tag = 0

    def __init__(self, name):
        self.name = name
        Device.tag += 1
        self.tag = Device.tag

    def getTag(self):
        return self.tag

    def getName(self):
        return self.name

    def getModel(self):
        return ''
