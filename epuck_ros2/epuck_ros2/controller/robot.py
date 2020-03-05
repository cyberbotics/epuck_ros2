"""Robot class."""


import sys
import time
from smbus2 import SMBus, i2c_msg
import VL53L0X

from .accelerometer import Accelerometer
from .distanceSensor import DistanceSensor
from .gyro import Gyro
from .led import LED
from .lightSensor import LightSensor
from .motor import Motor
from .positionSensor import PositionSensor

I2C_CHANNEL = 4
MAIN_ADDR = 0x1F
GROUND_ADDR = 0x60
IMU_ADDR = 0x68
FT903_I2C_CHANNEL = 3
FT903_ADDR = 0x1C
ACTUATORS_SIZE = 20
SENSORS_SIZE = 47
GROUND_SENSORS_SIZE = 6
IMU_SIZE = 12


class Robot(object):
    MODE_SIMULATION = 0
    MODE_CROSS_COMPILATION = 1
    MODE_REMOTE_CONTROL = 2

    def __init__(self):
        self.time = 0
        self.tof = VL53L0X.VL53L0X(i2c_bus=4, i2c_address=0x29)
        self.tof.open()
        self.tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)
        self.bus = SMBus(I2C_CHANNEL)
        self.busFT903 = SMBus(FT903_I2C_CHANNEL)
        self.devices = {}
        self.previousTime = None
        self.customData = ''
        for name in DistanceSensor.groundNames + DistanceSensor.proximityNames:
            self.devices[name] = DistanceSensor(name)
        for name in LightSensor.names:
            self.devices[name] = LightSensor(name)
        for name in PositionSensor.names:
            self.devices[name] = PositionSensor(name)
        for name in Motor.names:
            self.devices[name] = Motor(name)
        for name in LED.names:
            self.devices[name] = LED(name)
        for name in Accelerometer.names:
            self.devices[name] = Accelerometer(name)
        for name in Gyro.names:
            self.devices[name] = Gyro(name)
        self.devices['tof'] = DistanceSensor('tof')
        print('Starting controller.')

    def step(self, duration, blocking=False):
        if blocking:
            now = time.time()
            if self.previousTime is not None:
                diff = now - (self.previousTime + 0.001 * duration)
                if diff < 0:
                    time.sleep(-diff)
                    self.time += 0.001 * duration
                else:
                    self.time += diff + 0.001 * duration
            else:
                self.time += 0.001 * duration
            self.previousTime = time.time()

        self.devices['tof'].value = self.tof.get_distance() / 1000.0

        actuatorsData = []
        # left motor
        # 0.00628 = (2 * pi) / encoder_resolution
        leftSpeed = int(
            self.devices['left wheel motor'].getVelocity() / 0.0068)
        actuatorsData.append(leftSpeed & 0xFF)
        actuatorsData.append((leftSpeed >> 8) & 0xFF)
        # right motor
        rightSpeed = int(
            self.devices['right wheel motor'].getVelocity() / 0.0068)
        actuatorsData.append(rightSpeed & 0xFF)
        actuatorsData.append((rightSpeed >> 8) & 0xFF)
        # speaker sound
        actuatorsData.append(0)
        # LED1, LED3, LED5, LED7 on/off flag
        actuatorsData.append((self.devices['led0'].value & 0x1) |
                             (self.devices['led2'].value & 0x2) |
                             (self.devices['led4'].value & 0x4) |
                             (self.devices['led6'].value & 0x8))
        # LED2 R/G/B
        actuatorsData.append(
            int(((self.devices['led1'].value >> 16) & 0xFF) / 2.55))
        actuatorsData.append(
            int(((self.devices['led1'].value >> 8) & 0xFF) / 2.55))
        actuatorsData.append(int((self.devices['led1'].value & 0xFF) / 2.55))
        # LED4 R/G/B
        actuatorsData.append(
            int(((self.devices['led3'].value >> 16) & 0xFF) / 2.55))
        actuatorsData.append(
            int(((self.devices['led3'].value >> 8) & 0xFF) / 2.55))
        actuatorsData.append(int((self.devices['led3'].value & 0xFF) / 2.55))
        # LED6 R/G/B
        actuatorsData.append(
            int(((self.devices['led5'].value >> 16) & 0xFF) / 2.55))
        actuatorsData.append(
            int(((self.devices['led5'].value >> 8) & 0xFF) / 2.55))
        actuatorsData.append(int((self.devices['led5'].value & 0xFF) / 2.55))
        # LED8 R/G/B
        actuatorsData.append(
            int(((self.devices['led7'].value >> 16) & 0xFF) / 2.55))
        actuatorsData.append(
            int(((self.devices['led7'].value >> 8) & 0xFF) / 2.55))
        actuatorsData.append(int((self.devices['led7'].value & 0xFF) / 2.55))
        # Settings
        actuatorsData.append(0)
        # Checksum
        checksum = 0
        for data in actuatorsData:
            checksum ^= data

        actuatorsData.append(checksum)
        if len(actuatorsData) != ACTUATORS_SIZE:
            sys.exit('Wrond actuator data size.')
        # communication with i2c bus with main board address
        write = i2c_msg.write(MAIN_ADDR, actuatorsData)
        read = i2c_msg.read(MAIN_ADDR, SENSORS_SIZE)
        try:
            self.bus.i2c_rdwr(write, read)
        except:
            return
        sensorsData = list(read)

        checksum = 0
        for data in sensorsData:
            checksum ^= checksum ^ data
        if sensorsData[SENSORS_SIZE - 1] != checksum:
            print('Wrogn receiving checksum')
            return

        if len(sensorsData) != SENSORS_SIZE:
            sys.exit('Wrond actuator data size.')
        # Read and assign DistanceSensor values
        for i in range(8):
            self.devices[DistanceSensor.proximityNames[i]].value = (
                sensorsData[i * 2] & 0x00FF) | ((sensorsData[i * 2 + 1] << 8) & 0xFF00)
        # Read and assign LightSensor values
        for i in range(8):
            self.devices[LightSensor.names[i]].value = sensorsData[i *
                                                                   2 + 16] + (sensorsData[i * 2 + 17] << 8)
        # Read and assign PositionSensor values
        for i in range(2):
            self.devices[PositionSensor.names[i]].value = (
                sensorsData[i * 2 + 41] & 0x00FF) | ((sensorsData[i * 2 + 42] << 8) & 0xFF00)
            # 159.23 = encoder_resolution/ (2 * pi)
            self.devices[PositionSensor.names[i]].value /= 159.23

        # communication with the pi-puck extension FT903 address
        mapping = [2, 1, 0]
        for i in range(3):
            ledName = 'pi-puck led %d' % i
            if self.devices[ledName].changed:
                ledValue = ((0x01 if ((self.devices[ledName].value >> 16) & 0xFF) > 0 else 0) |
                            (0x02 if ((self.devices[ledName].value >> 8) & 0xFF) > 0 else 0) |
                            (0x04 if (self.devices[ledName].value & 0xFF) > 0 else 0))
                self.busFT903.write_byte_data(FT903_ADDR, mapping[i], ledValue)
                self.devices[ledName].changed = False
        # communication with i2c bus with ground sensors board address
        groundSensorsEnabled = False
        for name in DistanceSensor.groundNames:
            if self.devices[name].getSamplingPeriod() > 0:
                groundSensorsEnabled = True
                break
        if groundSensorsEnabled:
            read = i2c_msg.read(GROUND_ADDR, GROUND_SENSORS_SIZE)
            try:
                self.bus.i2c_rdwr(read)
            except:
                return
            groundData = list(read)
            for i in range(3):
                self.devices[DistanceSensor.groundNames[i]].value = (
                    groundData[i * 2] << 8) + groundData[i * 2 + 1]
        # communication with the i2c bus with the extension board address
        # if self.devices['accelerometer'].getSamplingPeriod() > 0 or self.devices['gyro'].getSamplingPeriod() > 0:
        # imuRead = i2c_msg.read(IMU_ADDR, IMU_SIZE)
        # self.bus.i2c_rdwr(imuRead)
        # imuData = list(imuRead)
        # if len(imuData) != IMU_SIZE:
        #     sys.exit('Wrond IMU data size.')
        # print(imuData)

    def getAccelerometer(self, name):
        if name in self.devices and isinstance(self.devices[name], Accelerometer):
            return self.devices[name]
        print('No Accelerometer device named "%s"\n' % name)
        return None

    def getDistanceSensor(self, name):
        if name in self.devices and isinstance(self.devices[name], DistanceSensor):
            return self.devices[name]
        print('No DistanceSensor device named "%s"\n' % name)
        return None

    def getLED(self, name):
        if name in self.devices and isinstance(self.devices[name], LED):
            return self.devices[name]
        print('No LED device named "%s"\n' % name)
        return None

    def getLightSensor(self, name):
        if name in self.devices and isinstance(self.devices[name], LightSensor):
            return self.devices[name]
        print('No LightSensor device named "%s"\n' % name)
        return None

    def getMotor(self, name):
        if name in self.devices and isinstance(self.devices[name], Motor):
            return self.devices[name]
        print('No Motor device named "%s"\n' % name)
        return None

    def getPositionSensor(self, name):
        if name in self.devices and isinstance(self.devices[name], PositionSensor):
            return self.devices[name]
        print('No PositionSensor device named "%s"\n' % name)
        return None

    def getName(self):
        return 'e-puck'

    def getTime(self):
        return self.time

    def getSupervisor(self):
        return False

    def getSynchronization(self):
        return False

    def getBasicTimeStep(self):
        return 32

    def getNumberOfDevices(self):
        return len(self.devices)

    def getDeviceByIndex(self, index):
        return self.devices[index]

    def getMode(self):
        return Robot.MODE_CROSS_COMPILATION

    def getCustomData(self):
        return self.customData

    def setCustomData(self, data):
        self.customData = data
