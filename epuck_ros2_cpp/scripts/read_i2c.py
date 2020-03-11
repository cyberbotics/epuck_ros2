"""
This scripts supposed to allow debugging of communication between the ROS controller and I2C
"""

import time


prev_data = []


def get_int16(bytearr):
    val = (bytearr[0] & 0x00FF) | ((bytearr[1] << 8) & 0xFF00)
    if val > 2**15:
        val -= 2**16
    return val


def get_params(buffer):
    params = {}
    params['left_speed'] = get_int16(buffer[0:2])
    params['right_speed'] = get_int16(buffer[2:4])
    return params


while True:
    data = []
    with open('/tmp/dev/i2c-4_write', 'rb') as f:
        data = list(f.read())

    if len(data) > 0 and prev_data != data:
        params = get_params(data)
        print(params)

        prev_data = data.copy()


    time.sleep(0.01)
