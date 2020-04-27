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


"""
Parse I2C data.

This scripts supposed to allow debugging of
communication between the ROS controller and I2C.
"""

import time


prev_data = []


def _get_int16(bytearr):
    val = (bytearr[0] & 0x00FF) | ((bytearr[1] << 8) & 0xFF00)
    if val > 2**15:
        val -= 2**16
    return val


def _get_params(buffer):
    params = {}
    params['left_speed'] = _get_int16(buffer[0:2])
    params['right_speed'] = _get_int16(buffer[2:4])
    return params


while True:
    data = []
    with open('/tmp/dev/i2c-4_write', 'rb') as f:
        data = list(f.read())

    if len(data) > 0 and prev_data != data:
        params = _get_params(data)
        print(params)

        prev_data = data.copy()

    time.sleep(0.01)
