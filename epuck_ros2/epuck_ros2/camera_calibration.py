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

# A temporary, very simple, solution to calibrate camera.
# It should be delete as soon as `camera_calibration` is ready for ROS2.


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np


CHECKERBOARD_WIDTH = 9
CHECKERBOARD_HEIGHT = 6
RECTANGLE_WIDTH = 0.0288


class CameraCalibration(Node):
    def __init__(self, name, args=None):
        super().__init__(name)

        self.create_subscription(
            Image, '/image_raw', self.on_image_received, 1)

        self.expected_points = np.zeros(
            (CHECKERBOARD_HEIGHT*CHECKERBOARD_WIDTH, 3), np.float32)
        self.expected_points[:, :2] = np.mgrid[0:CHECKERBOARD_WIDTH,
                                               0:CHECKERBOARD_HEIGHT].T.reshape(-1, 2)
        self.expected_points = self.expected_points * RECTANGLE_WIDTH

        self.object_points = []
        self.image_points = []

    def show_calibration_result(self):
        N_OK = len(self.object_points)
        K = np.zeros((3, 3))
        D = np.zeros((4, 1))
        rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
        tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]

        retval, K, D, rvecs, tvecs = \
            cv2.calibrateCamera(
                np.array(self.object_points),
                np.array(self.image_points),
                self.gray.shape[::-1],
                K,
                D,
                rvecs,
                tvecs
            )

        print('retval = %s' % retval)
        print('K = %s' % K)
        print('D = %s' % D)
        print('rvecs = %s' % rvecs)
        print('tvecs = %s' % tvecs)

    def on_image_received(self, msg):
        img = np.array(msg.data).reshape((480, 640, 3))
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        self.gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(self.gray,
                                                 (CHECKERBOARD_WIDTH,
                                                     CHECKERBOARD_HEIGHT),
                                                 cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_NORMALIZE_IMAGE
                                                 )
        if ret:
            self.object_points.append(self.expected_points)
            self.image_points.append(corners)

            for corner in corners:
                img = cv2.circle(
                    img, (corner[0][0], corner[0][1]), 10, (255, 0, 0))

            print('Success #%d' % len(self.object_points))

            if len(self.object_points) >= 1:
                self.show_calibration_result()

        cv2.imshow('image', img)
        k = cv2.waitKey(300)
        if k == 27:
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    calib = CameraCalibration('camera_calibration', args=args)
    rclpy.spin(calib)
    calib.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
