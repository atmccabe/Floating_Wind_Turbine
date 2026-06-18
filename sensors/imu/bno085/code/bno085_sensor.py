import math

import board
import busio
import numpy as np

from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C


CSV_FIELDS = [
    "accel_x_mps2",
    "accel_y_mps2",
    "accel_z_mps2",

    "gyro_x_radps",
    "gyro_y_radps",
    "gyro_z_radps",

    "quat_i",
    "quat_j",
    "quat_k",
    "quat_real",

    "roll_deg",
    "pitch_deg",
    "yaw_deg",
]


def quaternion_to_rotation_matrix(i, j, k, real):
    x = i
    y = j
    z = k
    w = real

    return np.array([
        [1 - 2 * (y * y + z * z),     2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),         1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),         2 * (y * z + x * w),     1 - 2 * (x * x + y * y)]
    ])


def rotation_matrix_to_euler(rotation_matrix):
    r = rotation_matrix

    roll = math.atan2(r[2, 1], r[2, 2])
    pitch = math.atan2(-r[2, 0], math.sqrt(r[2, 1] ** 2 + r[2, 2] ** 2))
    yaw = math.atan2(r[1, 0], r[0, 0])

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class Sensor:
    name = "bno085"
    csv_fields = CSV_FIELDS

    def __init__(self):
        self.bno = None
        self.zero_rotation_matrix = None
        self.latest_rotation_matrix = None

    def connect(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = BNO08X_I2C(i2c)

        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

        print("BNO085/BNO08x connected.")

    def zero(self):
        if self.latest_rotation_matrix is None:
            return False

        self.zero_rotation_matrix = self.latest_rotation_matrix.copy()
        return True

    def read(self):
        accel_x, accel_y, accel_z = self.bno.acceleration
        gyro_x, gyro_y, gyro_z = self.bno.gyro
        quat_i, quat_j, quat_k, quat_real = self.bno.quaternion

        if quat_i == 0 and quat_j == 0 and quat_k == 0 and quat_real == 0:
            return None

        current_rotation_matrix = quaternion_to_rotation_matrix(
            quat_i,
            quat_j,
            quat_k,
            quat_real
        )

        self.latest_rotation_matrix = current_rotation_matrix.copy()

        if self.zero_rotation_matrix is None:
            self.zero_rotation_matrix = current_rotation_matrix.copy()
            print("Initial zero set.")

        relative_rotation_matrix = self.zero_rotation_matrix.T @ current_rotation_matrix

        roll, pitch, yaw = rotation_matrix_to_euler(relative_rotation_matrix)

        return {
            "accel_x_mps2": accel_x,
            "accel_y_mps2": accel_y,
            "accel_z_mps2": accel_z,

            "gyro_x_radps": gyro_x,
            "gyro_y_radps": gyro_y,
            "gyro_z_radps": gyro_z,

            "quat_i": quat_i,
            "quat_j": quat_j,
            "quat_k": quat_k,
            "quat_real": quat_real,

            "roll_deg": roll,
            "pitch_deg": pitch,
            "yaw_deg": yaw,
        }
