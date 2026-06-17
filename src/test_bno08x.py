import csv
import math
import os
import time
from datetime import datetime

import board
import busio
import numpy as np

from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ROTATION_VECTOR,
)

from adafruit_bno08x.i2c import BNO08X_I2C


SAMPLE_RATE_HZ = 20
SAMPLE_PERIOD_SECONDS = 1.0 / SAMPLE_RATE_HZ

DATA_FOLDER = "/home/atmccabe/Floating_Wind_Turbine/data"


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


def main():
    os.makedirs(DATA_FOLDER, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_file = os.path.join(DATA_FOLDER, f"bno08x_log_{timestamp}.csv")

    print("Starting BNO08x data logger...")
    print(f"Sample rate: {SAMPLE_RATE_HZ} Hz")
    print(f"Saving data to: {output_file}")
    print("First valid sensor position becomes the zero reference.")
    print("Press CTRL+C to stop.")
    print()

    i2c = busio.I2C(board.SCL, board.SDA)
    bno = BNO08X_I2C(i2c)

    bno.enable_feature(BNO_REPORT_ACCELEROMETER)
    bno.enable_feature(BNO_REPORT_GYROSCOPE)
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

    zero_rotation_matrix = None
    sample_number = 0
    start_time = time.monotonic()
    next_sample_time = start_time

    with open(output_file, mode="w", newline="") as csv_file:
        writer = csv.writer(csv_file)

        writer.writerow([
            "sample",
            "time_s",

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

            "roll_deg_absolute",
            "pitch_deg_absolute",
            "yaw_deg_absolute",

            "roll_deg_zeroed",
            "pitch_deg_zeroed",
            "yaw_deg_zeroed",
        ])

        try:
            while True:
                now = time.monotonic()

                if now < next_sample_time:
                    time.sleep(next_sample_time - now)

                current_time = time.monotonic()
                elapsed_time = current_time - start_time
                next_sample_time += SAMPLE_PERIOD_SECONDS

                accel_x, accel_y, accel_z = bno.acceleration
                gyro_x, gyro_y, gyro_z = bno.gyro
                quat_i, quat_j, quat_k, quat_real = bno.quaternion

                if quat_i == 0 and quat_j == 0 and quat_k == 0 and quat_real == 0:
                    continue

                current_rotation_matrix = quaternion_to_rotation_matrix(
                    quat_i,
                    quat_j,
                    quat_k,
                    quat_real
                )

                if zero_rotation_matrix is None:
                    zero_rotation_matrix = current_rotation_matrix.copy()
                    print("Zero reference set.")

                absolute_roll, absolute_pitch, absolute_yaw = rotation_matrix_to_euler(
                    current_rotation_matrix
                )

                relative_rotation_matrix = zero_rotation_matrix.T @ current_rotation_matrix

                zeroed_roll, zeroed_pitch, zeroed_yaw = rotation_matrix_to_euler(
                    relative_rotation_matrix
                )

                writer.writerow([
                    sample_number,
                    f"{elapsed_time:.4f}",

                    f"{accel_x:.6f}",
                    f"{accel_y:.6f}",
                    f"{accel_z:.6f}",

                    f"{gyro_x:.6f}",
                    f"{gyro_y:.6f}",
                    f"{gyro_z:.6f}",

                    f"{quat_i:.8f}",
                    f"{quat_j:.8f}",
                    f"{quat_k:.8f}",
                    f"{quat_real:.8f}",

                    f"{absolute_roll:.4f}",
                    f"{absolute_pitch:.4f}",
                    f"{absolute_yaw:.4f}",

                    f"{zeroed_roll:.4f}",
                    f"{zeroed_pitch:.4f}",
                    f"{zeroed_yaw:.4f}",
                ])

                sample_number += 1

                if sample_number % SAMPLE_RATE_HZ == 0:
                    csv_file.flush()
                    print(
                        f"t={elapsed_time:7.2f}s | "
                        f"roll={zeroed_roll:7.2f} deg | "
                        f"pitch={zeroed_pitch:7.2f} deg | "
                        f"yaw={zeroed_yaw:7.2f} deg"
                    )

        except KeyboardInterrupt:
            csv_file.flush()
            print()
            print("Logging stopped.")
            print(f"Total samples recorded: {sample_number}")
            print(f"Saved file: {output_file}")


if __name__ == "__main__":
    main()
