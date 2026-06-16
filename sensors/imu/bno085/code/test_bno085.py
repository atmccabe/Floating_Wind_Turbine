import csv
import time # Measures how long test was running and reading.
from datetime import datetime # Gives current date and time.
from pathlib import Path # Makes file path cleaner.

import board # Lets python know the raspberry pi pin names.
import busio # Creates I2C communication line.

from adafruit_bno08x import ( # Specific to the BNO085 sensor.
	BNO_REPORT_ACCELEROMETER,
	BNO_REPORT_GYROSCOPE,
	BNO_REPORT_ROTATION_VECTOR)

from adafruit_bno08x.i2c import BNO08X_I2C # Sensor class for I2C.

# File saving setup
script_dir = Path(__file__).resolve().parent
sensor_dir = script_dir.parent

data_dir = sensor_dir / "data" / "raw"
data_dir.mkdir(parents=True, exist_ok=True)

timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
filename = data_dir / f"bno085_test_{timestamp}.csv"

# Connection setup
i2c = busio.I2C(board.SCL, board.SDA)
bno =  BNO08X_I2C(i2c)

# Needed data
bno.enable_featue(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

# Testing 
sample_rate = 20 #Hz
duration = 30 #s
dt = 1/sample_rate

print("Starting BNO085 real sensor test")
print(f"Saving data to: {filename}")
print("Press CTRL+C to stop early.")

# Opening CSV to write 
start_time = time.time()

with open(filename, "w", newline="") as file:
    writer = csv.writer(file)

    writer.writerow([
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
        "quat_real"
    ])

# Sensor loop
    try:
        while True:
            t = time.time() - start_time

            if t > duration_s:
                break

            accel_x, accel_y, accel_z = bno.acceleration
            gyro_x, gyro_y, gyro_z = bno.gyro
            quat_i, quat_j, quat_k, quat_real = bno.quaternion

            writer.writerow([
                round(t, 4),
                accel_x,
                accel_y,
                accel_z,
                gyro_x,
                gyro_y,
                gyro_z,
                quat_i,
                quat_j,
                quat_k,
                quat_real
            ])

            print(
                f"t={t:.2f}s | "
                f"accel=({accel_x:.2f}, {accel_y:.2f}, {accel_z:.2f}) m/s^2 | "
                f"gyro=({gyro_x:.2f}, {gyro_y:.2f}, {gyro_z:.2f}) rad/s"
            )

            time.sleep(dt)

    except KeyboardInterrupt:
        print("Stopped early by user.")

print("Done.")
