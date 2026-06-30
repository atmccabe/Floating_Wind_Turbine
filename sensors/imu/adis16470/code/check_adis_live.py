import os
import sys
import time
import importlib


PROJECT_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../../../..")
)
sys.path.insert(0, PROJECT_ROOT)


def load_sensor():
    module = importlib.import_module(
        "sensors.imu.adis16470.code.adis16470_sensor"
    )
    return module.Sensor()


sensor = load_sensor()

print("Connecting to ADIS16470...")
sensor.connect()
print("Connected.")
print("Move the sensor. Press CTRL+C to stop.\n")

try:
    while True:
        data = sensor.read()

        print(
            f"roll={data['roll_deg']:8.2f}  "
            f"pitch={data['pitch_deg']:8.2f}  "
            f"yaw={data['yaw_deg']:8.2f}  "
            f"ax={data['accel_x_g']:7.3f}g  "
            f"ay={data['accel_y_g']:7.3f}g  "
            f"az={data['accel_z_g']:7.3f}g"
        )

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopped.")
