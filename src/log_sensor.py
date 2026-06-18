import argparse
import csv
import importlib
import os
import select
import sys
import time
from datetime import datetime


PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

# This lets Python import from the main project folder.
sys.path.insert(0, PROJECT_ROOT)


def read_terminal_command():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip().lower()

    return None


def load_sensor(sensor_name):
    module_name = f"sensors.imu.{sensor_name}.code.{sensor_name}_sensor"
    module = importlib.import_module(module_name)
    return module.Sensor()


def make_data_folder(sensor_name):
    data_folder = os.path.join(
        PROJECT_ROOT,
        "sensors",
        "imu",
        sensor_name,
        "data",
        "raw"
    )

    os.makedirs(data_folder, exist_ok=True)

    return data_folder


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "sensor",
        nargs="?",
        default="bno085",
        help="Sensor name. Example: bno085, adis16470, vn200"
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=25.0,
        help="Sample rate in Hz"
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Optional test duration in seconds"
    )

    args = parser.parse_args()

    sensor = load_sensor(args.sensor)
    sensor.connect()

    data_folder = make_data_folder(args.sensor)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_file = os.path.join(data_folder, f"{sensor.name}_log_{timestamp}.csv")

    sample_period = 1.0 / args.rate

    fieldnames = [
        "sample",
        "time_s",
        "sensor",
    ] + sensor.csv_fields

    print()
    print("Starting generic sensor logger...")
    print(f"Sensor: {sensor.name}")
    print(f"Sample rate: {args.rate} Hz")
    print(f"Saving to: {output_file}")
    print()
    print("Commands:")
    print("  z + ENTER = zero current position")
    print("  q + ENTER = stop logging")
    print("  CTRL+C    = stop logging")
    print()

    sample = 0
    start_time = time.monotonic()
    next_sample_time = start_time

    with open(output_file, mode="w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        try:
            while True:
                command = read_terminal_command()

                if command == "z":
                    if sensor.zero():
                        print("Zero reset.")
                    else:
                        print("Could not zero yet. No sensor data.")

                elif command == "q":
                    print("Stopping logger.")
                    break

                now = time.monotonic()

                if now < next_sample_time:
                    time.sleep(next_sample_time - now)

                current_time = time.monotonic()
                elapsed_time = current_time - start_time
                next_sample_time += sample_period

                if args.duration is not None and elapsed_time >= args.duration:
                    print("Duration reached.")
                    break

                try:
                    sensor_data = sensor.read()
                except Exception as error:
                    print(f"Sensor read error: {error}")
                    time.sleep(0.1)
                    continue

                if sensor_data is None:
                    continue

                row = {
                    "sample": sample,
                    "time_s": elapsed_time,
                    "sensor": sensor.name,
                }

                row.update(sensor_data)

                writer.writerow(row)

                if sample % max(1, int(args.rate)) == 0:
                    csv_file.flush()

                    roll = sensor_data.get("roll_deg", 0.0)
                    pitch = sensor_data.get("pitch_deg", 0.0)
                    yaw = sensor_data.get("yaw_deg", 0.0)

                    print(
                        f"t={elapsed_time:7.2f}s | "
                        f"roll={roll:7.2f} | "
                        f"pitch={pitch:7.2f} | "
                        f"yaw={yaw:7.2f} | "
                        f"samples={sample}"
                    )

                sample += 1

        except KeyboardInterrupt:
            print()
            print("Keyboard stop.")

        csv_file.flush()

    print()
    print("Logging finished.")
    print(f"Total samples: {sample}")
    print(f"Saved file: {output_file}")


if __name__ == "__main__":
    main()
