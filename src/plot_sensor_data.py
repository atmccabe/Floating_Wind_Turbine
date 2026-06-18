import argparse
import glob
import os

import pandas as pd
import matplotlib.pyplot as plt


PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def get_sensor_data_folder(sensor_name):
    return os.path.join(
        PROJECT_ROOT,
        "sensors",
        "imu",
        sensor_name,
        "data",
        "raw"
    )


def get_sensor_plot_folder(sensor_name):
    return os.path.join(
        PROJECT_ROOT,
        "sensors",
        "imu",
        sensor_name,
        "plots"
    )


def find_newest_csv(sensor_name):
    data_folder = get_sensor_data_folder(sensor_name)
    pattern = os.path.join(data_folder, f"{sensor_name}_log_*.csv")

    files = glob.glob(pattern)

    if not files:
        raise FileNotFoundError(f"No CSV files found for sensor: {sensor_name}")

    return max(files, key=os.path.getmtime)


def get_column(df, possible_names):
    for name in possible_names:
        if name in df.columns:
            return name

    return None


def save_orientation_plot(df, output_path):
    roll_col = get_column(df, ["roll_deg", "roll_deg_zeroed"])
    pitch_col = get_column(df, ["pitch_deg", "pitch_deg_zeroed"])
    yaw_col = get_column(df, ["yaw_deg", "yaw_deg_zeroed"])

    if roll_col is None or pitch_col is None or yaw_col is None:
        print("Skipping orientation plot. Missing roll/pitch/yaw columns.")
        return

    plt.figure(figsize=(12, 6))

    plt.plot(df["time_s"], df[roll_col], label="Roll")
    plt.plot(df["time_s"], df[pitch_col], label="Pitch")
    plt.plot(df["time_s"], df[yaw_col], label="Yaw")

    plt.title("Sensor Orientation vs Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (degrees)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(output_path, dpi=200)
    plt.close()

    print(f"Saved: {output_path}")


def save_acceleration_plot(df, output_path):
    needed_columns = [
        "accel_x_mps2",
        "accel_y_mps2",
        "accel_z_mps2",
    ]

    for column in needed_columns:
        if column not in df.columns:
            print("Skipping acceleration plot. Missing acceleration columns.")
            return

    plt.figure(figsize=(12, 6))

    plt.plot(df["time_s"], df["accel_x_mps2"], label="Accel X")
    plt.plot(df["time_s"], df["accel_y_mps2"], label="Accel Y")
    plt.plot(df["time_s"], df["accel_z_mps2"], label="Accel Z")

    plt.title("Sensor Acceleration vs Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s²)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(output_path, dpi=200)
    plt.close()

    print(f"Saved: {output_path}")


def save_gyro_plot(df, output_path):
    needed_columns = [
        "gyro_x_radps",
        "gyro_y_radps",
        "gyro_z_radps",
    ]

    for column in needed_columns:
        if column not in df.columns:
            print("Skipping gyro plot. Missing gyro columns.")
            return

    plt.figure(figsize=(12, 6))

    plt.plot(df["time_s"], df["gyro_x_radps"], label="Gyro X")
    plt.plot(df["time_s"], df["gyro_y_radps"], label="Gyro Y")
    plt.plot(df["time_s"], df["gyro_z_radps"], label="Gyro Z")

    plt.title("Sensor Gyroscope vs Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Angular velocity (rad/s)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(output_path, dpi=200)
    plt.close()

    print(f"Saved: {output_path}")


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "sensor",
        nargs="?",
        default="bno085",
        help="Sensor name. Example: bno085, adis16470, vn200"
    )

    args = parser.parse_args()

    plot_folder = get_sensor_plot_folder(args.sensor)
    os.makedirs(plot_folder, exist_ok=True)

    csv_file = find_newest_csv(args.sensor)
    df = pd.read_csv(csv_file)

    base_name = os.path.splitext(os.path.basename(csv_file))[0]

    print(f"Using CSV: {csv_file}")
    print(f"Samples: {len(df)}")
    print(f"Columns: {df.columns.tolist()}")
    print()

    orientation_plot = os.path.join(plot_folder, f"{base_name}_orientation.png")
    acceleration_plot = os.path.join(plot_folder, f"{base_name}_acceleration.png")
    gyro_plot = os.path.join(plot_folder, f"{base_name}_gyro.png")

    save_orientation_plot(df, orientation_plot)
    save_acceleration_plot(df, acceleration_plot)
    save_gyro_plot(df, gyro_plot)


if __name__ == "__main__":
    main()

