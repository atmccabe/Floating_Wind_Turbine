import glob
import os

import pandas as pd
import matplotlib.pyplot as plt


DATA_FOLDER = "/home/atmccabe/Floating_Wind_Turbine/data"
PLOT_FOLDER = "/home/atmccabe/Floating_Wind_Turbine/data/plots"


def find_newest_bno08x_csv():
    patterns = [
        os.path.join(DATA_FOLDER, "bno08x_web_log_*.csv"),
        os.path.join(DATA_FOLDER, "bno08x_log_*.csv"),
    ]

    files = []

    for pattern in patterns:
        files.extend(glob.glob(pattern))

    if not files:
        raise FileNotFoundError("No BNO08x CSV files found in the data folder.")

    newest_file = max(files, key=os.path.getmtime)
    return newest_file


def save_orientation_plot(df, output_path):
    plt.figure(figsize=(12, 6))

    plt.plot(df["time_s"], df["roll_deg_zeroed"], label="Roll")
    plt.plot(df["time_s"], df["pitch_deg_zeroed"], label="Pitch")
    plt.plot(df["time_s"], df["yaw_deg_zeroed"], label="Yaw")

    plt.title("BNO08x Orientation vs Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (degrees)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(output_path, dpi=200)
    plt.close()


def save_acceleration_plot(df, output_path):
    plt.figure(figsize=(12, 6))

    plt.plot(df["time_s"], df["accel_x_mps2"], label="Accel X")
    plt.plot(df["time_s"], df["accel_y_mps2"], label="Accel Y")
    plt.plot(df["time_s"], df["accel_z_mps2"], label="Accel Z")

    plt.title("BNO08x Acceleration vs Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s²)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(output_path, dpi=200)
    plt.close()


def save_gyro_plot(df, output_path):
    plt.figure(figsize=(12, 6))

    plt.plot(df["time_s"], df["gyro_x_radps"], label="Gyro X")
    plt.plot(df["time_s"], df["gyro_y_radps"], label="Gyro Y")
    plt.plot(df["time_s"], df["gyro_z_radps"], label="Gyro Z")

    plt.title("BNO08x Gyroscope vs Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Angular velocity (rad/s)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(output_path, dpi=200)
    plt.close()


def main():
    os.makedirs(PLOT_FOLDER, exist_ok=True)

    csv_file = find_newest_bno08x_csv()

    print(f"Using CSV file: {csv_file}")

    df = pd.read_csv(csv_file)

    print()
    print("Columns found:")
    print(df.columns.tolist())
    print()
    print(f"Number of samples: {len(df)}")
    print(f"Start time: {df['time_s'].iloc[0]:.3f} s")
    print(f"End time:   {df['time_s'].iloc[-1]:.3f} s")

    base_name = os.path.splitext(os.path.basename(csv_file))[0]

    orientation_plot = os.path.join(PLOT_FOLDER, f"{base_name}_orientation.png")
    acceleration_plot = os.path.join(PLOT_FOLDER, f"{base_name}_acceleration.png")
    gyro_plot = os.path.join(PLOT_FOLDER, f"{base_name}_gyro.png")

    save_orientation_plot(df, orientation_plot)
    save_acceleration_plot(df, acceleration_plot)
    save_gyro_plot(df, gyro_plot)

    print()
    print("Saved plots:")
    print(orientation_plot)
    print(acceleration_plot)
    print(gyro_plot)


if __name__ == "__main__":
    main()
