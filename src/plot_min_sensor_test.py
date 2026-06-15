import csv
from pathlib import Path

import matplotlib.pyplot as plt


CSV_FILE = Path("data/min_sensor_test.csv")
PLOT_FILE = Path("data/min_sensor_test_plot.png")


def main():
    time_s = []
    pitch_deg = []
    roll_deg = []
    yaw_deg = []

    with open(CSV_FILE, "r") as file:
        reader = csv.DictReader(file)

        for row in reader:
            time_s.append(float(row["time_s"]))
            pitch_deg.append(float(row["pitch_deg"]))
            roll_deg.append(float(row["roll_deg"]))
            yaw_deg.append(float(row["yaw_deg"]))

    plt.figure(figsize=(10, 6))
    plt.plot(time_s, pitch_deg, label="Pitch")
    plt.plot(time_s, roll_deg, label="Roll")
    plt.plot(time_s, yaw_deg, label="Yaw")

    plt.xlabel("Time (s)")
    plt.ylabel("Angle (degrees)")
    plt.title("Minimal Sensor Test Data")
    plt.legend()
    plt.grid(True)

    plt.savefig(PLOT_FILE, dpi=150)
    plt.close()

    print(f"Plot saved to {PLOT_FILE}")


if __name__ == "__main__":
    main()
