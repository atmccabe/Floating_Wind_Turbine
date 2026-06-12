import csv
import math
import random
import time
from datetime import datetime

output_file = "data/min_sensor_test.csv"

print("Starting minimal sensor test...")
print(f"Saving data to {output_file}")

with open(output_file, "w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["timestamp", "time_s", "pitch_deg", "roll_deg", "yaw_deg"])

    start_time = time.time()

    for i in range(10):
        current_time = time.time() - start_time

        pitch_deg = 5 * math.sin(current_time)
        roll_deg = random.uniform(-1, 1)
        yaw_deg = (i * 10) % 360

        writer.writerow([
            datetime.now().isoformat(timespec="seconds"),
            round(current_time, 3),
            round(pitch_deg, 3),
            round(roll_deg, 3),
            round(yaw_deg, 3)
        ])

        print(f"t={current_time:.2f}s | pitch={pitch_deg:.2f} | roll={roll_deg:.2f} | yaw={yaw_deg:.2f}")

        time.sleep(0.5)

print("Minimal sensor test complete.")
