import csv
import math
import os
import threading
import time
from datetime import datetime

import board
import busio
import numpy as np
from flask import Flask, jsonify, send_from_directory, render_template_string

from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C


IMG_DIR = "/home/atmccabe/Floating_Wind_Turbine/img"
DATA_DIR = "/home/atmccabe/Floating_Wind_Turbine/data"

TOP_IMAGE_NAME = "bno08x_top.png"
BOTTOM_IMAGE_NAME = "bno08x_bottom.png"

SAMPLE_RATE_HZ = 25
SAMPLE_PERIOD_SECONDS = 1.0 / SAMPLE_RATE_HZ

app = Flask(__name__)

data_lock = threading.Lock()

latest_data = {
    "sample": 0,
    "time_s": 0.0,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "accel_x": 0.0,
    "accel_y": 0.0,
    "accel_z": 0.0,
    "gyro_x": 0.0,
    "gyro_y": 0.0,
    "gyro_z": 0.0,
    "log_file": "",
}

zero_rotation_matrix = None
latest_rotation_matrix = None
latest_csv_path = None


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


def sensor_and_logger_loop():
    global latest_data
    global zero_rotation_matrix
    global latest_rotation_matrix
    global latest_csv_path

    os.makedirs(DATA_DIR, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    latest_csv_path = os.path.join(DATA_DIR, f"bno08x_web_log_{timestamp}.csv")

    print("Starting BNO08x web dashboard + data logger...")
    print(f"Sample rate: {SAMPLE_RATE_HZ} Hz")
    print(f"Saving CSV to: {latest_csv_path}")

    i2c = busio.I2C(board.SCL, board.SDA)
    bno = BNO08X_I2C(i2c)

    bno.enable_feature(BNO_REPORT_ACCELEROMETER)
    bno.enable_feature(BNO_REPORT_GYROSCOPE)
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

    start_time = time.monotonic()
    next_sample_time = start_time
    sample = 0

    with open(latest_csv_path, mode="w", newline="") as csv_file:
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

            "roll_deg_zeroed",
            "pitch_deg_zeroed",
            "yaw_deg_zeroed",
        ])

        while True:
            now = time.monotonic()

            if now < next_sample_time:
                time.sleep(next_sample_time - now)

            current_time = time.monotonic()
            elapsed_time = current_time - start_time
            next_sample_time += SAMPLE_PERIOD_SECONDS

            try:
                accel_x, accel_y, accel_z = bno.acceleration
                gyro_x, gyro_y, gyro_z = bno.gyro
                quat_i, quat_j, quat_k, quat_real = bno.quaternion
            except Exception as error:
                print(f"Sensor read error: {error}")
                time.sleep(0.1)
                continue

            if quat_i == 0 and quat_j == 0 and quat_k == 0 and quat_real == 0:
                continue

            current_rotation_matrix = quaternion_to_rotation_matrix(
                quat_i,
                quat_j,
                quat_k,
                quat_real
            )

            latest_rotation_matrix = current_rotation_matrix.copy()

            if zero_rotation_matrix is None:
                zero_rotation_matrix = current_rotation_matrix.copy()
                print("Initial zero set.")

            relative_rotation_matrix = zero_rotation_matrix.T @ current_rotation_matrix
            roll, pitch, yaw = rotation_matrix_to_euler(relative_rotation_matrix)

            writer.writerow([
                sample,
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

                f"{roll:.4f}",
                f"{pitch:.4f}",
                f"{yaw:.4f}",
            ])

            if sample % SAMPLE_RATE_HZ == 0:
                csv_file.flush()
                print(
                    f"t={elapsed_time:7.2f}s | "
                    f"roll={roll:7.2f} | "
                    f"pitch={pitch:7.2f} | "
                    f"yaw={yaw:7.2f} | "
                    f"samples={sample}"
                )

            with data_lock:
                latest_data = {
                    "sample": sample,
                    "time_s": elapsed_time,

                    "roll": roll,
                    "pitch": pitch,
                    "yaw": yaw,

                    "accel_x": accel_x,
                    "accel_y": accel_y,
                    "accel_z": accel_z,

                    "gyro_x": gyro_x,
                    "gyro_y": gyro_y,
                    "gyro_z": gyro_z,

                    "log_file": os.path.basename(latest_csv_path),
                }

            sample += 1


HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>BNO08x Live Viewer + Data Logger</title>

    <style>
        body {
            margin: 0;
            background: #111;
            color: white;
            font-family: Arial, sans-serif;
            text-align: center;
        }

        h1 {
            margin-top: 18px;
            margin-bottom: 5px;
        }

        .numbers {
            font-size: 24px;
            margin: 8px;
        }

        .small {
            color: #aaa;
            font-size: 16px;
            margin: 5px;
        }

        .scene {
            width: 700px;
            height: 500px;
            margin: 15px auto;
            perspective: 900px;
            border: 2px solid #444;
            border-radius: 20px;
            background: #1b1b1b;
            display: flex;
            align-items: center;
            justify-content: center;
        }

        .board {
            width: 520px;
            height: 350px;
            position: relative;
            transform-style: preserve-3d;
            transition: transform 0.025s linear;
        }

        .face {
            position: absolute;
            width: 520px;
            height: 350px;
            left: 0;
            top: 0;
            background-size: contain;
            background-repeat: no-repeat;
            background-position: center;
            backface-visibility: hidden;
        }

        .front {
            background-image: url('/img/bno08x_top.png');
            transform: translateZ(8px);
        }

        .back {
            background-image: url('/img/bno08x_bottom.png');
            transform: rotateY(180deg) translateZ(8px);
        }

        .front-arrow {
            position: absolute;
            left: 240px;
            top: 40px;
            width: 120px;
            height: 8px;
            background: red;
            transform: translateZ(25px);
            border-radius: 4px;
        }

        .front-arrow::after {
            content: "";
            position: absolute;
            right: -20px;
            top: -8px;
            border-left: 22px solid red;
            border-top: 12px solid transparent;
            border-bottom: 12px solid transparent;
        }

        button, a.button {
            font-size: 18px;
            padding: 10px 18px;
            margin: 8px;
            border-radius: 10px;
            border: none;
            cursor: pointer;
            background: #eee;
            color: black;
            text-decoration: none;
            display: inline-block;
        }

        .data-box {
            margin: 10px auto;
            width: 720px;
            background: #1b1b1b;
            border: 1px solid #444;
            border-radius: 14px;
            padding: 12px;
            text-align: left;
            font-family: monospace;
            font-size: 16px;
        }
    </style>
</head>

<body>
    <h1>BNO08x Live Orientation + CSV Logger</h1>

    <div class="numbers">
        Roll: <span id="roll">0.0</span>° |
        Pitch: <span id="pitch">0.0</span>° |
        Yaw: <span id="yaw">0.0</span>°
    </div>

    <div class="small">
        Time: <span id="time">0.0</span> s |
        Samples: <span id="sample">0</span> |
        CSV: <span id="logfile">waiting...</span>
    </div>

    <button onclick="zeroSensor()">Zero current position</button>
    <a class="button" href="/download_csv">Download CSV</a>

    <div class="scene">
        <div class="board" id="board">
            <div class="face front"></div>
            <div class="face back"></div>
            <div class="front-arrow"></div>
        </div>
    </div>

    <div class="data-box">
        Accel m/s²:
        x=<span id="accel_x">0.0</span>,
        y=<span id="accel_y">0.0</span>,
        z=<span id="accel_z">0.0</span>
        <br>
        Gyro rad/s:
        x=<span id="gyro_x">0.0</span>,
        y=<span id="gyro_y">0.0</span>,
        z=<span id="gyro_z">0.0</span>
    </div>

    <script>
        async function updateData() {
            const response = await fetch("/data");
            const data = await response.json();

            document.getElementById("roll").textContent = data.roll.toFixed(1);
            document.getElementById("pitch").textContent = data.pitch.toFixed(1);
            document.getElementById("yaw").textContent = data.yaw.toFixed(1);

            document.getElementById("time").textContent = data.time_s.toFixed(2);
            document.getElementById("sample").textContent = data.sample;
            document.getElementById("logfile").textContent = data.log_file;

            document.getElementById("accel_x").textContent = data.accel_x.toFixed(3);
            document.getElementById("accel_y").textContent = data.accel_y.toFixed(3);
            document.getElementById("accel_z").textContent = data.accel_z.toFixed(3);

            document.getElementById("gyro_x").textContent = data.gyro_x.toFixed(3);
            document.getElementById("gyro_y").textContent = data.gyro_y.toFixed(3);
            document.getElementById("gyro_z").textContent = data.gyro_z.toFixed(3);

            const board = document.getElementById("board");

            board.style.transform =
                `rotateZ(${data.yaw}deg) rotateY(${data.pitch}deg) rotateX(${data.roll}deg)`;
        }

        async function zeroSensor() {
            await fetch("/zero", { method: "POST" });
        }

        setInterval(updateData, 40);
    </script>
</body>
</html>
"""


@app.route("/")
def index():
    return render_template_string(HTML_PAGE)


@app.route("/img/<path:filename>")
def images(filename):
    return send_from_directory(IMG_DIR, filename)


@app.route("/data")
def data():
    with data_lock:
        return jsonify(latest_data)


@app.route("/zero", methods=["POST"])
def zero():
    global zero_rotation_matrix

    if latest_rotation_matrix is not None:
        zero_rotation_matrix = latest_rotation_matrix.copy()
        print("Zero button pressed. Current position is now 0,0,0.")
        return jsonify({"status": "zeroed"})

    return jsonify({"status": "no sensor data yet"})


@app.route("/download_csv")
def download_csv():
    if latest_csv_path is None:
        return "No CSV file yet.", 404

    folder = os.path.dirname(latest_csv_path)
    filename = os.path.basename(latest_csv_path)

    return send_from_directory(folder, filename, as_attachment=True)


if __name__ == "__main__":
    thread = threading.Thread(target=sensor_and_logger_loop, daemon=True)
    thread.start()

    print()
    print("Starting web dashboard...")
    print("Open this on your Mac browser:")
    print("http://10.218.0.142:5000")
    print()

    app.run(host="0.0.0.0", port=5000)
