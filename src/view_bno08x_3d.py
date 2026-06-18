import math
import threading
import time

import board
import busio
from flask import Flask, jsonify, send_from_directory, render_template_string

from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C


IMG_DIR = "/home/atmccabe/Floating_Wind_Turbine/img"
TOP_IMAGE_NAME = "bno08x_top.png"
BOTTOM_IMAGE_NAME = "bno08x_bottom.png"

app = Flask(__name__)

orientation_data = {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}

zero_roll = 0.0
zero_pitch = 0.0
zero_yaw = 0.0


def quaternion_to_euler(i, j, k, real):
    # Roll
    sinr_cosp = 2 * (real * i + j * k)
    cosr_cosp = 1 - 2 * (i * i + j * j)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch
    sinp = 2 * (real * j - k * i)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw
    siny_cosp = 2 * (real * k + i * j)
    cosy_cosp = 1 - 2 * (j * j + k * k)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def wrap_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def sensor_loop():
    global orientation_data
    global zero_roll
    global zero_pitch
    global zero_yaw

    print("Starting BNO08x sensor loop...")

    i2c = busio.I2C(board.SCL, board.SDA)
    bno = BNO08X_I2C(i2c)

    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

    print("BNO08x connected.")

    first_zero_set = False

    while True:
        quat_i, quat_j, quat_k, quat_real = bno.quaternion

        if quat_i == 0 and quat_j == 0 and quat_k == 0 and quat_real == 0:
            time.sleep(0.05)
            continue

        roll, pitch, yaw = quaternion_to_euler(
            quat_i,
            quat_j,
            quat_k,
            quat_real
        )

        if not first_zero_set:
            zero_roll = roll
            zero_pitch = pitch
            zero_yaw = yaw
            first_zero_set = True
            print("Initial zero set.")

        orientation_data = {
            "roll": wrap_angle(roll - zero_roll),
            "pitch": wrap_angle(pitch - zero_pitch),
            "yaw": wrap_angle(yaw - zero_yaw),
        }

        time.sleep(0.02)


HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>BNO08x Live Viewer</title>

    <style>
        body {
            margin: 0;
            background: #111;
            color: white;
            font-family: Arial, sans-serif;
            text-align: center;
        }

        h1 {
            margin-top: 20px;
            margin-bottom: 5px;
        }

        .numbers {
            font-size: 26px;
            margin: 10px;
        }

        .hint {
            color: #aaa;
            margin-bottom: 15px;
        }

        .scene {
            width: 700px;
            height: 500px;
            margin: 0 auto;
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

        button {
            font-size: 18px;
            padding: 10px 20px;
            margin: 15px;
            border-radius: 10px;
            border: none;
            cursor: pointer;
        }
    </style>
</head>

<body>
    <h1>BNO08x Live Orientation</h1>

    <div class="numbers">
        Roll: <span id="roll">0.0</span>° |
        Pitch: <span id="pitch">0.0</span>° |
        Yaw: <span id="yaw">0.0</span>°
    </div>

    <div class="hint">
        Browser viewer = much smoother than Matplotlib. Red arrow shows front direction.
    </div>

    <button onclick="zeroSensor()">Zero current position</button>

    <div class="scene">
        <div class="board" id="board">
            <div class="face front"></div>
            <div class="face back"></div>
            <div class="front-arrow"></div>
        </div>
    </div>

    <script>
        async function updateData() {
            const response = await fetch("/data");
            const data = await response.json();

            const roll = data.roll;
            const pitch = data.pitch;
            const yaw = data.yaw;

            document.getElementById("roll").textContent = roll.toFixed(1);
            document.getElementById("pitch").textContent = pitch.toFixed(1);
            document.getElementById("yaw").textContent = yaw.toFixed(1);

            const board = document.getElementById("board");

            board.style.transform =
                `rotateZ(${yaw}deg) rotateY(${pitch}deg) rotateX(${roll}deg)`;
        }

        async function zeroSensor() {
            await fetch("/zero", { method: "POST" });
        }

        setInterval(updateData, 25);
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
    return jsonify(orientation_data)


@app.route("/zero", methods=["POST"])
def zero():
    global zero_roll
    global zero_pitch
    global zero_yaw

    # Since orientation_data is already relative, this resets by shifting zero to current absolute
    # in a simple way: request user to restart if this is not exact enough.
    # Better zeroing can be added by storing latest absolute values separately.
    zero_roll += orientation_data["roll"]
    zero_pitch += orientation_data["pitch"]
    zero_yaw += orientation_data["yaw"]

    print("Zero button pressed.")
    return jsonify({"status": "zeroed"})


if __name__ == "__main__":
    thread = threading.Thread(target=sensor_loop, daemon=True)
    thread.start()

    print("Starting web dashboard...")
    print("Open this on your Mac browser:")
    print("http://berrypi.local:5000")
    print()
    print("If that does not work, run hostname -I and use:")
    print("http://YOUR_PI_IP:5000")

    app.run(host="0.0.0.0", port=5000)
