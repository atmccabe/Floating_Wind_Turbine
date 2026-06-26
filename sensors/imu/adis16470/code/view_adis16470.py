import argparse
import importlib
import math
import os
import sys

import numpy as np

import matplotlib
matplotlib.use("TkAgg")

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


PROJECT_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../../../..")
)
sys.path.insert(0, PROJECT_ROOT)


VIEW_ELEV = 24
VIEW_AZIM = -45

ROLL_SIGN = 1.0
PITCH_SIGN = 1.0
YAW_SIGN = 1.0


def load_sensor(sensor_name):
    module_name = f"sensors.imu.{sensor_name}.code.{sensor_name}_sensor"
    module = importlib.import_module(module_name)
    return module.Sensor()


def rot_x(a):
    c = math.cos(a)
    s = math.sin(a)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c],
    ])


def rot_y(a):
    c = math.cos(a)
    s = math.sin(a)
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c],
    ])


def rot_z(a):
    c = math.cos(a)
    s = math.sin(a)
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1],
    ])


def euler_to_rotation(roll_deg, pitch_deg, yaw_deg):
    roll = math.radians(ROLL_SIGN * roll_deg)
    pitch = math.radians(PITCH_SIGN * pitch_deg)
    yaw = math.radians(YAW_SIGN * yaw_deg)

    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def strongest_axis(v):
    x, y, z = v

    values = {
        "+X": x,
        "-X": -x,
        "+Y": y,
        "-Y": -y,
        "+Z": z,
        "-Z": -z,
    }

    return max(values, key=values.get)


def make_box_points():
    length = 2.8
    width = 1.8
    height = 0.35

    x = length / 2
    y = width / 2
    z = height / 2

    return np.array([
        [-x, -y, -z],
        [ x, -y, -z],
        [ x,  y, -z],
        [-x,  y, -z],
        [-x, -y,  z],
        [ x, -y,  z],
        [ x,  y,  z],
        [-x,  y,  z],
    ])


BOX_POINTS = make_box_points()

FACE_INFO = [
    {"indices": [0, 1, 2, 3], "axis": "-Z", "name": "BOTTOM", "label": "-Z", "color": "cyan"},
    {"indices": [4, 5, 6, 7], "axis": "+Z", "name": "TOP",    "label": "+Z", "color": "orange"},
    {"indices": [0, 1, 5, 4], "axis": "-Y", "name": "BACK",   "label": "-Y", "color": "royalblue"},
    {"indices": [2, 3, 7, 6], "axis": "+Y", "name": "FRONT",  "label": "+Y", "color": "mediumorchid"},
    {"indices": [1, 2, 6, 5], "axis": "+X", "name": "RIGHT",  "label": "+X", "color": "limegreen"},
    {"indices": [0, 3, 7, 4], "axis": "-X", "name": "LEFT",   "label": "-X", "color": "tomato"},
]

AXIS_VECTORS = {
    "+X": np.array([1, 0, 0]),
    "-X": np.array([-1, 0, 0]),
    "+Y": np.array([0, 1, 0]),
    "-Y": np.array([0, -1, 0]),
    "+Z": np.array([0, 0, 1]),
    "-Z": np.array([0, 0, -1]),
}


def setup_3d_axis(ax):
    ax.set_xlim(-2.8, 2.8)
    ax.set_ylim(-2.8, 2.8)
    ax.set_zlim(-2.2, 2.2)

    ax.set_xlabel("Axis X", fontsize=11, labelpad=12)
    ax.set_ylabel("Axis Y", fontsize=11, labelpad=12)
    ax.set_zlabel("Axis Z", fontsize=11, labelpad=12)

    ax.set_xticks(np.arange(-2, 3, 1))
    ax.set_yticks(np.arange(-2, 3, 1))
    ax.set_zticks(np.arange(-2, 3, 1))

    ax.set_box_aspect([1, 1, 0.8])
    ax.view_init(elev=VIEW_ELEV, azim=VIEW_AZIM)
    ax.grid(True)

    ax.xaxis.pane.set_alpha(0.08)
    ax.yaxis.pane.set_alpha(0.08)
    ax.zaxis.pane.set_alpha(0.08)


def draw_world_axes(ax):
    ax.quiver(0, 0, 0, 1.2, 0, 0, color="red", linewidth=2)
    ax.quiver(0, 0, 0, 0, 1.2, 0, color="green", linewidth=2)
    ax.quiver(0, 0, 0, 0, 0, 1.2, color="blue", linewidth=2)

    ax.text(1.35, 0, 0, "X", fontsize=11)
    ax.text(0, 1.35, 0, "Y", fontsize=11)
    ax.text(0, 0, 1.35, "Z", fontsize=11)


def get_view_direction():
    az = math.radians(VIEW_AZIM)
    el = math.radians(VIEW_ELEV)

    return np.array([
        math.cos(el) * math.cos(az),
        math.cos(el) * math.sin(az),
        math.sin(el),
    ])


def update(frame):
    ax.clear()
    info_ax.clear()

    setup_3d_axis(ax)
    draw_world_axes(ax)
    info_ax.axis("off")

    data = sensor.read()

    roll = float(data.get("roll_deg", 0.0))
    pitch = float(data.get("pitch_deg", 0.0))
    yaw = float(data.get("yaw_deg", 0.0))

    ax_g = float(data.get("accel_x_g", 0.0))
    ay_g = float(data.get("accel_y_g", 0.0))
    az_g = float(data.get("accel_z_g", 0.0))

    accel_sensor = np.array([ax_g, ay_g, az_g])
    gravity_axis = strongest_axis(accel_sensor)

    rotation = euler_to_rotation(roll, pitch, yaw)
    rotated_points = BOX_POINTS @ rotation.T

    rotated_faces = []
    face_centers = {}

    for face in FACE_INFO:
        face_points = [rotated_points[index] for index in face["indices"]]
        rotated_faces.append(face_points)
        face_centers[face["axis"]] = np.array(face_points).mean(axis=0)

    face_colors = [face["color"] for face in FACE_INFO]

    box = Poly3DCollection(
        rotated_faces,
        facecolors=face_colors,
        edgecolors="black",
        linewidths=1.6,
        alpha=0.95,
    )

    ax.add_collection3d(box)

    view_dir = get_view_direction()

    for face_points, face in zip(rotated_faces, FACE_INFO):
        center = np.array(face_points).mean(axis=0)
        outward_normal = rotation @ AXIS_VECTORS[face["axis"]]

        visible = np.dot(outward_normal, view_dir) > 0.15

        if not visible and face["axis"] != gravity_axis:
            continue

        label_text = face["label"]

        if face["axis"] == gravity_axis:
            label_text = f"{face['label']}\nG"

        label_pos = center + 0.5 * outward_normal
        edge_color = "red" if face["axis"] == gravity_axis else "black"

        ax.text(
            label_pos[0],
            label_pos[1],
            label_pos[2],
            label_text,
            fontsize=12,
            fontweight="bold",
            color="black",
            ha="center",
            va="center",
            bbox=dict(
                facecolor="white",
                edgecolor=edge_color,
                linewidth=1.5,
                alpha=0.95,
                boxstyle="round,pad=0.20",
            ),
        )

    if gravity_axis in face_centers:
        face_center = face_centers[gravity_axis]
        world_normal = rotation @ AXIS_VECTORS[gravity_axis]

        arrow_start = face_center + 0.05 * world_normal
        arrow_direction = 0.85 * world_normal

        ax.quiver(
            arrow_start[0],
            arrow_start[1],
            arrow_start[2],
            arrow_direction[0],
            arrow_direction[1],
            arrow_direction[2],
            color="red",
            linewidth=6,
            arrow_length_ratio=0.30,
        )

    gravity_mag = np.linalg.norm(accel_sensor)

    dashboard = (
        "ADIS16470 LIVE VIEW\n"
        "-------------------\n\n"
        f"Roll:  {roll:7.1f}°\n"
        f"Pitch: {pitch:7.1f}°\n"
        f"Yaw:   {yaw:7.1f}°\n\n"
        f"Accel X: {ax_g:7.3f} g\n"
        f"Accel Y: {ay_g:7.3f} g\n"
        f"Accel Z: {az_g:7.3f} g\n\n"
        f"Gravity axis: {gravity_axis}\n"
        f"|accel|: {gravity_mag:5.2f} g\n\n"
        "KEYS\n"
        "----\n"
        "z = zero orientation\n"
        "q = quit\n"
    )

    info_ax.text(
        0.03,
        0.97,
        dashboard,
        fontsize=14,
        family="monospace",
        va="top",
        color="black",
        bbox=dict(
            facecolor="white",
            edgecolor="black",
            alpha=0.95,
            boxstyle="round,pad=0.55",
        ),
    )

    color_key = (
        "FACE COLOR KEY\n"
        "--------------\n"
        "Orange = TOP +Z\n"
        "Cyan   = BOTTOM -Z\n"
        "Purple = FRONT +Y\n"
        "Blue   = BACK -Y\n"
        "Green  = RIGHT +X\n"
        "Red    = LEFT -X\n"
    )

    info_ax.text(
        0.03,
        0.30,
        color_key,
        fontsize=12,
        family="monospace",
        va="top",
        color="black",
        bbox=dict(
            facecolor="white",
            edgecolor="black",
            alpha=0.95,
            boxstyle="round,pad=0.45",
        ),
    )


def on_key(event):
    if event.key == "z":
        sensor.zero()
        print("Zeroed ADIS orientation.")

    if event.key == "q":
        plt.close("all")


def main():
    global sensor
    global ax
    global info_ax

    parser = argparse.ArgumentParser()
    parser.add_argument("sensor", nargs="?", default="adis16470")
    parser.add_argument("--rate", type=float, default=15.0)
    args = parser.parse_args()

    print("Connecting to ADIS16470...")
    sensor = load_sensor(args.sensor)
    sensor.connect()

    print("ADIS16470 connected.")
    print("Opening live 3D viewer...")
    print("Move the physical ADIS sensor. Press z to zero. Press q to quit.")

    fig = plt.figure(figsize=(13, 8.5))
    grid = fig.add_gridspec(1, 2, width_ratios=[3.0, 1.25])

    ax = fig.add_subplot(grid[0, 0], projection="3d")
    info_ax = fig.add_subplot(grid[0, 1])

    fig.subplots_adjust(left=0.02, right=0.98, top=0.96, bottom=0.04)
    fig.canvas.mpl_connect("key_press_event", on_key)

    interval_ms = int(1000 / args.rate)

    animation = FuncAnimation(
        fig,
        update,
        interval=interval_ms,
        cache_frame_data=False,
    )

    plt.show()


if __name__ == "__main__":
    main()
