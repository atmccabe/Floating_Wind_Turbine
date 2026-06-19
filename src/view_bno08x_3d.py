import math

import numpy as np

import matplotlib
matplotlib.use("TkAgg")

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import board
import busio

from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_ACCELEROMETER,
)

try:
    from adafruit_bno08x import BNO_REPORT_GRAVITY
    HAS_GRAVITY_REPORT = True
except ImportError:
    BNO_REPORT_GRAVITY = None
    HAS_GRAVITY_REPORT = False

try:
    from adafruit_bno08x import BNO_REPORT_LINEAR_ACCELERATION
    HAS_LINEAR_ACCEL_REPORT = True
except ImportError:
    BNO_REPORT_LINEAR_ACCELERATION = None
    HAS_LINEAR_ACCEL_REPORT = False

from adafruit_bno08x.i2c import BNO08X_I2C


# -----------------------------
# User settings
# -----------------------------
SHOW_FACE_LABELS = True
SHOW_MOTION_ARROW = True
MOTION_ARROW_SCALE = 1.2
MOTION_ARROW_MIN_LENGTH = 1.0
MOTION_ARROW_MAX_LENGTH = 2.0
MOTION_THRESHOLD = 0.08

VIEW_ELEV = 24
VIEW_AZIM = -45


def quaternion_to_euler(i, j, k, real):
    sinr_cosp = 2 * (real * i + j * k)
    cosr_cosp = 1 - 2 * (i * i + j * j)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (real * j - k * i)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (real * k + i * j)
    cosy_cosp = 1 - 2 * (j * j + k * k)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def quaternion_to_rotation_matrix(i, j, k, real):
    x = i
    y = j
    z = k
    w = real

    norm = math.sqrt(w * w + x * x + y * y + z * z)

    if norm == 0:
        return np.eye(3)

    w /= norm
    x /= norm
    y /= norm
    z /= norm

    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def strongest_axis(vector, threshold):
    x, y, z = vector

    values = {
        "+X": x,
        "-X": -x,
        "+Y": y,
        "-Y": -y,
        "+Z": z,
        "-Z": -z,
    }

    best_axis = max(values, key=values.get)
    best_value = values[best_axis]

    if best_value < threshold:
        return "none"

    return best_axis


def read_gravity_vector():
    if HAS_GRAVITY_REPORT:
        try:
            return np.array(bno.gravity, dtype=float), "gravity"
        except Exception:
            pass

    try:
        return np.array(bno.acceleration, dtype=float), "accelerometer fallback"
    except Exception:
        return np.array([0.0, 0.0, 0.0]), "not reading"


def read_linear_acceleration():
    if HAS_LINEAR_ACCEL_REPORT:
        try:
            return np.array(bno.linear_acceleration, dtype=float), "linear acceleration"
        except Exception:
            pass

    return np.array([0.0, 0.0, 0.0]), "not available"


def make_box_points():
    # Bigger visual box so it is easier to see
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
    {
        "indices": [0, 1, 2, 3],
        "axis": "-Z",
        "name": "BOTTOM",
        "label": "-Z",
        "color": "slategray",
    },
    {
        "indices": [4, 5, 6, 7],
        "axis": "+Z",
        "name": "TOP",
        "label": "+Z",
        "color": "orange",
    },
    {
        "indices": [0, 1, 5, 4],
        "axis": "-Y",
        "name": "BACK",
        "label": "-Y",
        "color": "royalblue",
    },
    {
        "indices": [2, 3, 7, 6],
        "axis": "+Y",
        "name": "FRONT",
        "label": "+Y",
        "color": "mediumorchid",
    },
    {
        "indices": [1, 2, 6, 5],
        "axis": "+X",
        "name": "RIGHT",
        "label": "+X",
        "color": "limegreen",
    },
    {
        "indices": [0, 3, 7, 4],
        "axis": "-X",
        "name": "LEFT",
        "label": "-X",
        "color": "tomato",
    },
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

    ax.set_xlabel("Axis X", fontsize=11, labelpad=8)
    ax.set_ylabel("Axis Y", fontsize=11, labelpad=8)
    ax.set_zlabel("Axis Z", fontsize=11, labelpad=8)

    ax.set_xticks(np.arange(-2, 3, 1))
    ax.set_yticks(np.arange(-2, 3, 1))
    ax.set_zticks(np.arange(-2, 3, 1))

    ax.tick_params(labelsize=8)

    ax.set_box_aspect([1, 1, 0.8])
    ax.view_init(elev=VIEW_ELEV, azim=VIEW_AZIM)

    # Keep the 3D grid visible
    ax.grid(True)

    # Make the background panes light so the box still stands out
    ax.xaxis.pane.set_alpha(0.08)
    ax.yaxis.pane.set_alpha(0.08)
    ax.zaxis.pane.set_alpha(0.08)

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
    info_ax.axis("off")

    quat_i, quat_j, quat_k, quat_real = bno.quaternion

    if quat_i == 0 and quat_j == 0 and quat_k == 0 and quat_real == 0:
        info_ax.text(
            0.05,
            0.95,
            "Waiting for BNO08x data...",
            fontsize=18,
            fontweight="bold",
            va="top",
        )
        return

    roll, pitch, yaw = quaternion_to_euler(
        quat_i,
        quat_j,
        quat_k,
        quat_real,
    )

    rotation = quaternion_to_rotation_matrix(
        quat_i,
        quat_j,
        quat_k,
        quat_real,
    )

    gravity_sensor, gravity_source = read_gravity_vector()
    linear_sensor, linear_source = read_linear_acceleration()

    gravity_mag = np.linalg.norm(gravity_sensor)
    linear_mag = np.linalg.norm(linear_sensor)

    gravity_axis = strongest_axis(gravity_sensor, threshold=1.0)
    motion_axis = strongest_axis(linear_sensor, threshold=MOTION_THRESHOLD)

    rotated_points = BOX_POINTS @ rotation.T

    rotated_faces = []
    face_centers = {}

    for face in FACE_INFO:
        face_points = [rotated_points[index] for index in face["indices"]]
        rotated_faces.append(face_points)

        face_points_array = np.array(face_points)
        face_centers[face["axis"]] = face_points_array.mean(axis=0)

    face_colors = [face["color"] for face in FACE_INFO]

    box = Poly3DCollection(
        rotated_faces,
        facecolors=face_colors,
        edgecolors="black",
        linewidths=1.6,
        alpha=0.95,
    )

    ax.add_collection3d(box)

    # Label only visible faces, and keep labels short.
    if SHOW_FACE_LABELS:
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

            label_pos = center + 0.08 * outward_normal
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

    # Red arrow attached to the gravity face.
    if gravity_axis in face_centers and gravity_axis in AXIS_VECTORS:
        face_center = face_centers[gravity_axis]
        sensor_normal = AXIS_VECTORS[gravity_axis]
        world_normal = rotation @ sensor_normal

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

    # Magenta motion arrow.
    # This shows push/acceleration direction, not true position.
    if SHOW_MOTION_ARROW and linear_mag > MOTION_THRESHOLD:
        motion_world = rotation @ linear_sensor
        motion_length = np.linalg.norm(motion_world)

        if motion_length > 0:
            motion_direction = motion_world / motion_length

            arrow_length = linear_mag * MOTION_ARROW_SCALE
            arrow_length = max(arrow_length, MOTION_ARROW_MIN_LENGTH)
            arrow_length = min(arrow_length, MOTION_ARROW_MAX_LENGTH)

            motion_arrow = motion_direction * arrow_length

            ax.quiver(
                0,
                0,
                0,
                motion_arrow[0],
                motion_arrow[1],
                motion_arrow[2],
                color="magenta",
                linewidth=9,
                arrow_length_ratio=0.35,
            )

            ax.text(
                motion_arrow[0],
                motion_arrow[1],
                motion_arrow[2],
                "MOTION",
                color="magenta",
                fontsize=13,
                fontweight="bold",
                ha="center",
                va="center",
                bbox=dict(
                    facecolor="white",
                    edgecolor="magenta",
                    alpha=0.9,
                    boxstyle="round,pad=0.25",
                ),
            )

    # -----------------------------
    # Right-side dashboard
    # -----------------------------
    face_lookup = {face["axis"]: face for face in FACE_INFO}

    if gravity_axis in face_lookup:
        gravity_face_name = face_lookup[gravity_axis]["name"]
    else:
        gravity_face_name = "UNKNOWN"

    dashboard = (
        "BNO085 LIVE VIEW\n"
        "----------------\n\n"
        f"Roll:    {roll:7.1f}°\n"
        f"Pitch:   {pitch:7.1f}°\n"
        f"Yaw:     {yaw:7.1f}°\n\n"
        f"Gravity axis: {gravity_axis}\n"
        f"Gravity face: {gravity_face_name}\n"
        f"|g|:          {gravity_mag:5.2f} m/s²\n\n"
        f"Motion axis:  {motion_axis}\n"
        f"Linear accel: {linear_mag:5.2f} m/s²\n\n"
        "PLOT KEY\n"
        "--------\n"
        "Red arrow     = gravity side\n"
        "Magenta arrow = push/motion\n"
        "Small G label = gravity face\n"
    )

    info_ax.text(
        0.03,
        0.97,
        dashboard,
        fontsize=15,
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
        "Gray   = BOTTOM -Z\n"
        "Blue   = FRONT -Y\n"
        "Purple = BACK +Y\n"
        "Green  = RIGHT +X\n"
        "Red    = LEFT -X\n"
    )

    info_ax.text(
        0.03,
        0.33,
        color_key,
        fontsize=13,
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


print("Connecting to BNO08x...")

i2c = busio.I2C(board.SCL, board.SDA)
bno = BNO08X_I2C(i2c)

bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
bno.enable_feature(BNO_REPORT_ACCELEROMETER)

if HAS_GRAVITY_REPORT:
    try:
        bno.enable_feature(BNO_REPORT_GRAVITY)
        print("Gravity report enabled.")
    except Exception:
        print("Could not enable gravity report. Using accelerometer fallback.")

if HAS_LINEAR_ACCEL_REPORT:
    try:
        bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
        print("Linear acceleration report enabled.")
    except Exception:
        print("Could not enable linear acceleration report.")

print("BNO08x connected.")
print("Opening clean 3D viewer...")

fig = plt.figure(figsize=(13, 8.5))
grid = fig.add_gridspec(1, 2, width_ratios=[3.0, 1.25])

ax = fig.add_subplot(grid[0, 0], projection="3d")
info_ax = fig.add_subplot(grid[0, 1])

fig.subplots_adjust(left=0.02, right=0.98, top=0.96, bottom=0.04)

animation = FuncAnimation(
    fig,
    update,
    interval=100,
    cache_frame_data=False,
)

plt.show()
