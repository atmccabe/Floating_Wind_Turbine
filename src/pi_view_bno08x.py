import math
import sys
import time
import select

import board
import busio
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C


PLOT_DELAY = 0.04


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


def read_terminal_command():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip().lower()
    return None


def make_box_points():
    length = 1.0
    width = 0.60
    height = 0.18

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


def make_box_faces(points):
    return [
        [points[0], points[1], points[2], points[3]],  # bottom
        [points[4], points[5], points[6], points[7]],  # top
        [points[0], points[1], points[5], points[4]],  # side
        [points[1], points[2], points[6], points[5]],  # front
        [points[2], points[3], points[7], points[6]],  # side
        [points[3], points[0], points[4], points[7]],  # back
    ]


def rotate_points(rotation_matrix, points):
    return (rotation_matrix @ points.T).T


print("Starting simple BNO08x box viewer...")
print("Commands:")
print("  z + ENTER = zero current position")
print("  q + ENTER = quit")
print()

i2c = busio.I2C(board.SCL, board.SDA)
bno = BNO08X_I2C(i2c)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

box_points = make_box_points()

zero_rotation_matrix = None
running = True

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

ax.set_xlim([-1.5, 1.5])
ax.set_ylim([-1.5, 1.5])
ax.set_zlim([-1.5, 1.5])
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_box_aspect([1, 1, 1])

rotated_points = box_points.copy()
box_faces = make_box_faces(rotated_points)

box = Poly3DCollection(
    box_faces,
    facecolor="lightgray",
    edgecolor="black",
    linewidth=1.5,
    alpha=1.0
)
ax.add_collection3d(box)

# Simple front direction marker.
front_line, = ax.plot([0, 0.8], [0, 0], [0.18, 0.18], linewidth=4)

# Local sensor axes.
x_line, = ax.plot([0, 1.0], [0, 0], [0, 0], linewidth=2)
y_line, = ax.plot([0, 0], [0, 1.0], [0, 0], linewidth=2)
z_line, = ax.plot([0, 0], [0, 0], [0, 1.0], linewidth=2)

try:
    while running and plt.fignum_exists(fig.number):
        quat_i, quat_j, quat_k, quat_real = bno.quaternion

        if quat_i == 0 and quat_j == 0 and quat_k == 0 and quat_real == 0:
            time.sleep(0.05)
            continue

        current_rotation_matrix = quaternion_to_rotation_matrix(
            quat_i,
            quat_j,
            quat_k,
            quat_real
        )

        if zero_rotation_matrix is None:
            zero_rotation_matrix = current_rotation_matrix.copy()
            print("Initial zero set.")

        command = read_terminal_command()

        if command == "z":
            zero_rotation_matrix = current_rotation_matrix.copy()
            print("Zero reset.")

        elif command == "q":
            print("Quitting.")
            break

        relative_rotation_matrix = zero_rotation_matrix.T @ current_rotation_matrix

        roll, pitch, yaw = rotation_matrix_to_euler(relative_rotation_matrix)

        rotated_points = rotate_points(relative_rotation_matrix, box_points)
        box_faces = make_box_faces(rotated_points)
        box.set_verts(box_faces)

        # Update front marker.
        front_start = relative_rotation_matrix @ np.array([0.0, 0.0, 0.18])
        front_end = relative_rotation_matrix @ np.array([0.85, 0.0, 0.18])
        front_line.set_data_3d(
            [front_start[0], front_end[0]],
            [front_start[1], front_end[1]],
            [front_start[2], front_end[2]]
        )

        # Update local axes.
        x_axis = relative_rotation_matrix @ np.array([1.0, 0.0, 0.0])
        y_axis = relative_rotation_matrix @ np.array([0.0, 1.0, 0.0])
        z_axis = relative_rotation_matrix @ np.array([0.0, 0.0, 1.0])

        x_line.set_data_3d([0, x_axis[0]], [0, x_axis[1]], [0, x_axis[2]])
        y_line.set_data_3d([0, y_axis[0]], [0, y_axis[1]], [0, y_axis[2]])
        z_line.set_data_3d([0, z_axis[0]], [0, z_axis[1]], [0, z_axis[2]])

        ax.set_title(
            f"Simple BNO08x Box Viewer\n"
            f"Roll: {roll:6.1f} deg   Pitch: {pitch:6.1f} deg   Yaw: {yaw:6.1f} deg"
        )

        plt.pause(PLOT_DELAY)

except KeyboardInterrupt:
    print("\nStopped.")

plt.close()
