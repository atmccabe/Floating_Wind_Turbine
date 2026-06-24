#!/usr/bin/env python3

import time
import math
import numpy as np
import spidev

import matplotlib
matplotlib.use("TkAgg")

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


DIAG_STAT = 0x02

X_GYRO_OUT = 0x06
Y_GYRO_OUT = 0x0A
Z_GYRO_OUT = 0x0E

X_ACCL_OUT = 0x12
Y_ACCL_OUT = 0x16
Z_ACCL_OUT = 0x1A

PROD_ID = 0x72
EXPECTED_PROD_ID = 0x4056

G = 9.80665


def signed16(value):
    value = value & 0xFFFF
    if value & 0x8000:
        return value - 0x10000
    return value


def xfer16(spi, word):
    tx = [(word >> 8) & 0xFF, word & 0xFF]
    rx = spi.xfer2(tx)
    return (rx[0] << 8) | rx[1]


def read_reg16(spi, addr):
    xfer16(spi, (addr & 0x7F) << 8)
    time.sleep(0.00005)
    return xfer16(spi, 0x0000)


def read_sample(spi):
    prod = read_reg16(spi, PROD_ID)
    diag = read_reg16(spi, DIAG_STAT)

    gx_raw = signed16(read_reg16(spi, X_GYRO_OUT))
    gy_raw = signed16(read_reg16(spi, Y_GYRO_OUT))
    gz_raw = signed16(read_reg16(spi, Z_GYRO_OUT))

    ax_raw = signed16(read_reg16(spi, X_ACCL_OUT))
    ay_raw = signed16(read_reg16(spi, Y_ACCL_OUT))
    az_raw = signed16(read_reg16(spi, Z_ACCL_OUT))

    gx = math.radians(gx_raw * 0.1)
    gy = math.radians(gy_raw * 0.1)
    gz = math.radians(gz_raw * 0.1)

    ax = ax_raw * 0.00125 * G
    ay = ay_raw * 0.00125 * G
    az = az_raw * 0.00125 * G

    return prod, diag, gx, gy, gz, ax, ay, az


def rotation_matrix(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)

    cp = math.cos(pitch)
    sp = math.sin(pitch)

    cy = math.cos(yaw)
    sy = math.sin(yaw)

    rx = np.array([
        [1, 0, 0],
        [0, cr, -sr],
        [0, sr, cr],
    ])

    ry = np.array([
        [cp, 0, sp],
        [0, 1, 0],
        [-sp, 0, cp],
    ])

    rz = np.array([
        [cy, -sy, 0],
        [sy, cy, 0],
        [0, 0, 1],
    ])

    return rz @ ry @ rx


def make_box():
    x = 1.2
    y = 0.8
    z = 0.25

    points = np.array([
        [-x, -y, -z],
        [ x, -y, -z],
        [ x,  y, -z],
        [-x,  y, -z],
        [-x, -y,  z],
        [ x, -y,  z],
        [ x,  y,  z],
        [-x,  y,  z],
    ])

    faces = [
        [0, 1, 2, 3],
        [4, 5, 6, 7],
        [0, 1, 5, 4],
        [2, 3, 7, 6],
        [1, 2, 6, 5],
        [0, 3, 7, 4],
    ]

    return points, faces


def main():
    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.mode = 0b11
    spi.max_speed_hz = 100000
    spi.bits_per_word = 8
    spi.lsbfirst = False
    spi.cshigh = False

    points, faces = make_box()

    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    alpha = 0.98

    last_time = time.monotonic()

    fig = plt.figure()
    ax3d = fig.add_subplot(111, projection="3d")

    plt.ion()

    try:
        while True:
            now = time.monotonic()
            dt = now - last_time
            last_time = now

            prod, diag, gx, gy, gz, ax, ay, az = read_sample(spi)

            roll_acc = math.atan2(ay, az)
            pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az))

            roll = alpha * (roll + gx * dt) + (1.0 - alpha) * roll_acc
            pitch = alpha * (pitch + gy * dt) + (1.0 - alpha) * pitch_acc
            yaw = yaw + gz * dt

            r = rotation_matrix(roll, pitch, yaw)
            rotated = points @ r.T

            ax3d.clear()

            box_faces = []
            for face in faces:
                box_faces.append([rotated[i] for i in face])

            box = Poly3DCollection(box_faces, alpha=0.6, edgecolor="black")
            ax3d.add_collection3d(box)

            ax3d.quiver(0, 0, 0, 2, 0, 0, length=1.0)
            ax3d.quiver(0, 0, 0, 0, 2, 0, length=1.0)
            ax3d.quiver(0, 0, 0, 0, 0, 2, length=1.0)

            ax3d.text(2.2, 0, 0, "X")
            ax3d.text(0, 2.2, 0, "Y")
            ax3d.text(0, 0, 2.2, "Z")

            ax3d.set_xlim([-2, 2])
            ax3d.set_ylim([-2, 2])
            ax3d.set_zlim([-2, 2])

            ax3d.set_xlabel("X")
            ax3d.set_ylabel("Y")
            ax3d.set_zlabel("Z")

            ax3d.set_title(
                f"ADIS16470  PROD=0x{prod:04X}  "
                f"Roll={math.degrees(roll):.1f}  "
                f"Pitch={math.degrees(pitch):.1f}  "
                f"Yaw={math.degrees(yaw):.1f}"
            )

            plt.pause(0.02)

    except KeyboardInterrupt:
        print("\nStopped.")

    finally:
        spi.close()


if __name__ == "__main__":
    main()
