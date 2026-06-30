#!/usr/bin/env python3

import argparse
import re
from pathlib import Path

import numpy as np
import pandas as pd

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter, FFMpegWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def clean_name(name):
    return re.sub(r"[^a-z0-9]", "", str(name).lower())


def find_col(df, aliases):
    lookup = {clean_name(c): c for c in df.columns}
    for alias in aliases:
        key = clean_name(alias)
        if key in lookup:
            return lookup[key]
    return None


def read_num(df, col):
    return np.array(pd.to_numeric(df[col], errors="coerce"), dtype=float, copy=True)


def get_time(df):
    time_col = find_col(df, [
        "time_s", "time", "t_s", "t", "seconds", "sec",
        "timestamp_s", "timestamp"
    ])

    if time_col is None:
        return np.arange(len(df), dtype=float), "sample_index"

    t = read_num(df, time_col)

    if not np.isfinite(t).any():
        return np.arange(len(df), dtype=float), "sample_index"

    good = np.isfinite(t)
    t[~good] = np.interp(np.flatnonzero(~good), np.flatnonzero(good), t[good])

    t = t - t[0]
    return t, time_col


def maybe_to_rad(values, col_name):
    name = clean_name(col_name)

    if "rad" in name:
        return values

    if "deg" in name:
        return np.deg2rad(values)

    max_val = np.nanmax(np.abs(values))

    if max_val > 2.0 * np.pi + 0.5:
        return np.deg2rad(values)

    return values


def euler_to_rot_mats(roll, pitch, yaw):
    cr = np.cos(roll)
    sr = np.sin(roll)

    cp = np.cos(pitch)
    sp = np.sin(pitch)

    cy = np.cos(yaw)
    sy = np.sin(yaw)

    R = np.zeros((len(roll), 3, 3))

    R[:, 0, 0] = cy * cp
    R[:, 0, 1] = cy * sp * sr - sy * cr
    R[:, 0, 2] = cy * sp * cr + sy * sr

    R[:, 1, 0] = sy * cp
    R[:, 1, 1] = sy * sp * sr + cy * cr
    R[:, 1, 2] = sy * sp * cr - cy * sr

    R[:, 2, 0] = -sp
    R[:, 2, 1] = cp * sr
    R[:, 2, 2] = cp * cr

    return R


def quat_to_rot_mats(w, x, y, z):
    q = np.vstack([w, x, y, z]).T
    norms = np.linalg.norm(q, axis=1)
    norms[norms == 0] = 1.0

    w, x, y, z = (q / norms[:, None]).T

    R = np.zeros((len(w), 3, 3))

    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - z * w)
    R[:, 0, 2] = 2 * (x * z + y * w)

    R[:, 1, 0] = 2 * (x * y + z * w)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - x * w)

    R[:, 2, 0] = 2 * (x * z - y * w)
    R[:, 2, 1] = 2 * (y * z + x * w)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)

    return R


def gyro_to_rad(values, col_name):
    name = clean_name(col_name)

    if "dps" in name or "deg" in name:
        return np.deg2rad(values)

    return values


def accel_gyro_orientation(df, t, alpha):
    ax_col = find_col(df, ["accel_x_mps2", "accel_x", "acc_x", "ax_mps2", "ax"])
    ay_col = find_col(df, ["accel_y_mps2", "accel_y", "acc_y", "ay_mps2", "ay"])
    az_col = find_col(df, ["accel_z_mps2", "accel_z", "acc_z", "az_mps2", "az"])

    gx_col = find_col(df, ["gyro_x_radps", "gyro_x", "gx_radps", "gx_dps", "gx"])
    gy_col = find_col(df, ["gyro_y_radps", "gyro_y", "gy_radps", "gy_dps", "gy"])
    gz_col = find_col(df, ["gyro_z_radps", "gyro_z", "gz_radps", "gz_dps", "gz"])

    needed = [ax_col, ay_col, az_col, gx_col, gy_col, gz_col]
    if any(c is None for c in needed):
        return None, None

    ax = read_num(df, ax_col)
    ay = read_num(df, ay_col)
    az = read_num(df, az_col)

    gx = gyro_to_rad(read_num(df, gx_col), gx_col)
    gy = gyro_to_rad(read_num(df, gy_col), gy_col)
    gz = gyro_to_rad(read_num(df, gz_col), gz_col)

    n = len(df)

    roll = np.zeros(n)
    pitch = np.zeros(n)
    yaw = np.zeros(n)

    roll_acc = np.arctan2(ay, az)
    pitch_acc = np.arctan2(-ax, np.sqrt(ay * ay + az * az))

    roll[0] = roll_acc[0]
    pitch[0] = pitch_acc[0]
    yaw[0] = 0.0

    dt_default = 0.02
    if len(t) > 2:
        good_dt = np.diff(t)
        good_dt = good_dt[np.isfinite(good_dt) & (good_dt > 0)]
        if len(good_dt) > 0:
            dt_default = np.median(good_dt)

    for i in range(1, n):
        dt = t[i] - t[i - 1]
        if not np.isfinite(dt) or dt <= 0:
            dt = dt_default

        roll_gyro = roll[i - 1] + gx[i] * dt
        pitch_gyro = pitch[i - 1] + gy[i] * dt
        yaw[i] = yaw[i - 1] + gz[i] * dt

        roll[i] = alpha * roll_gyro + (1.0 - alpha) * roll_acc[i]
        pitch[i] = alpha * pitch_gyro + (1.0 - alpha) * pitch_acc[i]

    R = euler_to_rot_mats(roll, pitch, yaw)

    source = (
        "accel + gyro complementary filter "
        f"({ax_col}, {ay_col}, {az_col}, {gx_col}, {gy_col}, {gz_col})"
    )

    return R, source


def load_orientation(csv_path, alpha):
    df = pd.read_csv(csv_path)
    t, time_col = get_time(df)

    qw_col = find_col(df, ["quat_real", "quat_w", "quaternion_w", "qw", "q0"])
    qx_col = find_col(df, ["quat_i", "quat_x", "quaternion_x", "qx", "q1"])
    qy_col = find_col(df, ["quat_j", "quat_y", "quaternion_y", "qy", "q2"])
    qz_col = find_col(df, ["quat_k", "quat_z", "quaternion_z", "qz", "q3"])

    if all(c is not None for c in [qw_col, qx_col, qy_col, qz_col]):
        w = read_num(df, qw_col)
        x = read_num(df, qx_col)
        y = read_num(df, qy_col)
        z = read_num(df, qz_col)

        R = quat_to_rot_mats(w, x, y, z)
        source = f"quaternion columns ({qx_col}, {qy_col}, {qz_col}, {qw_col})"
        return df, t, R, source, time_col

    roll_col = find_col(df, ["roll_deg", "roll_rad", "roll", "phi_deg", "phi_rad", "phi"])
    pitch_col = find_col(df, ["pitch_deg", "pitch_rad", "pitch", "theta_deg", "theta_rad", "theta"])
    yaw_col = find_col(df, ["yaw_deg", "yaw_rad", "yaw", "heading_deg", "heading_rad", "heading", "psi_deg", "psi_rad", "psi"])

    if all(c is not None for c in [roll_col, pitch_col, yaw_col]):
        roll = maybe_to_rad(read_num(df, roll_col), roll_col)
        pitch = maybe_to_rad(read_num(df, pitch_col), pitch_col)
        yaw = maybe_to_rad(read_num(df, yaw_col), yaw_col)

        R = euler_to_rot_mats(roll, pitch, yaw)
        source = f"Euler columns ({roll_col}, {pitch_col}, {yaw_col})"
        return df, t, R, source, time_col

    R, source = accel_gyro_orientation(df, t, alpha)

    if R is not None:
        return df, t, R, source, time_col

    raise SystemExit(
        "\nCould not find usable motion columns.\n\n"
        "Supported options:\n"
        "  1) Quaternion: quat_i, quat_j, quat_k, quat_real\n"
        "  2) Euler angles: roll_deg, pitch_deg, yaw_deg\n"
        "  3) Raw estimate: accel_x/y/z and gyro_x/y/z\n\n"
        f"Columns found:\n{list(df.columns)}\n"
    )


def make_box():
    length = 1.15
    width = 0.70
    height = 0.25

    x = length / 2
    y = width / 2
    z = height / 2

    verts = np.array([
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

    return verts, faces


def transform_faces(verts, faces, R):
    rotated = verts @ R.T
    return [[rotated[i] for i in face] for face in faces]


def choose_frames(t, n, fps, max_frames):
    if n <= 1:
        return np.array([0])

    dt = np.diff(t)
    dt = dt[np.isfinite(dt) & (dt > 0)]

    if len(dt) == 0:
        step = 1
    else:
        sample_rate = 1.0 / np.median(dt)
        step = max(1, int(round(sample_rate / fps)))

    idx = np.arange(0, n, step)

    if max_frames > 0 and len(idx) > max_frames:
        idx = np.linspace(0, n - 1, max_frames).astype(int)

    return np.unique(idx)


def default_output_path(csv_path):
    csv_path = Path(csv_path)

    if csv_path.parent.name == "raw" and csv_path.parent.parent.name == "data":
        sensor_dir = csv_path.parent.parent.parent
        out_dir = sensor_dir / "animations"
    else:
        out_dir = csv_path.parent / "animations"

    out_dir.mkdir(parents=True, exist_ok=True)

    return out_dir / f"{csv_path.stem}_3d_replay.gif"


def make_animation(csv_path, out_path, fps, max_frames, alpha, dpi):
    df, t, R_mats, source, time_col = load_orientation(csv_path, alpha)

    frame_idx = choose_frames(t, len(df), fps, max_frames)

    verts, faces = make_box()

    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(111, projection="3d")

    ax.set_xlim(-1.4, 1.4)
    ax.set_ylim(-1.4, 1.4)
    ax.set_zlim(-1.4, 1.4)

    ax.set_xlabel("World X")
    ax.set_ylabel("World Y")
    ax.set_zlabel("World Z")

    ax.set_box_aspect([1, 1, 1])

    # Fixed world axes
    ax.plot([0, 1.2], [0, 0], [0, 0], color="red", alpha=0.25, label="World X")
    ax.plot([0, 0], [0, 1.2], [0, 0], color="green", alpha=0.25, label="World Y")
    ax.plot([0, 0], [0, 0], [0, 1.2], color="blue", alpha=0.25, label="World Z")

    # Sensor box
    first_R = R_mats[frame_idx[0]]
    poly = Poly3DCollection(
        transform_faces(verts, faces, first_R),
        alpha=0.65,
        edgecolors="black",
        linewidths=0.7
    )
    ax.add_collection3d(poly)

    # Moving sensor axes
    local_x, = ax.plot([0, 1], [0, 0], [0, 0], color="red", linewidth=3, label="Sensor X")
    local_y, = ax.plot([0, 0], [0, 1], [0, 0], color="green", linewidth=3, label="Sensor Y")
    local_z, = ax.plot([0, 0], [0, 0], [0, 1], color="blue", linewidth=3, label="Sensor Z")

    title = ax.set_title("Loading...")

    ax.legend(loc="upper left")

    basis = np.eye(3)
    axis_len = 1.05

    def update(frame_number):
        i = frame_idx[frame_number]
        R = R_mats[i]

        poly.set_verts(transform_faces(verts, faces, R))

        ends = basis @ R.T * axis_len

        local_x.set_data([0, ends[0, 0]], [0, ends[0, 1]])
        local_x.set_3d_properties([0, ends[0, 2]])

        local_y.set_data([0, ends[1, 0]], [0, ends[1, 1]])
        local_y.set_3d_properties([0, ends[1, 2]])

        local_z.set_data([0, ends[2, 0]], [0, ends[2, 1]])
        local_z.set_3d_properties([0, ends[2, 2]])

        title.set_text(
            f"{Path(csv_path).name}\n"
            f"t = {t[i]:.2f} s | frame {frame_number + 1}/{len(frame_idx)}\n"
            f"motion source: {source}"
        )

        return poly, local_x, local_y, local_z, title

    anim = FuncAnimation(
        fig,
        update,
        frames=len(frame_idx),
        interval=1000 / fps,
        blit=False
    )

    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    suffix = out_path.suffix.lower()

    print(f"Reading: {csv_path}")
    print(f"Rows: {len(df)}")
    print(f"Time column: {time_col}")
    print(f"Motion source: {source}")
    print(f"Animation frames: {len(frame_idx)}")
    print(f"Saving: {out_path}")

    if suffix == ".mp4":
        writer = FFMpegWriter(fps=fps)
    else:
        writer = PillowWriter(fps=fps)

    anim.save(out_path, writer=writer, dpi=dpi)
    plt.close(fig)

    print("Done.")


def main():
    parser = argparse.ArgumentParser(
        description="Replay IMU CSV motion as a 3D animation."
    )

    parser.add_argument("csv", help="Path to CSV file")
    parser.add_argument("--out", default=None, help="Output .gif or .mp4 path")
    parser.add_argument("--fps", type=int, default=20, help="Animation frames per second")
    parser.add_argument("--max-frames", type=int, default=900, help="Max animation frames. Use 0 for no cap.")
    parser.add_argument("--alpha", type=float, default=0.98, help="Accel/gyro complementary filter weight")
    parser.add_argument("--dpi", type=int, default=120, help="Output resolution")

    args = parser.parse_args()

    csv_path = Path(args.csv)

    if not csv_path.exists():
        raise SystemExit(f"CSV file not found: {csv_path}")

    out_path = Path(args.out) if args.out else default_output_path(csv_path)

    make_animation(
        csv_path=csv_path,
        out_path=out_path,
        fps=args.fps,
        max_frames=args.max_frames,
        alpha=args.alpha,
        dpi=args.dpi
    )


if __name__ == "__main__":
    main()
