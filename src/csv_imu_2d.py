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
from matplotlib.patches import Polygon


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

    if not good.all():
        t[~good] = np.interp(
            np.flatnonzero(~good),
            np.flatnonzero(good),
            t[good]
        )

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


def gyro_to_rad(values, col_name):
    name = clean_name(col_name)

    if "dps" in name or "deg" in name:
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


def smooth(values, window):
    if window <= 1:
        return values

    return (
        pd.Series(values)
        .rolling(window=window, center=True, min_periods=1)
        .mean()
        .to_numpy()
    )


def get_default_paths(csv_path):
    csv_path = Path(csv_path)

    if csv_path.parent.name == "raw" and csv_path.parent.parent.name == "data":
        sensor_dir = csv_path.parent.parent.parent

        animation_dir = sensor_dir / "animations"
        plot_dir = sensor_dir / "plots"
        processed_dir = sensor_dir / "data" / "processed"
    else:
        animation_dir = csv_path.parent / "animations"
        plot_dir = csv_path.parent / "plots"
        processed_dir = csv_path.parent / "processed"

    animation_dir.mkdir(parents=True, exist_ok=True)
    plot_dir.mkdir(parents=True, exist_ok=True)
    processed_dir.mkdir(parents=True, exist_ok=True)

    gif_path = animation_dir / f"{csv_path.stem}_2d_pitch.gif"
    angle_csv_path = processed_dir / f"{csv_path.stem}_2d_angle.csv"
    angle_plot_path = plot_dir / f"{csv_path.stem}_2d_angle.png"

    return gif_path, angle_csv_path, angle_plot_path


def axis_index(axis_name):
    axis_name = axis_name.lower()

    if axis_name == "x":
        return 0

    if axis_name == "y":
        return 1

    if axis_name == "z":
        return 2

    raise ValueError("axis must be x, y, or z")


def compute_side_angle(R_mats, axis_name, zero_first, invert_angle, smooth_window):
    idx = axis_index(axis_name)

    # Local sensor axis expressed in world coordinates.
    axis_world = R_mats[:, :, idx]

    # Side view uses the X-Z plane.
    x = axis_world[:, 0]
    z = axis_world[:, 2]

    angle_rad = np.unwrap(np.arctan2(x, z))
    angle_raw_deg = np.rad2deg(angle_rad)

    if zero_first:
        angle_display_deg = angle_raw_deg - angle_raw_deg[0]
    else:
        angle_display_deg = angle_raw_deg.copy()

    if invert_angle:
        angle_display_deg = -angle_display_deg
        angle_raw_deg = -angle_raw_deg

    angle_smooth_deg = smooth(angle_display_deg, smooth_window)

    return angle_raw_deg, angle_display_deg, angle_smooth_deg


def rotated_box(center_x, center_z, angle_deg, length=0.28, width=0.12):
    a = np.deg2rad(angle_deg)

    ux = np.sin(a)
    uz = np.cos(a)

    px = np.cos(a)
    pz = -np.sin(a)

    cx = center_x
    cz = center_z

    p1 = [
        cx + ux * length / 2 + px * width / 2,
        cz + uz * length / 2 + pz * width / 2
    ]

    p2 = [
        cx + ux * length / 2 - px * width / 2,
        cz + uz * length / 2 - pz * width / 2
    ]

    p3 = [
        cx - ux * length / 2 - px * width / 2,
        cz - uz * length / 2 - pz * width / 2
    ]

    p4 = [
        cx - ux * length / 2 + px * width / 2,
        cz - uz * length / 2 + pz * width / 2
    ]

    return np.array([p1, p2, p3, p4])


def save_angle_files(t, angle_raw, angle_display, angle_smooth, angle_csv_path, angle_plot_path):
    out_df = pd.DataFrame({
        "time_s": t,
        "angle_raw_deg": angle_raw,
        "angle_zeroed_deg": angle_display,
        "angle_smooth_deg": angle_smooth
    })

    out_df.to_csv(angle_csv_path, index=False)

    fig, ax = plt.subplots(figsize=(10, 5))

    ax.plot(t, angle_display, label="Angle")
    ax.plot(t, angle_smooth, label="Smoothed angle")

    ax.axhline(0, linestyle="--", linewidth=1)

    ax.set_title("2D Angle Over Time")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Angle from vertical [deg]")
    ax.grid(True)
    ax.legend()

    fig.tight_layout()
    fig.savefig(angle_plot_path, dpi=150)
    plt.close(fig)


def make_animation(csv_path, out_path, fps, max_frames, alpha, dpi, axis_name,
                   zero_first, invert_angle, smooth_window, tower_length):
    df, t, R_mats, source, time_col = load_orientation(csv_path, alpha)

    angle_raw, angle_display, angle_smooth = compute_side_angle(
        R_mats=R_mats,
        axis_name=axis_name,
        zero_first=zero_first,
        invert_angle=invert_angle,
        smooth_window=smooth_window
    )

    default_gif, angle_csv_path, angle_plot_path = get_default_paths(csv_path)

    if out_path is None:
        out_path = default_gif
    else:
        out_path = Path(out_path)

    out_path.parent.mkdir(parents=True, exist_ok=True)

    save_angle_files(
        t=t,
        angle_raw=angle_raw,
        angle_display=angle_display,
        angle_smooth=angle_smooth,
        angle_csv_path=angle_csv_path,
        angle_plot_path=angle_plot_path
    )

    frame_idx = choose_frames(t, len(df), fps, max_frames)

    fig, ax = plt.subplots(figsize=(8.5, 7))
    fig.subplots_adjust(right=0.76, top=0.88)

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-1.25 * tower_length, 1.25 * tower_length)
    ax.set_ylim(-0.20 * tower_length, 1.25 * tower_length)

    ax.set_xlabel("Axis X")
    ax.set_ylabel("Axis Z")
    ax.set_title("2D Angle Replay")

    ax.grid(True)

    ax.plot(
        [-1.15 * tower_length, 1.15 * tower_length],
        [0, 0],
        linewidth=1,
        alpha=0.5
    )

    ax.plot(
        [0, 0],
        [0, 1.15 * tower_length],
        linestyle="--",
        linewidth=1.5,
        alpha=0.7,
        label="Vertical / Axis Z"
    )

    ax.text(
        0.72 * tower_length,
        1.12 * tower_length,
        "+Axis Y out of screen ⊙",
        fontsize=11
    )

    tower_line, = ax.plot([], [], linewidth=6, solid_capstyle="round", label="Sensor/tower axis")
    arc_line, = ax.plot([], [], linewidth=2)

    pivot_point, = ax.plot([0], [0], marker="o", markersize=8)

    start_angle = angle_smooth[frame_idx[0]]
    start_top_x = tower_length * np.sin(np.deg2rad(start_angle))
    start_top_z = tower_length * np.cos(np.deg2rad(start_angle))

    box_center_x = start_top_x - 0.11 * tower_length * np.sin(np.deg2rad(start_angle))
    box_center_z = start_top_z - 0.11 * tower_length * np.cos(np.deg2rad(start_angle))

    box_patch = Polygon(
        rotated_box(box_center_x, box_center_z, start_angle),
        closed=True,
        alpha=0.65,
        edgecolor="black"
    )

    ax.add_patch(box_patch)

    angle_text = fig.text(
        0.80,
        0.76,
        "",
        fontsize=13,
        va="top",
        bbox=dict(
            boxstyle="round",
            facecolor="white",
            edgecolor="black",
            alpha=0.90
        )
    )

    ax.legend(loc="lower right")

    def update(frame_number):
        i = frame_idx[frame_number]

        angle = angle_smooth[i]
        angle_rad = np.deg2rad(angle)

        top_x = tower_length * np.sin(angle_rad)
        top_z = tower_length * np.cos(angle_rad)

        tower_line.set_data([0, top_x], [0, top_z])

        box_center_x = top_x - 0.11 * tower_length * np.sin(angle_rad)
        box_center_z = top_z - 0.11 * tower_length * np.cos(angle_rad)

        box_patch.set_xy(rotated_box(box_center_x, box_center_z, angle))

        arc_radius = 0.30 * tower_length
        arc_angles = np.linspace(0, angle_rad, 60)

        arc_x = arc_radius * np.sin(arc_angles)
        arc_z = arc_radius * np.cos(arc_angles)

        arc_line.set_data(arc_x, arc_z)

        angle_text.set_text(
            f"Angle\n"
            f"{angle:7.2f}°\n\n"
            f"Time\n"
            f"{t[i]:7.2f} s\n\n"
            f"Frame\n"
            f"{frame_number + 1}/{len(frame_idx)}"
        )

        return tower_line, arc_line, box_patch, angle_text, pivot_point

    anim = FuncAnimation(
        fig,
        update,
        frames=len(frame_idx),
        interval=1000 / fps,
        blit=False
    )

    suffix = out_path.suffix.lower()

    print(f"Reading: {csv_path}")
    print(f"Rows: {len(df)}")
    print(f"Time column: {time_col}")
    print(f"Motion source: {source}")
    print(f"2D view: Axis X-Z plane")
    print(f"Axis Y: out of screen")
    print(f"Angle axis: local sensor {axis_name.upper()} axis")
    print(f"Animation frames: {len(frame_idx)}")
    print(f"Min angle: {np.nanmin(angle_smooth):.2f} deg")
    print(f"Max angle: {np.nanmax(angle_smooth):.2f} deg")
    print(f"Saving animation: {out_path}")
    print(f"Saving angle CSV: {angle_csv_path}")
    print(f"Saving angle plot: {angle_plot_path}")

    if suffix == ".mp4":
        writer = FFMpegWriter(fps=fps)
    else:
        writer = PillowWriter(fps=fps)

    def show_progress(current_frame, total_frames):
        total = total_frames if total_frames else len(frame_idx)
        current = current_frame + 1

        if current > total:
            current = total

        percent = 100.0 * current / total
        bar_length = 30
        filled = int(bar_length * current / total)
        bar = "#" * filled + "-" * (bar_length - filled)

        print(
            f"\rSaving animation: |{bar}| {percent:5.1f}% "
            f"({current}/{total})",
            end="",
            flush=True
        )

        if current >= total:
            print()
            print("Finalizing file... this can take a bit for GIFs.")

    anim.save(
        out_path,
        writer=writer,
        dpi=dpi,
        progress_callback=show_progress
    )

    plt.close(fig)

    print("Done.")


def main():
    parser = argparse.ArgumentParser(
        description="Replay IMU CSV motion as a 2D X-Z side-view animation and measure tilt angle."
    )

    parser.add_argument("csv", help="Path to CSV file")
    parser.add_argument("--out", default=None, help="Output .gif or .mp4 path")
    parser.add_argument("--fps", type=int, default=20, help="Animation frames per second")
    parser.add_argument("--max-frames", type=int, default=900, help="Max animation frames. Use 0 for no cap.")
    parser.add_argument("--alpha", type=float, default=0.98, help="Accel/gyro complementary filter weight")
    parser.add_argument("--dpi", type=int, default=120, help="Output resolution")

    parser.add_argument(
        "--axis",
        default="z",
        choices=["x", "y", "z"],
        help="Which local sensor axis points along the tower. Default: z"
    )

    parser.add_argument(
        "--absolute",
        action="store_true",
        help="Use absolute angle instead of setting the first frame to 0 degrees"
    )

    parser.add_argument(
        "--invert-angle",
        action="store_true",
        help="Flip positive/negative angle direction"
    )

    parser.add_argument(
        "--smooth-window",
        type=int,
        default=5,
        help="Moving average window for displayed angle"
    )

    parser.add_argument(
        "--tower-length",
        type=float,
        default=1.0,
        help="Display length of the 2D tower/sensor line"
    )

    args = parser.parse_args()

    csv_path = Path(args.csv)

    if not csv_path.exists():
        raise SystemExit(f"CSV file not found: {csv_path}")

    make_animation(
        csv_path=csv_path,
        out_path=args.out,
        fps=args.fps,
        max_frames=args.max_frames,
        alpha=args.alpha,
        dpi=args.dpi,
        axis_name=args.axis,
        zero_first=not args.absolute,
        invert_angle=args.invert_angle,
        smooth_window=args.smooth_window,
        tower_length=args.tower_length
    )


if __name__ == "__main__":
    main()
