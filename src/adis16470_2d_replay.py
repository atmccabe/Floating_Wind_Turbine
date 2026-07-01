#!/usr/bin/env python3

import argparse
import re
from pathlib import Path

import numpy as np
import pandas as pd

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter, PillowWriter
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
    time_col = find_col(df, ["time_s", "time", "t_s", "t", "seconds", "sec"])

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

    return t - t[0], time_col


def angle_to_deg(values, col_name):
    name = clean_name(col_name)

    if "rad" in name:
        return np.rad2deg(values)

    return values


def smooth(values, window):
    if window <= 1:
        return values

    return (
        pd.Series(values)
        .rolling(window=window, center=True, min_periods=1)
        .mean()
        .to_numpy()
    )


def plane_info(plane):
    plane = plane.lower()

    if plane == "xy":
        return "X", "Y", "Z"

    if plane == "xz":
        return "X", "Z", "Y"

    if plane == "yz":
        return "Y", "Z", "X"

    raise ValueError("plane must be xy, xz, or yz")


def choose_frames(n, max_frames):
    if n <= 1:
        return np.array([0])

    if max_frames == 0:
        return np.arange(n)

    return np.linspace(0, n - 1, max_frames).astype(int)


def default_paths(csv_path):
    csv_path = Path(csv_path)

    if csv_path.parent.name == "raw" and csv_path.parent.parent.name == "data":
        sensor_dir = csv_path.parent.parent.parent
        animation_dir = sensor_dir / "animations"
        processed_dir = sensor_dir / "data" / "processed"
        plot_dir = sensor_dir / "plots"
    else:
        animation_dir = csv_path.parent / "animations"
        processed_dir = csv_path.parent / "processed"
        plot_dir = csv_path.parent / "plots"

    animation_dir.mkdir(parents=True, exist_ok=True)
    processed_dir.mkdir(parents=True, exist_ok=True)
    plot_dir.mkdir(parents=True, exist_ok=True)

    video_path = animation_dir / f"{csv_path.stem}_adis_2d.mp4"
    angle_csv_path = processed_dir / f"{csv_path.stem}_adis_2d_angle.csv"
    angle_plot_path = plot_dir / f"{csv_path.stem}_adis_2d_angle.png"

    return video_path, angle_csv_path, angle_plot_path


def rotated_rect(center_h, center_v, angle_deg, length=0.25, width=0.12):
    a = np.deg2rad(angle_deg)

    uh = np.sin(a)
    uv = np.cos(a)

    ph = np.cos(a)
    pv = -np.sin(a)

    p1 = [
        center_h + uh * length / 2 + ph * width / 2,
        center_v + uv * length / 2 + pv * width / 2
    ]

    p2 = [
        center_h + uh * length / 2 - ph * width / 2,
        center_v + uv * length / 2 - pv * width / 2
    ]

    p3 = [
        center_h - uh * length / 2 - ph * width / 2,
        center_v - uv * length / 2 - pv * width / 2
    ]

    p4 = [
        center_h - uh * length / 2 + ph * width / 2,
        center_v - uv * length / 2 + pv * width / 2
    ]

    return np.array([p1, p2, p3, p4])


def save_angle_outputs(t, raw_angle, display_angle, smooth_angle, angle_csv_path, angle_plot_path):
    out_df = pd.DataFrame({
        "time_s": t,
        "angle_raw_deg": raw_angle,
        "angle_display_deg": display_angle,
        "angle_smooth_deg": smooth_angle
    })

    out_df.to_csv(angle_csv_path, index=False)

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(t, display_angle, label="Angle")
    ax.plot(t, smooth_angle, label="Smoothed angle")
    ax.axhline(0, linestyle="--", linewidth=1)
    ax.set_title("ADIS16470 2D Angle")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Angle [deg]")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(angle_plot_path, dpi=150)
    plt.close(fig)


def make_animation(csv_path, out_path, fps, max_frames, angle_column, plane,
                   zero_first, invert_angle, smooth_window, tower_length, dpi):
    csv_path = Path(csv_path)
    df = pd.read_csv(csv_path)
    t, time_col = get_time(df)

    angle_col = find_col(df, [angle_column])

    if angle_col is None:
        raise SystemExit(
            f"\nAngle column not found: {angle_column}\n"
            f"Columns found:\n{list(df.columns)}\n"
        )

    raw_angle = angle_to_deg(read_num(df, angle_col), angle_col)

    if zero_first:
        display_angle = raw_angle - raw_angle[0]
    else:
        display_angle = raw_angle.copy()

    if invert_angle:
        raw_angle = -raw_angle
        display_angle = -display_angle

    smooth_angle = smooth(display_angle, smooth_window)

    default_video, angle_csv_path, angle_plot_path = default_paths(csv_path)

    if out_path is None:
        out_path = default_video
    else:
        out_path = Path(out_path)

    out_path.parent.mkdir(parents=True, exist_ok=True)

    save_angle_outputs(
        t=t,
        raw_angle=raw_angle,
        display_angle=display_angle,
        smooth_angle=smooth_angle,
        angle_csv_path=angle_csv_path,
        angle_plot_path=angle_plot_path
    )

    frame_idx = choose_frames(len(df), max_frames)

    horizontal_name, vertical_name, out_name = plane_info(plane)

    fig, ax = plt.subplots(figsize=(8.5, 7))
    fig.subplots_adjust(right=0.76, top=0.88)

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-1.25 * tower_length, 1.25 * tower_length)
    ax.set_ylim(-0.20 * tower_length, 1.25 * tower_length)

    ax.set_xlabel(f"Axis {horizontal_name}")
    ax.set_ylabel(f"Axis {vertical_name}")
    ax.set_title("ADIS 2D Replay")
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
        label=f"Vertical / Axis {vertical_name}"
    )

    ax.text(
        0.65 * tower_length,
        1.12 * tower_length,
        f"+Axis {out_name} out of screen ⊙",
        fontsize=11
    )

    tower_line, = ax.plot([], [], linewidth=6, solid_capstyle="round", label="Sensor/tower angle")
    arc_line, = ax.plot([], [], linewidth=2)
    pivot_point, = ax.plot([0], [0], marker="o", markersize=8)

    board_patch = Polygon(
        rotated_rect(0, 0.85 * tower_length, 0),
        closed=True,
        alpha=0.65,
        edgecolor="black"
    )
    ax.add_patch(board_patch)

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

        angle = smooth_angle[i]
        angle_rad = np.deg2rad(angle)

        top_h = tower_length * np.sin(angle_rad)
        top_v = tower_length * np.cos(angle_rad)

        tower_line.set_data([0, top_h], [0, top_v])

        board_h = top_h - 0.10 * tower_length * np.sin(angle_rad)
        board_v = top_v - 0.10 * tower_length * np.cos(angle_rad)

        board_patch.set_xy(rotated_rect(board_h, board_v, angle))

        arc_radius = 0.30 * tower_length
        arc_angles = np.linspace(0, angle_rad, 60)
        arc_h = arc_radius * np.sin(arc_angles)
        arc_v = arc_radius * np.cos(arc_angles)
        arc_line.set_data(arc_h, arc_v)

        angle_text.set_text(
            f"Angle\n"
            f"{angle:7.2f}°\n\n"
            f"Time\n"
            f"{t[i]:7.2f} s\n\n"
            f"Frame\n"
            f"{frame_number + 1}/{len(frame_idx)}"
        )

        return tower_line, arc_line, board_patch, angle_text, pivot_point

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
    print(f"Angle column: {angle_col}")
    print(f"2D plane: Axis {horizontal_name}-{vertical_name}")
    print(f"Axis {out_name}: out of screen")
    print(f"Animation frames: {len(frame_idx)}")
    print(f"Min angle: {np.nanmin(smooth_angle):.2f} deg")
    print(f"Max angle: {np.nanmax(smooth_angle):.2f} deg")
    print(f"Saving video: {out_path}")
    print(f"Saving angle CSV: {angle_csv_path}")
    print(f"Saving angle plot: {angle_plot_path}")

    if suffix == ".gif":
        writer = PillowWriter(fps=fps)
    else:
        writer = FFMpegWriter(fps=fps)

    def show_progress(current_frame, total_frames):
        total = total_frames if total_frames else len(frame_idx)
        current = min(current_frame + 1, total)

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
            print("Finalizing file...")

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
        description="ADIS16470-only 2D replay from CSV angle columns."
    )

    parser.add_argument("csv", help="Path to ADIS16470 CSV file")
    parser.add_argument("--out", default=None, help="Output .mp4 or .gif path")
    parser.add_argument("--fps", type=int, default=20, help="Video FPS")
    parser.add_argument("--max-frames", type=int, default=0, help="0 = use full CSV rows")
    parser.add_argument("--angle-column", default="pitch_deg", help="Column to animate, like pitch_deg, roll_deg, accel_pitch_deg")
    parser.add_argument("--plane", default="xy", choices=["xy", "xz", "yz"], help="2D display plane")
    parser.add_argument("--zero-first", action="store_true", help="Set first angle value to 0 degrees")
    parser.add_argument("--invert-angle", action="store_true", help="Flip angle sign")
    parser.add_argument("--smooth-window", type=int, default=5, help="Moving average smoothing window")
    parser.add_argument("--tower-length", type=float, default=1.0, help="Display length")
    parser.add_argument("--dpi", type=int, default=120, help="Video resolution")

    args = parser.parse_args()

    make_animation(
        csv_path=args.csv,
        out_path=args.out,
        fps=args.fps,
        max_frames=args.max_frames,
        angle_column=args.angle_column,
        plane=args.plane,
        zero_first=args.zero_first,
        invert_angle=args.invert_angle,
        smooth_window=args.smooth_window,
        tower_length=args.tower_length,
        dpi=args.dpi
    )


if __name__ == "__main__":
    main()
