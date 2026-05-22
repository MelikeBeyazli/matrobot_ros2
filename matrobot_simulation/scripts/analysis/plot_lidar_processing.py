#!/usr/bin/env python3
"""
Creates academic LiDAR processing plots.

Input:
  ~/matrobot_ws/src/matrobot_ros2/matrobot_simulation/logs/lidar_snapshot.csv

Output:
  ~/matrobot_ws/src/matrobot_ros2/matrobot_simulation/plots/academic/fig07_lidar_raw_filtered.png
"""

from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def configure_academic_style():
    plt.rcParams.update({
        "figure.figsize": (7.2, 5.8),
        "figure.dpi": 120,
        "savefig.dpi": 300,
        "font.size": 10,
        "axes.titlesize": 11,
        "axes.labelsize": 10,
        "legend.fontsize": 8,
        "axes.grid": True,
        "grid.linestyle": "--",
        "grid.linewidth": 0.5,
    })


def main():
    configure_academic_style()

    package_root = (
        Path.home()
        / "matrobot_ws"
        / "src"
        / "matrobot_ros2"
        / "matrobot_simulation"
    )

    input_file = package_root / "logs" / "lidar_snapshot.csv"
    output_dir = package_root / "plots" / "academic"
    output_dir.mkdir(parents=True, exist_ok=True)

    if not input_file.exists():
        raise FileNotFoundError(
            f"Missing LiDAR snapshot: {input_file}\\n"
            "Run lidar_snapshot_recorder.py first."
        )

    df = pd.read_csv(input_file)
    angle = df["angle_rad"].to_numpy()
    raw = pd.to_numeric(df["raw_range_m"], errors="coerce").to_numpy()
    filtered = pd.to_numeric(df["filtered_range_m"], errors="coerce").to_numpy()

    fig, axes = plt.subplots(1, 2, figsize=(9.0, 4.2))

    axes[0].plot(df["angle_deg"], raw, linestyle="-", label="Raw LiDAR ranges")
    axes[0].plot(df["angle_deg"], filtered, linestyle="--", label="Filtered valid ranges")
    axes[0].set_title("LiDAR Range Data")
    axes[0].set_xlabel("Scan angle [deg]")
    axes[0].set_ylabel("Range [m]")
    axes[0].legend(loc="best")

    x_raw = raw * np.cos(angle)
    y_raw = raw * np.sin(angle)
    x_filtered = filtered * np.cos(angle)
    y_filtered = filtered * np.sin(angle)

    axes[1].scatter(x_raw, y_raw, s=8, marker=".", label="Raw scan points")
    axes[1].scatter(x_filtered, y_filtered, s=8, marker="x", label="Filtered scan points")
    axes[1].set_title("LiDAR Scan in Robot Frame")
    axes[1].set_xlabel("x [m]")
    axes[1].set_ylabel("y [m]")
    axes[1].axis("equal")
    axes[1].legend(loc="best")

    plt.tight_layout()
    plt.savefig(output_dir / "fig07_lidar_raw_filtered.png", bbox_inches="tight")
    plt.close()

    print(f"LiDAR figure saved to: {output_dir / 'fig07_lidar_raw_filtered.png'}")


if __name__ == "__main__":
    main()
