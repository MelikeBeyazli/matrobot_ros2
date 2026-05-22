#!/usr/bin/env python3
"""
Academic plotting suite for the Mobile Robots assignment.

This script creates report-ready figures:

Figure 1  : 2D environment map with start, goal, planned path and executed path
Figure 2  : Planned path vs executed trajectory
Figure 3  : Navigation performance time series
Figure 4  : Behavior Tree and recovery state timeline
Figure 5  : Localization XY comparison, if localization CSV exists
Figure 6  : Localization error analysis, if ground-truth columns exist
Table CSV : navigation_academic_metrics.csv

Expected default inputs:
  map:
    ~/matrobot_ws/src/matrobot_ros2/matrobot_simulation/map/default_map.yaml

  navigation log:
    ~/matrobot_ws/src/matrobot_ros2/matrobot_simulation/logs/bt_astar_navigation_log.csv

  optional localization log:
    ~/matrobot_ws/src/matrobot_ros2/matrobot_simulation/logs/ekf_ukf_synchronized_analysis.csv

Outputs:
  ~/matrobot_ws/src/matrobot_ros2/matrobot_simulation/plots/academic/
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple

import math
import yaml
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from PIL import Image


@dataclass
class ProjectPaths:
    package_root: Path = Path.home() / "matrobot_ws" / "src" / "matrobot_ros2" / "matrobot_simulation"

    @property
    def map_yaml(self) -> Path:
        return self.package_root / "map" / "default_map.yaml"

    @property
    def nav_log(self) -> Path:
        return self.package_root / "logs" / "bt_astar_navigation_log.csv"

    @property
    def localization_log(self) -> Path:
        return self.package_root / "logs" / "ekf_ukf_synchronized_analysis.csv"

    @property
    def output_dir(self) -> Path:
        path = self.package_root / "plots" / "academic"
        path.mkdir(parents=True, exist_ok=True)
        return path


def configure_academic_style() -> None:
    plt.rcParams.update({
        "figure.figsize": (7.2, 5.2),
        "figure.dpi": 120,
        "savefig.dpi": 300,
        "font.size": 10,
        "axes.titlesize": 11,
        "axes.labelsize": 10,
        "legend.fontsize": 8,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "axes.grid": True,
        "grid.linestyle": "--",
        "grid.linewidth": 0.5,
        "lines.linewidth": 1.6,
    })


def save_figure(path: Path) -> None:
    plt.tight_layout()
    plt.savefig(path, bbox_inches="tight")
    plt.close()


def read_navigation_log(path: Path) -> pd.DataFrame:
    if not path.exists():
        raise FileNotFoundError(f"Navigation log not found: {path}")

    df = pd.read_csv(path)
    required = ["time", "x", "y", "yaw", "distance_to_goal", "front_distance"]

    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"Navigation log missing columns: {missing}")

    df = df.dropna(subset=["time", "x", "y"])
    df["time"] = df["time"] - df["time"].iloc[0]
    return df


def load_map_from_yaml(map_yaml: Path):
    if not map_yaml.exists():
        return None

    with open(map_yaml, "r") as f:
        info = yaml.safe_load(f)

    image_path = Path(info["image"])
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path

    if not image_path.exists():
        return None

    image = Image.open(image_path).convert("L")
    image_array = np.array(image)

    resolution = float(info["resolution"])
    origin = info["origin"]
    origin_x = float(origin[0])
    origin_y = float(origin[1])

    height, width = image_array.shape

    extent = [
        origin_x,
        origin_x + width * resolution,
        origin_y,
        origin_y + height * resolution,
    ]

    return image_array, extent


def draw_map_background(map_data):
    if map_data is None:
        return

    image_array, extent = map_data

    # Use gray colormap only; no custom colors are hardcoded.
    plt.imshow(
        np.flipud(image_array),
        cmap="gray",
        extent=extent,
        origin="lower",
        alpha=0.75,
    )


def estimate_start_goal(df: pd.DataFrame) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    start = (float(df["x"].iloc[0]), float(df["y"].iloc[0]))

    if "goal_x" in df.columns and "goal_y" in df.columns:
        goal = (float(df["goal_x"].iloc[0]), float(df["goal_y"].iloc[0]))
    else:
        goal = (float(df["x"].iloc[-1]), float(df["y"].iloc[-1]))

    return start, goal


def plot_environment_overview(df: pd.DataFrame, map_data, output_dir: Path) -> None:
    start, goal = estimate_start_goal(df)

    plt.figure(figsize=(7.4, 6.0))
    draw_map_background(map_data)

    plt.plot(df["x"], df["y"], linestyle="-", label="Executed trajectory")
    plt.scatter([start[0]], [start[1]], marker="o", s=55, label="Start")
    plt.scatter([goal[0]], [goal[1]], marker="*", s=95, label="Goal")

    plt.title("2D Environment Map and Robot Trajectory")
    plt.xlabel("x position [m]")
    plt.ylabel("y position [m]")
    plt.axis("equal")
    plt.legend(loc="best")
    save_figure(output_dir / "fig01_environment_map_trajectory.png")


def plot_path_tracking(df: pd.DataFrame, map_data, output_dir: Path) -> None:
    start, goal = estimate_start_goal(df)

    plt.figure(figsize=(7.4, 6.0))
    draw_map_background(map_data)

    # Executed path from odometry/fusion log
    plt.plot(df["x"], df["y"], linestyle="-", label="Executed path")

    # Planned path is not logged as all waypoints in the basic CSV.
    # This plot still shows the final target and executed trajectory.
    # If planned path waypoints are logged later, they can be overlaid here.
    plt.plot([start[0], goal[0]], [start[1], goal[1]], linestyle="--", label="Start-goal reference")

    plt.scatter([start[0]], [start[1]], marker="o", s=55, label="Start")
    plt.scatter([goal[0]], [goal[1]], marker="*", s=95, label="Goal")

    plt.title("Path Tracking Result")
    plt.xlabel("x position [m]")
    plt.ylabel("y position [m]")
    plt.axis("equal")
    plt.legend(loc="best")
    save_figure(output_dir / "fig02_path_tracking_result.png")


def plot_navigation_time_series(df: pd.DataFrame, output_dir: Path) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(7.2, 8.0), sharex=True)

    axes[0].plot(df["time"], df["distance_to_goal"], label="Distance to goal")
    axes[0].set_title("Goal Convergence")
    axes[0].set_ylabel("Distance [m]")
    axes[0].legend(loc="best")

    axes[1].plot(df["time"], df["front_distance"], label="Front LiDAR distance")
    axes[1].set_title("Obstacle Proximity")
    axes[1].set_ylabel("Distance [m]")
    axes[1].legend(loc="best")

    if "yaw_error_to_goal" in df.columns:
        axes[2].plot(df["time"], df["yaw_error_to_goal"], label="Goal yaw error")
    else:
        axes[2].plot(df["time"], df["yaw"], label="Robot yaw")
    axes[2].set_title("Final Orientation Behavior")
    axes[2].set_xlabel("Time [s]")
    axes[2].set_ylabel("Angle [rad]")
    axes[2].legend(loc="best")

    save_figure(output_dir / "fig03_navigation_time_series.png")


def encode_categories(series: pd.Series) -> Tuple[pd.Series, Dict[str, int]]:
    categories = {name: idx for idx, name in enumerate(sorted(series.dropna().astype(str).unique()))}
    encoded = series.astype(str).map(categories)
    return encoded, categories


def plot_bt_timeline(df: pd.DataFrame, output_dir: Path) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(7.2, 6.0), sharex=True)

    if "bt_state" in df.columns:
        encoded, categories = encode_categories(df["bt_state"])
        axes[0].step(df["time"], encoded, where="post", label="BT state")
        axes[0].set_yticks(list(categories.values()))
        axes[0].set_yticklabels(list(categories.keys()))
        axes[0].set_title("Behavior Tree State Timeline")
        axes[0].set_ylabel("BT state")
        axes[0].legend(loc="best")

    if "recovery_phase" in df.columns:
        encoded, categories = encode_categories(df["recovery_phase"])
        axes[1].step(df["time"], encoded, where="post", label="Recovery phase")
        axes[1].set_yticks(list(categories.values()))
        axes[1].set_yticklabels(list(categories.keys()))
        axes[1].set_title("Recovery Behavior Timeline")
        axes[1].set_xlabel("Time [s]")
        axes[1].set_ylabel("Recovery phase")
        axes[1].legend(loc="best")

    save_figure(output_dir / "fig04_bt_recovery_timeline.png")


def find_column(df: pd.DataFrame, candidates) -> Optional[str]:
    lower_map = {c.lower(): c for c in df.columns}
    for candidate in candidates:
        if candidate.lower() in lower_map:
            return lower_map[candidate.lower()]
    return None


def load_localization(path: Path) -> Optional[pd.DataFrame]:
    if not path.exists():
        return None

    df = pd.read_csv(path)
    if len(df) == 0:
        return None

    return df


def plot_localization_results(loc_df: pd.DataFrame, output_dir: Path) -> None:
    time_col = find_column(loc_df, ["time", "timestamp", "t", "sec"])
    if time_col is None:
        loc_df = loc_df.copy()
        loc_df["sample_index"] = np.arange(len(loc_df))
        time_col = "sample_index"

    ekf_x = find_column(loc_df, ["ekf_x", "x_ekf", "ekf_pose_x"])
    ekf_y = find_column(loc_df, ["ekf_y", "y_ekf", "ekf_pose_y"])

    ukf_x = find_column(loc_df, ["ukf_x", "x_ukf", "ukf_pose_x"])
    ukf_y = find_column(loc_df, ["ukf_y", "y_ukf", "ukf_pose_y"])

    gt_x = find_column(loc_df, ["gt_x", "ground_truth_x", "truth_x", "x_gt", "gazebo_x"])
    gt_y = find_column(loc_df, ["gt_y", "ground_truth_y", "truth_y", "y_gt", "gazebo_y"])

    odom_x = find_column(loc_df, ["odom_x", "x_odom", "wheel_odom_x"])
    odom_y = find_column(loc_df, ["odom_y", "y_odom", "wheel_odom_y"])

    plt.figure(figsize=(7.4, 6.0))

    if gt_x and gt_y:
        plt.plot(loc_df[gt_x], loc_df[gt_y], linestyle="-", label="Ground truth")
    if odom_x and odom_y:
        plt.plot(loc_df[odom_x], loc_df[odom_y], linestyle=":", label="Dead reckoning / odometry")
    if ekf_x and ekf_y:
        plt.plot(loc_df[ekf_x], loc_df[ekf_y], linestyle="--", label="EKF estimate")
    if ukf_x and ukf_y:
        plt.plot(loc_df[ukf_x], loc_df[ukf_y], linestyle="-.", label="UKF estimate")

    plt.title("Localization Trajectory Comparison")
    plt.xlabel("x position [m]")
    plt.ylabel("y position [m]")
    plt.axis("equal")
    plt.legend(loc="best")
    save_figure(output_dir / "fig05_localization_xy_comparison.png")

    if gt_x and gt_y:
        plt.figure(figsize=(7.2, 5.2))

        if odom_x and odom_y:
            odom_error = np.sqrt((loc_df[odom_x] - loc_df[gt_x]) ** 2 + (loc_df[odom_y] - loc_df[gt_y]) ** 2)
            plt.plot(loc_df[time_col], odom_error, linestyle=":", label=f"Odometry error, RMSE={rmse(odom_error):.3f} m")

        if ekf_x and ekf_y:
            ekf_error = np.sqrt((loc_df[ekf_x] - loc_df[gt_x]) ** 2 + (loc_df[ekf_y] - loc_df[gt_y]) ** 2)
            plt.plot(loc_df[time_col], ekf_error, linestyle="--", label=f"EKF error, RMSE={rmse(ekf_error):.3f} m")

        if ukf_x and ukf_y:
            ukf_error = np.sqrt((loc_df[ukf_x] - loc_df[gt_x]) ** 2 + (loc_df[ukf_y] - loc_df[gt_y]) ** 2)
            plt.plot(loc_df[time_col], ukf_error, linestyle="-.", label=f"UKF error, RMSE={rmse(ukf_error):.3f} m")

        plt.title("Localization Position Error")
        plt.xlabel("Time / sample")
        plt.ylabel("Position error [m]")
        plt.legend(loc="best")
        save_figure(output_dir / "fig06_localization_error_analysis.png")


def rmse(values) -> float:
    values = np.asarray(values)
    return float(np.sqrt(np.nanmean(values ** 2)))


def mae(values) -> float:
    values = np.asarray(values)
    return float(np.nanmean(np.abs(values)))


def compute_navigation_metrics(df: pd.DataFrame) -> pd.DataFrame:
    duration_s = float(df["time"].iloc[-1] - df["time"].iloc[0]) if len(df) > 1 else 0.0

    dx = df["x"].diff().fillna(0.0)
    dy = df["y"].diff().fillna(0.0)
    travelled_distance_m = float(np.sqrt(dx ** 2 + dy ** 2).sum())

    final_distance_to_goal_m = float(df["distance_to_goal"].iloc[-1])
    min_distance_to_goal_m = float(df["distance_to_goal"].min())
    mean_front_distance_m = float(df["front_distance"].mean())
    min_front_distance_m = float(df["front_distance"].min())

    recovery_samples = 0
    if "recovery_phase" in df.columns:
        recovery_samples = int((df["recovery_phase"].astype(str) != "IDLE").sum())

    bt_states = ""
    if "bt_state" in df.columns:
        bt_states = ", ".join(sorted(df["bt_state"].dropna().astype(str).unique()))

    return pd.DataFrame([{
        "duration_s": duration_s,
        "travelled_distance_m": travelled_distance_m,
        "final_distance_to_goal_m": final_distance_to_goal_m,
        "min_distance_to_goal_m": min_distance_to_goal_m,
        "mean_front_distance_m": mean_front_distance_m,
        "min_front_distance_m": min_front_distance_m,
        "recovery_samples": recovery_samples,
        "bt_states_observed": bt_states,
        "sample_count": int(len(df)),
    }])


def main():
    configure_academic_style()
    paths = ProjectPaths()

    nav_df = read_navigation_log(paths.nav_log)
    map_data = load_map_from_yaml(paths.map_yaml)

    plot_environment_overview(nav_df, map_data, paths.output_dir)
    plot_path_tracking(nav_df, map_data, paths.output_dir)
    plot_navigation_time_series(nav_df, paths.output_dir)
    plot_bt_timeline(nav_df, paths.output_dir)

    metrics = compute_navigation_metrics(nav_df)
    metrics_path = paths.output_dir / "navigation_academic_metrics.csv"
    metrics.to_csv(metrics_path, index=False)

    loc_df = load_localization(paths.localization_log)
    if loc_df is not None:
        plot_localization_results(loc_df, paths.output_dir)

    print(f"Academic figures saved to: {paths.output_dir}")
    print(f"Metrics saved to: {metrics_path}")
    print(metrics.to_string(index=False))


if __name__ == "__main__":
    main()
