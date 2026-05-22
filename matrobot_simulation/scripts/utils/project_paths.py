#!/usr/bin/env python3
from pathlib import Path

class ProjectPaths:
    def __init__(self):
        self.workspace_root = Path.home() / "matrobot_ws"
        self.package_root = self.workspace_root / "src" / "matrobot_ros2" / "matrobot_simulation"
        self.map_dir = self.package_root / "map"
        self.log_dir = self.package_root / "logs"
        self.plot_dir = self.package_root / "plots"
        self.default_map_yaml = self.map_dir / "default_map.yaml"
        self.navigation_log = self.log_dir / "bt_astar_navigation_log.csv"
        self.ensure_directories()

    def ensure_directories(self):
        self.map_dir.mkdir(parents=True, exist_ok=True)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.plot_dir.mkdir(parents=True, exist_ok=True)
