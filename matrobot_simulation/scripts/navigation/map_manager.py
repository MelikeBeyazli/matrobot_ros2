#!/usr/bin/env python3
import math
import os
from pathlib import Path
from typing import Tuple
import yaml
from PIL import Image
from nav_msgs.msg import OccupancyGrid

GridCell = Tuple[int, int]
WorldPoint = Tuple[float, float]

class MapManager:
    def __init__(self, map_yaml_path: str, robot_radius_m: float, publish_inflated_map: bool = False):
        self.map_yaml_path = Path(map_yaml_path)
        self.robot_radius_m = robot_radius_m
        self.publish_inflated_map = publish_inflated_map
        self.load_map()

    def load_map(self):
        with open(self.map_yaml_path, "r") as file:
            map_info = yaml.safe_load(file)
        image_path = map_info["image"]
        if not os.path.isabs(image_path):
            image_path = self.map_yaml_path.parent / image_path
        self.resolution = float(map_info["resolution"])
        self.origin_x = float(map_info["origin"][0])
        self.origin_y = float(map_info["origin"][1])
        self.occupied_thresh = float(map_info.get("occupied_thresh", 0.65))
        self.negate = int(map_info.get("negate", 0))
        image = Image.open(image_path).convert("L")
        self.map_width, self.map_height = image.size
        pixels = image.load()
        self.raw_grid = [[0 for _ in range(self.map_height)] for _ in range(self.map_width)]
        for mx in range(self.map_width):
            for my in range(self.map_height):
                pixel = pixels[mx, my] / 255.0
                occ = 1.0 - pixel if self.negate == 0 else pixel
                self.raw_grid[mx][my] = 1 if occ > self.occupied_thresh else 0
        self.robot_radius_cells = max(1, int(math.ceil(self.robot_radius_m / self.resolution)))
        self.static_inflated_grid = [col[:] for col in self.raw_grid]
        self.inflate_static_obstacles()
        self.clear_dynamic_layer()

    def clear_dynamic_layer(self):
        self.dynamic_grid = [[0 for _ in range(self.map_height)] for _ in range(self.map_width)]
        self.rebuild_planning_grid()

    def rebuild_planning_grid(self):
        self.planning_grid = [col[:] for col in self.static_inflated_grid]
        for gx in range(self.map_width):
            for gy in range(self.map_height):
                if self.dynamic_grid[gx][gy] == 1:
                    self.planning_grid[gx][gy] = 1

    def inflate_static_obstacles(self):
        occupied = []
        for gx in range(self.map_width):
            for gy in range(self.map_height):
                if self.raw_grid[gx][gy] == 1:
                    occupied.append((gx, gy))
        for gx, gy in occupied:
            self.inflate_cell_into_grid(gx, gy, self.static_inflated_grid)

    def inflate_cell_into_grid(self, gx: int, gy: int, grid, radius_cells=None):
        radius = self.robot_radius_cells if radius_cells is None else max(1, int(radius_cells))
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if dx * dx + dy * dy > radius * radius:
                    continue
                nx, ny = gx + dx, gy + dy
                if self.in_bounds((nx, ny)):
                    grid[nx][ny] = 1

    def add_dynamic_obstacle_world(self, wx: float, wy: float):
        gx, gy = self.world_to_grid(wx, wy)
        if self.in_bounds((gx, gy)):
            self.inflate_cell_into_grid(gx, gy, self.dynamic_grid)
            self.rebuild_planning_grid()

    def add_dynamic_block_world(self, wx: float, wy: float, radius_cells: int):
        gx, gy = self.world_to_grid(wx, wy)
        if self.in_bounds((gx, gy)):
            self.inflate_cell_into_grid(gx, gy, self.dynamic_grid, radius_cells=radius_cells)
            self.rebuild_planning_grid()

    def world_to_grid(self, wx: float, wy: float) -> GridCell:
        gx = int((wx - self.origin_x) / self.resolution)
        gy_from_bottom = int((wy - self.origin_y) / self.resolution)
        gy = self.map_height - 1 - gy_from_bottom
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> WorldPoint:
        wx = self.origin_x + gx * self.resolution
        gy_from_bottom = self.map_height - 1 - gy
        wy = self.origin_y + gy_from_bottom * self.resolution
        return wx, wy

    def in_bounds(self, cell: GridCell) -> bool:
        gx, gy = cell
        return 0 <= gx < self.map_width and 0 <= gy < self.map_height

    def is_free(self, cell: GridCell) -> bool:
        gx, gy = cell
        return self.in_bounds(cell) and self.planning_grid[gx][gy] == 0

    def build_occupancy_grid_msg(self, frame_id: str, stamp) -> OccupancyGrid:
        grid_to_publish = self.static_inflated_grid if self.publish_inflated_map else self.raw_grid
        msg = OccupancyGrid()
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp
        msg.info.resolution = self.resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        data = []
        for gy_from_bottom in range(self.map_height):
            image_y = self.map_height - 1 - gy_from_bottom
            for gx in range(self.map_width):
                data.append(100 if grid_to_publish[gx][image_y] == 1 else 0)
        msg.data = data
        return msg
