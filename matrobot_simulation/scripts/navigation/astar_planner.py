#!/usr/bin/env python3
import heapq
import math
from typing import Dict, List, Optional, Tuple
from navigation.map_manager import GridCell, MapManager, WorldPoint

class AStarPlanner:
    def __init__(self, map_manager: MapManager):
        self.map = map_manager

    def plan(self, start_world: WorldPoint, goal_world: WorldPoint) -> List[WorldPoint]:
        start = self.find_nearest_free_cell(self.map.world_to_grid(*start_world))
        goal = self.find_nearest_free_cell(self.map.world_to_grid(*goal_world))
        if start is None or goal is None:
            return []
        open_set: List[Tuple[float, GridCell]] = []
        heapq.heappush(open_set, (0.0, start))
        came_from: Dict[GridCell, GridCell] = {}
        g_score: Dict[GridCell, float] = {start: 0.0}
        visited = set()
        while open_set:
            _, current = heapq.heappop(open_set)
            if current in visited:
                continue
            visited.add(current)
            if current == goal:
                return self.reconstruct_path(came_from, current)
            for neighbor, move_cost in self.get_neighbors(current):
                tentative_g = g_score[current] + move_cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))
        return []

    def get_neighbors(self, cell: GridCell) -> List[Tuple[GridCell, float]]:
        gx, gy = cell
        motions = [(1,0,1.0),(-1,0,1.0),(0,1,1.0),(0,-1,1.0),(1,1,1.414),(1,-1,1.414),(-1,1,1.414),(-1,-1,1.414)]
        return [((gx+dx, gy+dy), cost) for dx, dy, cost in motions if self.map.is_free((gx+dx, gy+dy))]

    def find_nearest_free_cell(self, cell: GridCell, max_radius: int = 80) -> Optional[GridCell]:
        if self.map.is_free(cell):
            return cell
        gx, gy = cell
        for radius in range(1, max_radius + 1):
            candidates = []
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    candidate = (gx + dx, gy + dy)
                    if self.map.is_free(candidate):
                        candidates.append((math.hypot(dx, dy), candidate))
            if candidates:
                candidates.sort(key=lambda item: item[0])
                return candidates[0][1]
        return None

    def reconstruct_path(self, came_from: Dict[GridCell, GridCell], current: GridCell) -> List[WorldPoint]:
        grid_path = [current]
        while current in came_from:
            current = came_from[current]
            grid_path.append(current)
        grid_path.reverse()
        return [self.map.grid_to_world(gx, gy) for gx, gy in grid_path]

    @staticmethod
    def heuristic(a: GridCell, b: GridCell) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])
