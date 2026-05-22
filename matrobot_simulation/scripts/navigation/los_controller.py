#!/usr/bin/env python3
import math
from typing import List, Optional, Tuple
from geometry_msgs.msg import Twist
WorldPoint = Tuple[float, float]

class LOSController:
    def __init__(self, lookahead_distance: float, normal_speed: float, slow_distance: float, stop_distance: float, yaw_gain: float, max_angular_speed: float):
        self.lookahead_distance = lookahead_distance
        self.normal_speed = normal_speed
        self.slow_distance = slow_distance
        self.stop_distance = stop_distance
        self.yaw_gain = yaw_gain
        self.max_angular_speed = max_angular_speed
        self.current_wp_index = 0

    def reset(self):
        self.current_wp_index = 0

    def find_target(self, path: List[WorldPoint], x: float, y: float) -> Optional[WorldPoint]:
        if not path:
            return None
        closest_index = self.current_wp_index
        closest_distance = float('inf')
        for i in range(self.current_wp_index, len(path)):
            px, py = path[i]
            d = math.hypot(px - x, py - y)
            if d < closest_distance:
                closest_distance = d
                closest_index = i
        target_index = closest_index
        for i in range(closest_index, len(path)):
            px, py = path[i]
            if math.hypot(px - x, py - y) >= self.lookahead_distance:
                target_index = i
                break
        self.current_wp_index = closest_index
        return path[target_index]

    def compute_cmd(self, path: List[WorldPoint], x: float, y: float, yaw: float, front_distance: float) -> Twist:
        cmd = Twist()
        target = self.find_target(path, x, y)
        if target is None:
            return cmd
        tx, ty = target
        target_yaw = math.atan2(ty - y, tx - x)
        yaw_error = self.normalize_angle(target_yaw - yaw)
        heading_factor = max(0.0, 1.0 - abs(yaw_error))
        if front_distance >= self.slow_distance:
            speed_factor = 1.0
        else:
            speed_factor = (front_distance - self.stop_distance) / (self.slow_distance - self.stop_distance)
            speed_factor = self.clamp(speed_factor, 0.0, 1.0)
        cmd.linear.x = self.normal_speed * heading_factor * speed_factor
        cmd.angular.z = self.clamp(self.yaw_gain * yaw_error, -self.max_angular_speed, self.max_angular_speed)
        if abs(yaw_error) > 0.8:
            cmd.linear.x = 0.0
        return cmd

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))
