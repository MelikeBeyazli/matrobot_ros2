#!/usr/bin/env python3
"""
Frontier-based explorer for SLAM mapping mode.

Use this only for mapping experiments, not for the main navigation demo.

Features:
- Subscribes to /map from slam_toolbox
- Finds frontier cells: known free cells next to unknown cells
- Chooses a nearby reachable-looking frontier
- Drives toward the selected frontier
- If obstacle blocks the target, blacklists that frontier and picks another
- Clears blacklist if no valid frontier remains
"""

import math
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


WorldPoint = Tuple[float, float]


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")

        self.declare_parameter("mode", "mapping")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("odom_topic", "/odometry/local")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.declare_parameter("frontier_search_stride", 3)
        self.declare_parameter("frontier_min_distance", 0.80)
        self.declare_parameter("frontier_max_distance", 6.00)
        self.declare_parameter("target_reached_distance", 0.35)

        self.declare_parameter("normal_speed", 0.18)
        self.declare_parameter("turn_speed_limit", 0.60)
        self.declare_parameter("yaw_gain", 1.30)

        self.declare_parameter("front_stop_distance", 0.60)
        self.declare_parameter("backup_speed", -0.10)
        self.declare_parameter("backup_duration", 0.9)
        self.declare_parameter("reselect_period", 4.0)

        self.declare_parameter("blacklist_radius", 0.80)
        self.declare_parameter("max_blacklist_size", 40)

        if self.get_parameter("mode").value != "mapping":
            raise RuntimeError("frontier_explorer.py only runs with mode:=mapping")

        self.frontier_search_stride = int(self.get_parameter("frontier_search_stride").value)
        self.frontier_min_distance = float(self.get_parameter("frontier_min_distance").value)
        self.frontier_max_distance = float(self.get_parameter("frontier_max_distance").value)
        self.target_reached_distance = float(self.get_parameter("target_reached_distance").value)

        self.normal_speed = float(self.get_parameter("normal_speed").value)
        self.turn_speed_limit = float(self.get_parameter("turn_speed_limit").value)
        self.yaw_gain = float(self.get_parameter("yaw_gain").value)

        self.front_stop_distance = float(self.get_parameter("front_stop_distance").value)
        self.backup_speed = float(self.get_parameter("backup_speed").value)
        self.backup_duration = float(self.get_parameter("backup_duration").value)
        self.reselect_period = float(self.get_parameter("reselect_period").value)

        self.blacklist_radius = float(self.get_parameter("blacklist_radius").value)
        self.max_blacklist_size = int(self.get_parameter("max_blacklist_size").value)

        self.map_msg: Optional[OccupancyGrid] = None
        self.scan: Optional[LaserScan] = None

        self.x: Optional[float] = None
        self.y: Optional[float] = None
        self.yaw: Optional[float] = None

        self.current_target: Optional[WorldPoint] = None
        self.last_target_select_time = 0.0

        self.blacklisted_targets: List[WorldPoint] = []

        self.backing_up = False
        self.backup_start_time = 0.0

        self.cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            10,
        )

        self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self.map_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self.odom_callback,
            10,
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self.scan_callback,
            10,
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Frontier explorer started with blacklist support.")

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg

    def scan_callback(self, msg: LaserScan):
        self.scan = msg

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def control_loop(self):
        if self.map_msg is None or self.x is None or self.y is None or self.yaw is None:
            return

        if self.backing_up:
            self.run_backup()
            return

        front = self.get_front_min_distance()

        if front < self.front_stop_distance:
            self.blacklist_current_target()
            self.start_backup()
            return

        if self.current_target is None:
            self.select_new_target()
            return

        distance_to_target = math.hypot(
            self.current_target[0] - self.x,
            self.current_target[1] - self.y,
        )

        now = self.now_seconds()

        if distance_to_target < self.target_reached_distance:
            self.get_logger().info("Frontier target reached. Selecting new frontier.")
            self.current_target = None
            self.select_new_target()
            return

        if now - self.last_target_select_time > self.reselect_period:
            self.select_new_target()
            return

        self.drive_to_target()

    def select_new_target(self):
        frontiers = self.find_frontiers()

        if not frontiers:
            self.stop_robot()
            self.get_logger().warn("No frontier found. Exploration may be complete.")
            return

        best_target = self.choose_best_frontier(frontiers)

        if best_target is None:
            self.get_logger().warn("All nearby frontiers are blacklisted. Clearing blacklist once.")
            self.blacklisted_targets.clear()
            best_target = self.choose_best_frontier(frontiers)

        if best_target is None:
            self.stop_robot()
            self.get_logger().warn("No valid frontier target selected after blacklist reset.")
            return

        self.current_target = best_target
        self.last_target_select_time = self.now_seconds()

        self.get_logger().info(
            f"New frontier target: x={best_target[0]:.2f}, y={best_target[1]:.2f}"
        )

    def find_frontiers(self) -> List[WorldPoint]:
        msg = self.map_msg
        width = msg.info.width
        height = msg.info.height
        data = msg.data

        frontiers: List[WorldPoint] = []
        stride = max(1, self.frontier_search_stride)

        for gy in range(1, height - 1, stride):
            for gx in range(1, width - 1, stride):
                idx = gy * width + gx

                if data[idx] != 0:
                    continue

                if self.has_unknown_neighbor(gx, gy, width, height, data):
                    wx, wy = self.grid_to_world(gx, gy)
                    dist = math.hypot(wx - self.x, wy - self.y)

                    if self.frontier_min_distance <= dist <= self.frontier_max_distance:
                        frontiers.append((wx, wy))

        return frontiers

    def has_unknown_neighbor(self, gx: int, gy: int, width: int, height: int, data) -> bool:
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue

                nx = gx + dx
                ny = gy + dy

                if nx < 0 or ny < 0 or nx >= width or ny >= height:
                    continue

                if data[ny * width + nx] == -1:
                    return True

        return False

    def choose_best_frontier(self, frontiers: List[WorldPoint]) -> Optional[WorldPoint]:
        if self.x is None or self.y is None or self.yaw is None:
            return None

        best_score = float("inf")
        best_target = None

        for wx, wy in frontiers:
            candidate = (wx, wy)

            if self.is_blacklisted(candidate):
                continue

            distance = math.hypot(wx - self.x, wy - self.y)
            heading = math.atan2(wy - self.y, wx - self.x)
            yaw_error = abs(self.normalize_angle(heading - self.yaw))

            # Prefer close frontiers, but avoid requiring sharp turns.
            score = distance + 0.7 * yaw_error

            if score < best_score:
                best_score = score
                best_target = candidate

        return best_target

    def blacklist_current_target(self):
        if self.current_target is None:
            return

        if not self.is_blacklisted(self.current_target):
            self.blacklisted_targets.append(self.current_target)

            if len(self.blacklisted_targets) > self.max_blacklist_size:
                self.blacklisted_targets.pop(0)

            self.get_logger().warn(
                f"Blacklisted unreachable frontier: "
                f"x={self.current_target[0]:.2f}, y={self.current_target[1]:.2f}"
            )

        self.current_target = None

    def is_blacklisted(self, target: WorldPoint) -> bool:
        tx, ty = target

        for bx, by in self.blacklisted_targets:
            if math.hypot(tx - bx, ty - by) < self.blacklist_radius:
                return True

        return False

    def drive_to_target(self):
        if self.current_target is None or self.x is None or self.y is None or self.yaw is None:
            return

        tx, ty = self.current_target
        target_yaw = math.atan2(ty - self.y, tx - self.x)
        yaw_error = self.normalize_angle(target_yaw - self.yaw)

        cmd = Twist()

        cmd.angular.z = self.clamp(
            self.yaw_gain * yaw_error,
            -self.turn_speed_limit,
            self.turn_speed_limit,
        )

        if abs(yaw_error) < 0.7:
            heading_factor = max(0.0, 1.0 - abs(yaw_error))
            cmd.linear.x = self.normal_speed * heading_factor
        else:
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)

    def start_backup(self):
        self.backing_up = True
        self.backup_start_time = self.now_seconds()
        self.stop_robot()
        self.get_logger().warn("Obstacle close. Backing up and selecting another frontier.")

    def run_backup(self):
        elapsed = self.now_seconds() - self.backup_start_time

        cmd = Twist()

        if elapsed < self.backup_duration:
            cmd.linear.x = self.backup_speed
            cmd.angular.z = 0.25
            self.cmd_pub.publish(cmd)
            return

        self.backing_up = False
        self.stop_robot()
        self.select_new_target()

    def get_front_min_distance(self) -> float:
        if self.scan is None:
            return 10.0

        values = []

        for i, r in enumerate(self.scan.ranges):
            if math.isnan(r) or math.isinf(r):
                continue

            if r < self.scan.range_min or r > self.scan.range_max:
                continue

            angle = self.scan.angle_min + i * self.scan.angle_increment
            angle_deg = math.degrees(angle)

            if -30.0 <= angle_deg <= 30.0:
                values.append(r)

        return min(values) if values else 10.0

    def grid_to_world(self, gx: int, gy: int) -> WorldPoint:
        msg = self.map_msg

        wx = msg.info.origin.position.x + (gx + 0.5) * msg.info.resolution
        wy = msg.info.origin.position.y + (gy + 0.5) * msg.info.resolution

        return wx, wy

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def clamp(value: float, lower: float, upper: float):
        return max(lower, min(upper, value))

    def destroy_node(self):
        self.stop_robot()
        return super().destroy_node()


def main(args: Optional[Sequence[str]] = None):
    rclpy.init(args=args)
    node = FrontierExplorer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
