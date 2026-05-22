#!/usr/bin/env python3
import csv
import math
from enum import Enum
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster

from navigation.astar_planner import AStarPlanner
from navigation.bt_core import ActionNode, BTStatus, ConditionNode, SelectorNode, SequenceNode
from navigation.los_controller import LOSController
from navigation.map_manager import MapManager, WorldPoint
from utils.project_paths import ProjectPaths


class RecoveryPhase(Enum):
    IDLE = 0
    MARK_BLOCKED = 1
    BACKUP = 2
    TURN_TO_CLEAR = 3
    REPLAN = 4


class AStarBTNavigator(Node):
    def __init__(self) -> None:
        super().__init__("astar_bt_navigator")
        self.paths = ProjectPaths()

        params = {
            "mode": "navigation",
            "map_yaml_path": str(self.paths.default_map_yaml),

            "goal_x": 10.0,
            "goal_y": -2.0,
            "goal_yaw": 0.0,
            "goal_yaw_tolerance": 0.40,
            "final_yaw_gain": 1.20,

            "odom_topic": "/odometry/local",
            "scan_topic": "/scan",
            "cmd_vel_topic": "/cmd_vel",
            "map_topic": "/map",
            "path_topic": "/planned_path",

            "map_frame": "map",
            "odom_frame": "odom",
            "publish_static_map_to_odom": True,
            "publish_inflated_map": False,

            "robot_radius_m": 0.24,
            "safety_margin_m": 0.18,
            "lookahead_distance": 0.80,
            "goal_tolerance": 0.55,
            "near_goal_replan_distance": 1.20,
            "final_approach_distance": 0.85,
            "final_approach_speed": 0.08,
            "final_approach_yaw_tolerance": 0.35,

            "lidar_obstacle_range": 1.20,
            "slow_distance": 1.10,
            "stop_distance": 0.60,
            "critical_distance": 0.42,

            "normal_speed": 0.24,
            "min_navigation_speed": 0.06,
            "max_angular_speed": 0.80,
            "yaw_gain": 1.60,
            "replan_period": 2.50,

            "backup_duration": 1.60,
            "backup_speed": -0.14,
            "turn_duration": 2.20,
            "turn_speed": 0.58,
            "recovery_block_radius_cells": 10,
            "recovery_block_forward_distance": 1.50,

            "max_linear_accel": 0.35,
            "max_angular_accel": 1.20,
            "control_period": 0.10,
        }

        for key, value in params.items():
            self.declare_parameter(key, value)

        self.mode = self.get_parameter("mode").value
        if self.mode != "navigation":
            raise RuntimeError(
                "bt_navigator_node.py only runs in mode:=navigation. "
                "Use mapping/frontier_explorer.py or mapping/coverage_explorer.py for mapping."
            )

        self.map_yaml_path = self.get_parameter("map_yaml_path").value

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.goal_yaw = float(self.get_parameter("goal_yaw").value)
        self.goal_yaw_tolerance = float(self.get_parameter("goal_yaw_tolerance").value)
        self.final_yaw_gain = float(self.get_parameter("final_yaw_gain").value)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.map_topic = self.get_parameter("map_topic").value
        self.path_topic = self.get_parameter("path_topic").value

        self.map_frame = self.get_parameter("map_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.publish_static_tf = bool(self.get_parameter("publish_static_map_to_odom").value)

        self.robot_radius_m = float(self.get_parameter("robot_radius_m").value)
        self.safety_margin_m = float(self.get_parameter("safety_margin_m").value)
        self.effective_robot_radius_m = self.robot_radius_m + self.safety_margin_m

        self.lookahead_distance = float(self.get_parameter("lookahead_distance").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.near_goal_replan_distance = float(self.get_parameter("near_goal_replan_distance").value)
        self.final_approach_distance = float(self.get_parameter("final_approach_distance").value)
        self.final_approach_speed = float(self.get_parameter("final_approach_speed").value)
        self.final_approach_yaw_tolerance = float(self.get_parameter("final_approach_yaw_tolerance").value)

        self.lidar_obstacle_range = float(self.get_parameter("lidar_obstacle_range").value)
        self.slow_distance = float(self.get_parameter("slow_distance").value)
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.critical_distance = float(self.get_parameter("critical_distance").value)

        self.normal_speed = float(self.get_parameter("normal_speed").value)
        self.min_navigation_speed = float(self.get_parameter("min_navigation_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.yaw_gain = float(self.get_parameter("yaw_gain").value)
        self.replan_period = float(self.get_parameter("replan_period").value)

        self.backup_duration = float(self.get_parameter("backup_duration").value)
        self.backup_speed = float(self.get_parameter("backup_speed").value)
        self.turn_duration = float(self.get_parameter("turn_duration").value)
        self.turn_speed = float(self.get_parameter("turn_speed").value)
        self.recovery_block_radius_cells = int(self.get_parameter("recovery_block_radius_cells").value)
        self.recovery_block_forward_distance = float(self.get_parameter("recovery_block_forward_distance").value)

        self.max_linear_accel = float(self.get_parameter("max_linear_accel").value)
        self.max_angular_accel = float(self.get_parameter("max_angular_accel").value)
        self.control_period = float(self.get_parameter("control_period").value)

        self.x: Optional[float] = None
        self.y: Optional[float] = None
        self.yaw: Optional[float] = None
        self.scan: Optional[LaserScan] = None

        self.path: List[WorldPoint] = []
        self.last_replan_time = 0.0
        self.last_obstacle_added = False
        self.last_cmd = Twist()

        self.recovery_phase = RecoveryPhase.IDLE
        self.recovery_start_time: Optional[float] = None
        self.recovery_turn_direction = 1.0
        self.bt_state = "WAITING_FOR_DATA"

        self.map_manager = MapManager(
            self.map_yaml_path,
            self.effective_robot_radius_m,
            bool(self.get_parameter("publish_inflated_map").value),
        )
        self.planner = AStarPlanner(self.map_manager)
        self.controller = LOSController(
            self.lookahead_distance,
            self.normal_speed,
            self.slow_distance,
            self.stop_distance,
            self.yaw_gain,
            self.max_angular_speed,
        )

        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched_qos.reliability = ReliabilityPolicy.RELIABLE

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, latched_qos)

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        if self.publish_static_tf:
            self.tf_broadcaster = StaticTransformBroadcaster(self)
            self.publish_static_map_to_odom()

        self.log_file = open(self.paths.navigation_log, "w", newline="")
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow([
            "time", "x", "y", "yaw",
            "goal_x", "goal_y", "goal_yaw",
            "distance_to_goal", "yaw_error_to_goal",
            "path_length", "front_distance",
            "bt_state", "recovery_phase",
        ])

        self.behavior_tree = self.create_behavior_tree()
        self.control_timer = self.create_timer(self.control_period, self.control_loop)
        self.map_timer = self.create_timer(1.0, self.publish_map)

        self.publish_map()
        self.get_logger().info("Navigation mode: corrected BT A* navigator started.")
        self.get_logger().info(
            f"effective_robot_radius={self.effective_robot_radius_m:.2f}, "
            f"stop={self.stop_distance:.2f}, slow={self.slow_distance:.2f}, final_approach={self.final_approach_distance:.2f}"
        )

    def create_behavior_tree(self):
        return SelectorNode([
            SequenceNode([
                ConditionNode(self.is_goal_reached),
                ActionNode(self.action_stop_at_goal),
            ]),
            SequenceNode([
                ConditionNode(self.is_goal_position_reached),
                ActionNode(self.action_align_goal_yaw),
            ]),
            SequenceNode([
                ConditionNode(self.is_final_approach_area),
                ActionNode(self.action_final_approach),
            ]),
            SequenceNode([
                ConditionNode(self.needs_recovery),
                ActionNode(self.action_recovery),
            ]),
            SequenceNode([
                ActionNode(self.action_ensure_path),
                ActionNode(self.action_update_dynamic_obstacles),
                ActionNode(self.action_replan_if_needed),
                ActionNode(self.action_follow_path_los),
            ]),
        ])

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def scan_callback(self, msg):
        self.scan = msg

    def publish_map(self):
        self.map_pub.publish(
            self.map_manager.build_occupancy_grid_msg(
                self.map_frame,
                self.get_clock().now().to_msg(),
            )
        )

    def publish_static_map_to_odom(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

    def publish_path(self):
        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()

        for wx, wy in self.path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.path_pub.publish(msg)

    def clear_old_path(self):
        self.path = []
        self.controller.reset()
        self.publish_path()

    def replan(self):
        if self.x is None or self.y is None:
            return False

        new_path = self.planner.plan((self.x, self.y), (self.goal_x, self.goal_y))

        if not new_path:
            self.get_logger().warn("No path found.")
            return False

        self.path = new_path
        self.controller.reset()
        self.publish_path()
        self.get_logger().info(f"New path planned. Waypoints: {len(self.path)}")
        return True

    def update_obstacles_from_lidar(self):
        if self.scan is None or self.x is None or self.y is None or self.yaw is None:
            return False

        if self.is_near_goal_area():
            self.last_obstacle_added = False
            return False

        if self.recovery_phase == RecoveryPhase.IDLE:
            self.map_manager.clear_dynamic_layer()

        obstacle_added = False

        for i, distance in enumerate(self.scan.ranges):
            if not self.valid_range(distance) or distance > self.lidar_obstacle_range:
                continue

            angle = self.scan.angle_min + i * self.scan.angle_increment
            angle_deg = math.degrees(angle)

            if not (-95.0 <= angle_deg <= 95.0):
                continue

            world_angle = self.yaw + angle
            self.map_manager.add_dynamic_obstacle_world(
                self.x + distance * math.cos(world_angle),
                self.y + distance * math.sin(world_angle),
            )
            obstacle_added = True

        return obstacle_added

    def mark_front_blocked_zone(self):
        if self.x is None or self.y is None or self.yaw is None:
            return

        d = self.stop_distance
        while d <= self.recovery_block_forward_distance:
            self.map_manager.add_dynamic_block_world(
                self.x + d * math.cos(self.yaw),
                self.y + d * math.sin(self.yaw),
                self.recovery_block_radius_cells,
            )
            d += 0.15

    def valid_range(self, value):
        return (
            self.scan is not None
            and not math.isnan(value)
            and not math.isinf(value)
            and self.scan.range_min <= value <= self.scan.range_max
        )

    def get_front_min_distance(self):
        return self.get_sector_min_distance(-40.0, 40.0, 10.0)

    def get_side_clearance(self):
        return (
            self.get_sector_min_distance(35.0, 130.0, 10.0),
            self.get_sector_min_distance(-130.0, -35.0, 10.0),
        )

    def get_sector_min_distance(self, min_angle_deg, max_angle_deg, default):
        if self.scan is None:
            return default

        values = []
        for i, distance in enumerate(self.scan.ranges):
            if not self.valid_range(distance):
                continue

            angle_deg = math.degrees(self.scan.angle_min + i * self.scan.angle_increment)
            if min_angle_deg <= angle_deg <= max_angle_deg:
                values.append(distance)

        return min(values) if values else default

    def needs_recovery(self):
        if self.is_near_goal_area():
            return False

        if self.recovery_phase != RecoveryPhase.IDLE:
            return True

        return self.scan is not None and self.get_front_min_distance() <= self.stop_distance

    def is_near_goal_area(self):
        return (
            self.x is not None
            and self.y is not None
            and math.hypot(self.goal_x - self.x, self.goal_y - self.y) < self.near_goal_replan_distance
        )

    def is_final_approach_area(self):
        return (
            self.x is not None
            and self.y is not None
            and math.hypot(self.goal_x - self.x, self.goal_y - self.y) < self.final_approach_distance
        )

    def is_goal_position_reached(self):
        return (
            self.x is not None
            and self.y is not None
            and math.hypot(self.goal_x - self.x, self.goal_y - self.y) < self.goal_tolerance
        )

    def is_goal_reached(self):
        if self.x is None or self.y is None or self.yaw is None:
            return False

        yaw_error = self.normalize_angle(self.goal_yaw - self.yaw)
        return self.is_goal_position_reached() and abs(yaw_error) < self.goal_yaw_tolerance

    def action_stop_at_goal(self):
        self.bt_state = "GOAL_REACHED"
        self.stop_robot()
        return BTStatus.SUCCESS

    def action_align_goal_yaw(self):
        self.bt_state = "ALIGN_GOAL_YAW"

        if self.yaw is None:
            return BTStatus.FAILURE

        yaw_error = self.normalize_angle(self.goal_yaw - self.yaw)

        if abs(yaw_error) < self.goal_yaw_tolerance:
            self.stop_robot()
            return BTStatus.SUCCESS

        cmd = Twist()
        cmd.angular.z = self.clamp(self.final_yaw_gain * yaw_error, -0.45, 0.45)
        self.publish_smoothed_cmd(cmd)
        return BTStatus.RUNNING

    def action_ensure_path(self):
        self.bt_state = "ENSURE_PATH"
        return BTStatus.SUCCESS if self.path or self.replan() else BTStatus.FAILURE

    def action_update_dynamic_obstacles(self):
        self.bt_state = "UPDATE_DYNAMIC_OBSTACLES"
        self.last_obstacle_added = self.update_obstacles_from_lidar()
        return BTStatus.SUCCESS

    def action_replan_if_needed(self):
        self.bt_state = "REPLAN_IF_NEEDED"

        if self.is_near_goal_area():
            return BTStatus.SUCCESS

        now = self.now_seconds()
        if self.last_obstacle_added and now - self.last_replan_time > self.replan_period:
            self.last_replan_time = now
            return BTStatus.SUCCESS if self.replan() else BTStatus.FAILURE

        return BTStatus.SUCCESS

    def action_final_approach(self):
        self.bt_state = "FINAL_APPROACH"

        if self.x is None or self.y is None or self.yaw is None:
            self.stop_robot()
            return BTStatus.FAILURE

        distance_to_goal = math.hypot(self.goal_x - self.x, self.goal_y - self.y)

        if distance_to_goal < self.goal_tolerance:
            self.stop_robot()
            return BTStatus.SUCCESS

        target_yaw = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
        yaw_error = self.normalize_angle(target_yaw - self.yaw)

        cmd = Twist()
        cmd.angular.z = self.clamp(1.0 * yaw_error, -0.35, 0.35)

        if abs(yaw_error) < self.final_approach_yaw_tolerance:
            cmd.linear.x = min(
                self.final_approach_speed,
                max(0.03, 0.25 * distance_to_goal),
            )
        else:
            cmd.linear.x = 0.0

        self.publish_smoothed_cmd(cmd)
        return BTStatus.RUNNING

    def action_follow_path_los(self):
        self.bt_state = "FOLLOW_PATH_LOS"

        if not self.path or self.x is None or self.y is None or self.yaw is None:
            self.stop_robot()
            return BTStatus.FAILURE

        distance_to_goal = math.hypot(self.goal_x - self.x, self.goal_y - self.y)

        if distance_to_goal < self.goal_tolerance:
            self.stop_robot()
            return BTStatus.SUCCESS

        if distance_to_goal < self.final_approach_distance:
            return self.action_final_approach()

        front = self.get_front_min_distance()

        if front <= self.critical_distance:
            self.stop_robot()
            self.start_recovery()
            return BTStatus.RUNNING

        cmd = self.controller.compute_cmd(self.path, self.x, self.y, self.yaw, front)
        cmd = self.apply_distance_based_speed_limit(cmd, front)
        self.publish_smoothed_cmd(cmd)
        return BTStatus.RUNNING

    def apply_distance_based_speed_limit(self, cmd, front):
        if cmd.linear.x <= 0.0:
            return cmd

        if front >= self.slow_distance:
            max_speed = self.normal_speed
        elif front <= self.stop_distance:
            max_speed = 0.0
        else:
            ratio = (front - self.stop_distance) / (self.slow_distance - self.stop_distance)
            ratio = self.clamp(ratio, 0.0, 1.0)
            max_speed = self.min_navigation_speed + ratio * (self.normal_speed - self.min_navigation_speed)

        cmd.linear.x = min(cmd.linear.x, max_speed)
        return cmd

    def action_recovery(self):
        self.bt_state = "RECOVERY"

        if self.recovery_phase == RecoveryPhase.IDLE:
            self.start_recovery()

        return self.tick_recovery()

    def start_recovery(self):
        left, right = self.get_side_clearance()
        self.recovery_turn_direction = 1.0 if left > right else -1.0
        self.recovery_phase = RecoveryPhase.MARK_BLOCKED
        self.recovery_start_time = self.now_seconds()

        self.clear_old_path()
        self.stop_robot()

        self.get_logger().warn(
            f"BT recovery started. old path cleared. left={left:.2f}, right={right:.2f}"
        )

    def tick_recovery(self):
        if self.recovery_start_time is None:
            self.recovery_start_time = self.now_seconds()

        elapsed = self.now_seconds() - self.recovery_start_time
        cmd = Twist()

        if self.recovery_phase == RecoveryPhase.MARK_BLOCKED:
            self.mark_front_blocked_zone()
            self.recovery_phase = RecoveryPhase.BACKUP
            self.recovery_start_time = self.now_seconds()
            self.get_logger().warn("Recovery: front blocked zone marked, backing up.")
            return BTStatus.RUNNING

        if self.recovery_phase == RecoveryPhase.BACKUP:
            if elapsed < self.backup_duration:
                cmd.linear.x = self.backup_speed
                cmd.angular.z = -0.15 * self.recovery_turn_direction
                self.publish_smoothed_cmd(cmd)
                return BTStatus.RUNNING

            self.recovery_phase = RecoveryPhase.TURN_TO_CLEAR
            self.recovery_start_time = self.now_seconds()
            return BTStatus.RUNNING

        if self.recovery_phase == RecoveryPhase.TURN_TO_CLEAR:
            if elapsed < self.turn_duration:
                cmd.angular.z = self.turn_speed * self.recovery_turn_direction
                self.publish_smoothed_cmd(cmd)
                return BTStatus.RUNNING

            self.recovery_phase = RecoveryPhase.REPLAN
            self.recovery_start_time = self.now_seconds()
            return BTStatus.RUNNING

        if self.recovery_phase == RecoveryPhase.REPLAN:
            if self.replan():
                self.recovery_phase = RecoveryPhase.IDLE
                self.get_logger().info("BT recovery completed.")
                return BTStatus.SUCCESS

            self.recovery_phase = RecoveryPhase.TURN_TO_CLEAR
            self.recovery_start_time = self.now_seconds()
            self.get_logger().warn("Recovery replan failed. Turning again.")
            return BTStatus.RUNNING

        return BTStatus.FAILURE

    def control_loop(self):
        if self.x is None or self.y is None or self.yaw is None or self.scan is None:
            return

        self.log_data()
        status = self.behavior_tree.tick()

        if status == BTStatus.FAILURE:
            self.bt_state = "BT_FAILURE"
            self.stop_robot()

    def log_data(self):
        if self.x is None or self.y is None or self.yaw is None:
            return

        self.csv_writer.writerow([
            self.now_seconds(),
            self.x,
            self.y,
            self.yaw,
            self.goal_x,
            self.goal_y,
            self.goal_yaw,
            math.hypot(self.goal_x - self.x, self.goal_y - self.y),
            self.normalize_angle(self.goal_yaw - self.yaw),
            len(self.path),
            self.get_front_min_distance(),
            self.bt_state,
            self.recovery_phase.name,
        ])
        self.log_file.flush()

    def stop_robot(self):
        self.publish_smoothed_cmd(Twist(), force=True)

    def publish_smoothed_cmd(self, target_cmd: Twist, force: bool = False):
        if force:
            self.last_cmd = target_cmd
            self.cmd_pub.publish(target_cmd)
            return

        max_dv = self.max_linear_accel * self.control_period
        max_dw = self.max_angular_accel * self.control_period

        cmd = Twist()
        cmd.linear.x = self.rate_limit(self.last_cmd.linear.x, target_cmd.linear.x, max_dv)
        cmd.angular.z = self.rate_limit(self.last_cmd.angular.z, target_cmd.angular.z, max_dw)

        self.last_cmd = cmd
        self.cmd_pub.publish(cmd)

    @staticmethod
    def rate_limit(current, target, max_delta):
        delta = target - current
        if delta > max_delta:
            return current + max_delta
        if delta < -max_delta:
            return current - max_delta
        return target

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))

    def destroy_node(self):
        try:
            self.stop_robot()
            if hasattr(self, "log_file") and not self.log_file.closed:
                self.log_file.close()
        finally:
            return super().destroy_node()


def main(args: Optional[Sequence[str]] = None):
    rclpy.init(args=args)
    node = AStarBTNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()