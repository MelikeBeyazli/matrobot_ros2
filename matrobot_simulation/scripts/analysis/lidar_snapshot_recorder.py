#!/usr/bin/env python3
"""
Records one LiDAR scan snapshot for report visualization.

Output:
  ~/matrobot_ws/src/matrobot_ros2/matrobot_simulation/logs/lidar_snapshot.csv
"""

from pathlib import Path
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarSnapshotRecorder(Node):
    def __init__(self):
        super().__init__("lidar_snapshot_recorder")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("output_file", str(
            Path.home()
            / "matrobot_ws"
            / "src"
            / "matrobot_ros2"
            / "matrobot_simulation"
            / "logs"
            / "lidar_snapshot.csv"
        ))

        self.output_file = Path(self.get_parameter("output_file").value)
        self.output_file.parent.mkdir(parents=True, exist_ok=True)

        self.subscription = self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self.scan_callback,
            10,
        )

        self.get_logger().info("Waiting for one LiDAR scan...")

    def scan_callback(self, msg: LaserScan):
        with open(self.output_file, "w") as f:
            f.write("index,angle_rad,angle_deg,raw_range_m,filtered_range_m,is_valid\n")

            for i, r in enumerate(msg.ranges):
                angle = msg.angle_min + i * msg.angle_increment
                valid = (
                    not math.isnan(r)
                    and not math.isinf(r)
                    and msg.range_min <= r <= msg.range_max
                )

                filtered = r if valid else ""

                f.write(
                    f"{i},{angle},{math.degrees(angle)},{r},{filtered},{int(valid)}\n"
                )

        self.get_logger().info(f"LiDAR snapshot saved to: {self.output_file}")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = LidarSnapshotRecorder()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
