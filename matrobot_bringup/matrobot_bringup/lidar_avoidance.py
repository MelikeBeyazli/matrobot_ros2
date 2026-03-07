#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math


class LidarAvoidance(Node):
    def __init__(self):
        super().__init__('lidar_avoidance')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.forward_speed = 0.20
        self.turn_speed = -0.80
        self.stop_distance = 0.60

        self.get_logger().info("LiDAR engel kaçınma düğümü başlatıldı.")

    def scan_callback(self, msg: LaserScan):
        twist = Twist()
        front_distances = []

        for i, distance in enumerate(msg.ranges):
            # Her ölçümün açısını radyan cinsinden hesapla
            angle_rad = msg.angle_min + i * msg.angle_increment

            # Dereceye çevir
            angle_deg = math.degrees(angle_rad)

            # Açıyı 0-360 aralığına taşı
            if angle_deg < 0:
                angle_deg += 360.0

            # Ön bölge: 0-60 ve 300-360 derece
            if 0 <= angle_deg <= 15 or 345 <= angle_deg <= 360:
                if math.isinf(distance) or math.isnan(distance) or distance <= 0.0:
                    continue

                front_distances.append(distance)

        if len(front_distances) == 0:
            self.get_logger().warn("Ön bölgeden geçerli LiDAR verisi alınamadı.")
            twist.linear.x = 0.0
            twist.angular.z = self.turn_speed
            self.cmd_vel_pub.publish(twist)
            return

        front_average = sum(front_distances) / len(front_distances)

        if front_average < self.stop_distance:
            twist.linear.x = 0.0
            twist.angular.z = self.turn_speed
            self.get_logger().info(
                f"Engel algılandı | Ön ortalama: {front_average:.2f} m -> Sağa dönüyor"
            )
        else:
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0
            self.get_logger().info(
                f"Yol açık | Ön ortalama: {front_average:.2f} m -> İleri gidiyor"
            )

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoidance()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node kapatılıyor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
