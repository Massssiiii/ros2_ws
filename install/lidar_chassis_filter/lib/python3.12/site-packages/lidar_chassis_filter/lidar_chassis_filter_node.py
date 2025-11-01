#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import ast  # pour convertir la chaîne en liste Python


class LidarChassisFilter(Node):
    """
    Node ROS2 pour filtrer le châssis du robot dans un scan LIDAR 360°.
    Compatible ROS2 Jazzy.
    """

    def __init__(self):
        super().__init__('lidar_chassis_filter')

        # --- Paramètres configurables ---
        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan_filtered')
        self.declare_parameter('min_range', 0.25)
        self.declare_parameter(
            'forbidden_zones',
            "[[-0.8, -0.6], [0.6, 0.8], [2.3, 2.5], [-2.5, -2.3]]"
        )
        # -------------------------------

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.min_range = self.get_parameter('min_range').value

        # Conversion string → liste Python
        forbidden_str = self.get_parameter('forbidden_zones').value
        try:
            self.forbidden_zones = [tuple(zone) for zone in ast.literal_eval(forbidden_str)]
        except Exception as e:
            self.get_logger().warn(f"Erreur parsing forbidden_zones : {e}")
            self.forbidden_zones = []

        self.get_logger().info(
            f"✅ LidarChassisFilter actif — input: {self.input_topic}, "
            f"output: {self.output_topic}, min_range: {self.min_range} m"
        )

        self.sub = self.create_subscription(LaserScan, self.input_topic, self.callback, 10)
        self.pub = self.create_publisher(LaserScan, self.output_topic, 10)

    def callback(self, msg: LaserScan):
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        new_ranges = []

        for r, θ in zip(msg.ranges, angles):
            # 1️⃣ Filtrage par distance (châssis proche)
            if r < self.min_range:
                new_ranges.append(float('inf'))
                continue

            # 2️⃣ Filtrage par zones angulaires
            if any(start <= θ <= end for (start, end) in self.forbidden_zones):
                new_ranges.append(float('inf'))
                continue

            new_ranges.append(r)

        msg.ranges = new_ranges
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarChassisFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

