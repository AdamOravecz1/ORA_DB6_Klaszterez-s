#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        # Feliratkozás a /scan témára
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10  # QoS mélység
        )
        self.subscription  # Megelőzi a figyelmeztetést a nem használt változóról
        self.get_logger().info("Lidar Subscriber Node started. Listening to /scan...")

    def lidar_callback(self, msg):
        # Kiírjuk a kapott adatokat
        self.get_logger().info(f"Received Lidar Data:\nRanges: {msg.ranges}\n")


def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped cleanly")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
