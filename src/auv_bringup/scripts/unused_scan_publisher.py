#!/usr/bin/env python3
"""Publishes an always-empty LaserScan on /unused_scan.

collision_monitor requires at least one observation_sources entry (see
auv_navigation/config/nav2_params.yaml's collision_monitor block), and
treats a source that never publishes anything as invalid/stale -- its
fallback for that is to force the robot's outgoing cmd_vel to zero,
which silently blocked all Nav2-driven motion even though the planner
and controller were both working correctly. Publishing a real (if
empty -- no ranges, meaning "no detections") scan keeps it satisfied
without pretending to have a working obstacle sensor.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class UnusedScanPublisher(Node):
    def __init__(self):
        super().__init__('unused_scan_publisher')
        self.publisher_ = self.create_publisher(LaserScan, '/unused_scan', 10)
        self.timer_ = self.create_timer(0.1, self.publish_scan)

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.angle_min = 0.0
        msg.angle_max = 0.0
        msg.angle_increment = 1.0
        msg.range_min = 0.1
        msg.range_max = 10.0
        msg.ranges = []
        msg.intensities = []
        self.publisher_.publish(msg)


def main():
    rclpy.init()
    node = UnusedScanPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
