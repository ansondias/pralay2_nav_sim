#!/usr/bin/env python3
"""Keep the hull level by driving the fin pair differentially."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def quaternion_to_roll(w, x, y, z):
    return math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))


class BasicRollHold(Node):
    def __init__(self):
        super().__init__("basic_roll_hold")

        self.declare_parameter("target_angle", 0.0)      # deg, 0 = level
        self.declare_parameter("angle_threshold", 15.0)  # deg
        self.declare_parameter("max_fin_angle", 10.0)    # deg
        self.declare_parameter("control_rate", 20.0)     # Hz
        self.declare_parameter("fin_sign", 1.0)          # flip if it rolls the wrong way

        p = self.get_parameter
        self.target = math.radians(float(p("target_angle").value))
        self.threshold = math.radians(float(p("angle_threshold").value))
        self.max_fin = math.radians(float(p("max_fin_angle").value))
        self.fin_sign = float(p("fin_sign").value)

        self.roll = None

        self.imu_subscription = self.create_subscription(Imu, "/imu", self.imu_callback, 10)
        self.fin_pub = self.create_publisher(Float64, "/roll_hold/aileron", 10)

        self.create_timer(1.0 / float(p("control_rate").value), self.control_step)
        self.get_logger().info(
            f"basic roll hold: target {math.degrees(self.target):.1f} deg, "
            f"band +-{math.degrees(self.threshold):.1f} deg, "
            f"max fin {math.degrees(self.max_fin):.1f} deg")

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        self.roll = quaternion_to_roll(q.w, q.x, q.y, q.z)

    def control_step(self):
        if self.roll is None:
            return

        error = self.target - self.roll

        if error > self.threshold:
            aileron = self.max_fin
        elif error < -self.threshold:
            aileron = -self.max_fin
        else:
            aileron = map_range(error, -self.threshold, self.threshold,
                                -self.max_fin, self.max_fin)

        self.fin_pub.publish(Float64(data=float(self.fin_sign * aileron)))


def main(args=None):
    rclpy.init(args=args)
    node = BasicRollHold()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
