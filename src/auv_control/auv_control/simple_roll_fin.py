#!/usr/bin/env python3
"""Keep the hull level by driving the fin pair differentially."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def fin_angle_for(error: float, kp: float, deadband: float, max_fin: float) -> float:
    """Proportional fin angle for an attitude error (error = target - angle), in radians."""
    if abs(error) <= deadband:
        # Level enough: don't chase sensor noise.
        return 0.0
    # More roll -> proportionally more fin, up to the roll budget. No step:
    # the slope just flattens at the clamp.
    return clamp(kp * error, max_fin)


def slew_limit(target: float, previous: float, max_step: float) -> float:
    """Move from previous toward target by at most max_step."""
    return previous + clamp(target - previous, max_step)


def quaternion_to_roll(w: float, x: float, y: float, z: float) -> float:
    """Roll (rad) about the hull's long axis, from an orientation quaternion."""
    return math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))


class SimpleRollFin(Node):
    def __init__(self):
        super().__init__("simple_roll_fin")

        self.declare_parameter("target_angle", 0.0)      # deg
        self.declare_parameter("full_fin_at", 30.0)      # deg of error for max fin
        self.declare_parameter("deadband", 2.0)          # deg: within this, stop correcting
        self.declare_parameter("max_fin_angle", 10.0)    # deg, roll budget
        self.declare_parameter("max_fin_rate", 30.0)     # deg/s slew limit
        self.declare_parameter("fin_topic", "/roll_hold/aileron")
        self.declare_parameter("control_rate", 20.0)     # Hz
        self.declare_parameter("fin_sign", 1.0)          # flip if it rolls the wrong way

        p = self.get_parameter
        self.target = math.radians(float(p("target_angle").value))
        self.max_fin = math.radians(float(p("max_fin_angle").value))
        self.kp = self.max_fin / math.radians(float(p("full_fin_at").value))
        self.deadband = math.radians(float(p("deadband").value))
        self.fin_sign = float(p("fin_sign").value)
        self.dt = 1.0 / float(p("control_rate").value)
        self.max_step = math.radians(float(p("max_fin_rate").value)) * self.dt

        self.roll = None
        self.fin = 0.0

        self.imu_subscription = self.create_subscription(Imu, "/imu", self.imu_callback, 10)
        self.fin_pub = self.create_publisher(Float64, p("fin_topic").value, 10)

        self.create_timer(self.dt, self.control_step)
        self.get_logger().info(
            f"simple roll fin: target {math.degrees(self.target):.1f} deg, "
            f"max fin {math.degrees(self.max_fin):.1f} deg at "
            f"{float(p('full_fin_at').value):.0f} deg error, "
            f"deadband {math.degrees(self.deadband):.1f} deg"
        )

    def imu_callback(self, msg: Imu):
        """Take roll only. The Imu message also carries angular_velocity,
        linear_acceleration and three covariance matrices; none are used."""
        q = msg.orientation
        self.roll = quaternion_to_roll(q.w, q.x, q.y, q.z)

    def control_step(self):
        if self.roll is None:
            return  # no IMU yet: leave the fins alone
        error = self.target - self.roll
        wanted = fin_angle_for(error, self.kp, self.deadband, self.max_fin)
        self.fin = slew_limit(wanted, self.fin, self.max_step)
        self.fin_pub.publish(Float64(data=float(self.fin_sign * self.fin)))


def main(args=None):
    rclpy.init(args=args)
    node = SimpleRollFin()
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
