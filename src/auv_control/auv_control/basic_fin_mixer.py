#!/usr/bin/env python3
"""Mix the depth and roll commands onto the horizontal fin pair."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


def clamp(value, limit):
    return max(-limit, min(limit, value))


class BasicFinMixer(Node):
    def __init__(self):
        super().__init__("basic_fin_mixer")

        self.declare_parameter("max_fin_angle", 45.0)   # deg, the fin stop
        # Held back from depth so roll always keeps this much fin and can
        # keep correcting, slowly, even while depth is asking for everything.
        self.declare_parameter("roll_reserve", 5.0)     # deg
        self.declare_parameter("publish_rate", 20.0)    # Hz
        # Per-fin multipliers, [Left, Right]: the URDF axes of the pair are
        # mirrored, so the second fin needs the opposite joint value to
        # deflect the same physical way.
        self.declare_parameter("fin_pair_signs", [1.0, -1.0])

        p = self.get_parameter
        self.max_fin = math.radians(float(p("max_fin_angle").value))
        self.roll_reserve = math.radians(float(p("roll_reserve").value))
        self.max_elevator = self.max_fin - self.roll_reserve
        self.fin_pair_signs = list(p("fin_pair_signs").value)

        self.elevator = 0.0
        self.aileron = 0.0

        self.create_subscription(Float64, "/depth_hold/elevator", self.elevator_callback, 10)
        self.create_subscription(Float64, "/roll_hold/aileron", self.aileron_callback, 10)
        self.fin_pub = self.create_publisher(Float64MultiArray, "/fin_controller/commands", 10)

        self.create_timer(1.0 / float(p("publish_rate").value), self.publish_commands)
        self.get_logger().info(
            f"basic fin mixer: Left/Right = elevator +- aileron, "
            f"fin stop {math.degrees(self.max_fin):.0f} deg, "
            f"elevator capped at {math.degrees(self.max_elevator):.0f} deg "
            f"({math.degrees(self.roll_reserve):.0f} deg reserved for roll)")

    def elevator_callback(self, msg: Float64):
        self.elevator = float(msg.data)

    def aileron_callback(self, msg: Float64):
        self.aileron = float(msg.data)

    def publish_commands(self):
        # Depth takes what it needs, but never the last roll_reserve degrees;
        # roll gets the rest, so it always has at least that much to correct
        # with and neither fin is ever asked for more than the stop.
        elevator = clamp(self.elevator, self.max_elevator)
        headroom = self.max_fin - abs(elevator)
        aileron = clamp(self.aileron, headroom)

        s = self.fin_pair_signs
        msg = Float64MultiArray()
        msg.data = [
            s[0] * (elevator + aileron),   # Fin_Left
            s[1] * (elevator - aileron),   # Fin_Right
        ]
        self.fin_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BasicFinMixer()
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
