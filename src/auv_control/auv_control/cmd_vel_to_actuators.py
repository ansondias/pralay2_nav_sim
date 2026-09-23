#!/usr/bin/env python3
"""Mix the depth and roll commands onto the horizontal fin pair."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Float64MultiArray


class CmdVelToActuators(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_actuators")

        # 0.80 rad, deliberately inside the URDF's +-0.9076 joint limit: in
        # Gazebo the fins are position-servoed through gz_ros2_control, and
        # commanding exactly the hard stop makes the servo fight the joint
        # limit constraint every step, which injects impulses into the hull
        # (it tumbled during a Nav2 Spin recovery with fins at full lock).
        self.declare_parameter("max_fin_angle", 0.80)
        self.declare_parameter("elevator_sign", 1.0)
        # Per-fin multipliers in controller order [Left, Right].
        self.declare_parameter("fin_pair_signs", [1.0, -1.0])
        self.declare_parameter("publish_rate", 20.0)
        # An input older than this counts as absent (that controller is not
        # running), and its contribution falls back to zero.
        self.declare_parameter("input_timeout", 0.5)
        self.declare_parameter("elevator_topic", "/depth_hold/elevator")
        self.declare_parameter("aileron_topic", "/roll_hold/aileron")
        self.declare_parameter("in_band_topic", "/depth_hold/in_band")
        # Hold limits sum to 45 deg so no fin clips while holding depth.
        self.declare_parameter("elevator_hold_limit", math.radians(35.0))
        self.declare_parameter("aileron_hold_limit", math.radians(10.0))
        self.declare_parameter("elevator_full_limit", math.radians(45.0))
        self.declare_parameter("fin_commands_topic", "/fin_controller/commands")

        self.max_fin_angle = self.get_parameter("max_fin_angle").value
        self.elevator_sign = self.get_parameter("elevator_sign").value
        self.fin_pair_signs = list(self.get_parameter("fin_pair_signs").value)
        self.input_timeout = self.get_parameter("input_timeout").value
        self.elevator_hold_limit = self.get_parameter("elevator_hold_limit").value
        self.aileron_hold_limit = self.get_parameter("aileron_hold_limit").value
        self.elevator_full_limit = self.get_parameter("elevator_full_limit").value
        if len(self.fin_pair_signs) != 2:
            raise ValueError("fin_pair_signs must have 2 entries [Left, Right]")

        self.fin_pub = self.create_publisher(
            Float64MultiArray, self.get_parameter("fin_commands_topic").value, 10
        )

        self.elevator = 0.0
        self.elevator_time = None
        self.aileron = 0.0
        self.aileron_time = None
        self.depth_in_band = False

        self.create_subscription(
            Float64, self.get_parameter("elevator_topic").value, self.on_elevator, 10
        )
        self.create_subscription(
            Float64, self.get_parameter("aileron_topic").value, self.on_aileron, 10
        )
        self.create_subscription(
            Bool, self.get_parameter("in_band_topic").value, self.on_in_band, 10
        )

        period = 1.0 / self.get_parameter("publish_rate").value
        self.create_timer(period, self.publish_commands)

        self.get_logger().info(
            "fin mixer: Left/Right = elevator +- aileron; "
            "hold budget %.0f/%.0f deg, full elevator %.0f deg"
            % (
                math.degrees(self.elevator_hold_limit),
                math.degrees(self.aileron_hold_limit),
                math.degrees(self.elevator_full_limit),
            )
        )

    def on_elevator(self, msg: Float64):
        self.elevator = float(msg.data)
        self.elevator_time = self.get_clock().now()

    def on_aileron(self, msg: Float64):
        self.aileron = float(msg.data)
        self.aileron_time = self.get_clock().now()

    def on_in_band(self, msg: Bool):
        self.depth_in_band = bool(msg.data)

    def _fresh(self, value, stamp):
        """value if it arrived within input_timeout, else 0 (controller absent)."""
        if stamp is None:
            return 0.0
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return value if age <= self.input_timeout else 0.0

    def share_elevator_pair(self, elevator, aileron):
        """Split the horizontal pair's travel between depth and roll.

        Holding depth (in band): fixed 35/10 deg budget, so neither fin can
        clip. Diving/climbing (out of band): depth may use the full 45 deg
        and roll only gets whatever headroom is left, down to none.
        """
        if self.depth_in_band:
            elevator = self._clamp(elevator, self.elevator_hold_limit)
            aileron = self._clamp(aileron, self.aileron_hold_limit)
        else:
            elevator = self._clamp(elevator, self.elevator_full_limit)
            headroom = max(0.0, self.elevator_full_limit - abs(elevator))
            aileron = self._clamp(aileron, headroom)
        return elevator, aileron

    def compute(self):
        elevator = self.elevator_sign * self._clamp(
            self._fresh(self.elevator, self.elevator_time), self.max_fin_angle
        )
        aileron = self._fresh(self.aileron, self.aileron_time)
        elevator, aileron = self.share_elevator_pair(elevator, aileron)

        s = self.fin_pair_signs
        return [
            s[0] * (elevator + aileron),  # Fin_Left   (depth + roll)
            s[1] * (elevator - aileron),  # Fin_Right  (depth - roll)
        ]

    def publish_commands(self):
        fin_msg = Float64MultiArray()
        fin_msg.data = self.compute()
        self.fin_pub.publish(fin_msg)

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToActuators()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C / launch shutdown: exit quietly
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
