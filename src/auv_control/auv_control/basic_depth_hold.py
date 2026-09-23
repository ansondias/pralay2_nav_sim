#!/usr/bin/env python3
"""Dive to a target depth and hold it with a cascaded depth -> pitch -> elevator loop."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Imu
from std_msgs.msg import Bool, Float64, String


def clamp(value, limit):
    return max(-limit, min(limit, value))


def quaternion_to_pitch(w, x, y, z):
    sinp = clamp(2.0 * (w * y - z * x), 1.0)
    return math.asin(sinp)


class BasicDepthHold(Node):
    def __init__(self):
        super().__init__("basic_depth_hold")

        self.declare_parameter("target_depth", 1.0)    # m
        self.declare_parameter("tolerance", 0.2)       # m: 0.8 .. 1.2 is in band
        self.declare_parameter("max_pitch", 20.0)      # deg, nose-down limit
        self.declare_parameter("kp_depth", 20.0)       # deg of pitch per m of error
        self.declare_parameter("kp_pitch", 2.0)        # deg of elevator per deg of pitch error
        self.declare_parameter("max_elevator", 45.0)   # deg
        self.declare_parameter("control_rate", 20.0)   # Hz
        self.declare_parameter("elevator_sign", 1.0)
        # Pressure -> depth. FluidPressure is in PASCALS: 101325 Pa == 1013.25 hPa.
        self.declare_parameter("fluid_density", 1025.0)
        self.declare_parameter("gravity", 9.80665)
        self.declare_parameter("surface_pressure_pa", 101325.0)

        p = self.get_parameter
        self.target = float(p("target_depth").value)
        self.tolerance = float(p("tolerance").value)
        self.max_pitch = math.radians(float(p("max_pitch").value))
        self.kp_depth = float(p("kp_depth").value)
        self.kp_pitch = float(p("kp_pitch").value)
        self.max_elevator = math.radians(float(p("max_elevator").value))
        self.elevator_sign = float(p("elevator_sign").value)
        self.rho = float(p("fluid_density").value)
        self.g = float(p("gravity").value)
        self.surface_pressure = float(p("surface_pressure_pa").value)

        self.depth = None
        self.pitch = 0.0
        self.state = "DIVING"

        self.current_pressure_subscription = self.create_subscription(
            FluidPressure, "/depth/pressure", self.pressure_callback, 10)
        self.imu_subscription = self.create_subscription(Imu, "/imu", self.imu_callback, 10)

        self.elevator_pub = self.create_publisher(Float64, "/depth_hold/elevator", 10)
        self.in_band_pub = self.create_publisher(Bool, "/depth_hold/in_band", 10)
        self.state_pub = self.create_publisher(String, "/depth_hold/state", 10)

        self.create_timer(1.0 / float(p("control_rate").value), self.control_step)
        self.get_logger().info(
            f"basic depth hold: target {self.target:.2f} m "
            f"+-{self.tolerance:.2f} m, max pitch {math.degrees(self.max_pitch):.0f} deg")

    def pressure_callback(self, msg: FluidPressure):
        self.depth = (msg.fluid_pressure - self.surface_pressure) / (self.rho * self.g)

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        self.pitch = quaternion_to_pitch(q.w, q.x, q.y, q.z)

    def control_step(self):
        if self.depth is None:
            return

        depth_error = self.target - self.depth      # +ve: too shallow, go down
        in_band = abs(depth_error) <= self.tolerance

        # DIVING until the band is reached for the first time, then HOLDING.
        if in_band:
            self.state = "HOLDING"

        if self.state == "DIVING":
            # Full nose-down (or up), limited to max_pitch.
            desired_pitch = math.copysign(self.max_pitch, depth_error)
        else:
            # Outer loop: proportional depth error -> desired pitch.
            desired_pitch = clamp(self.kp_depth * depth_error, self.max_pitch)

        # Inner loop: pitch error -> elevator. Desired pitch is +ve nose down,
        # elevator is +ve nose up, hence the sign flip.
        pitch_error = desired_pitch - self.pitch
        elevator = clamp(-self.kp_pitch * pitch_error, self.max_elevator)

        self.elevator_pub.publish(Float64(data=float(self.elevator_sign * elevator)))
        self.in_band_pub.publish(Bool(data=bool(in_band)))
        self.state_pub.publish(String(data=self.state))


def main(args=None):
    rclpy.init(args=args)
    node = BasicDepthHold()
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
