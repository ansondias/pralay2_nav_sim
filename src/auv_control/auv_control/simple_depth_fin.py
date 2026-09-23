#!/usr/bin/env python3
"""Hold a target depth by driving the elevator fins."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Bool, Float64


def map_range(x: float, in_min: float, in_max: float,
              out_min: float, out_max: float) -> float:
    """Linear re-map of x from [in_min, in_max] to [out_min, out_max] (Arduino map())."""
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def fin_angle_for(error: float, threshold: float, max_fin: float) -> float:
    """Elevator angle (rad, +ve = nose up) for a depth error (m, +ve = too shallow).

    Too shallow needs nose DOWN, i.e. a negative elevator, hence the sign flip.
    """
    if error > threshold:
        # Too shallow, outside the band: full nose down.
        fin = -max_fin
    elif error < -threshold:
        # Too deep, outside the band: full nose up.
        fin = max_fin
    else:
        # Inside the band: proportional to the deviation.
        fin = -map_range(error, -threshold, threshold, -max_fin, max_fin)
    return fin


class SimpleDepthFin(Node):
    def __init__(self):
        super().__init__("simple_depth_fin")

        self.declare_parameter("target_depth", 2.0)      # m
        self.declare_parameter("depth_threshold", 0.3)   # m
        self.declare_parameter("max_fin_angle", math.radians(45.0))  # rad, 45 deg servo stop
        self.declare_parameter("depth_source", "pressure")  # "pressure" | "depth"
        self.declare_parameter("pressure_topic", "/depth/pressure")
        self.declare_parameter("depth_topic", "/bar30/depth")
        self.declare_parameter("elevator_topic", "/depth_hold/elevator")
        self.declare_parameter("control_rate", 20.0)     # Hz
        self.declare_parameter("elevator_sign", 1.0)     # flip if it climbs when it should dive
        # Pressure -> depth. PASCALS: 101325 Pa == 1013.25 hPa.
        self.declare_parameter("fluid_density", 1025.0)   # kg/m^3 (997 fresh water)
        self.declare_parameter("gravity", 9.80665)
        self.declare_parameter("surface_pressure_pa", 101325.0)
        self.declare_parameter("auto_zero", True)         # average the first samples instead
        self.declare_parameter("auto_zero_samples", 20)

        p = self.get_parameter
        self.target = float(p("target_depth").value)
        self.threshold = float(p("depth_threshold").value)
        self.max_fin = float(p("max_fin_angle").value)
        self.elevator_sign = float(p("elevator_sign").value)
        self.rho = float(p("fluid_density").value)
        self.g = float(p("gravity").value)
        self.surface_pressure = float(p("surface_pressure_pa").value)
        self.auto_zero = bool(p("auto_zero").value)
        self.auto_zero_samples = int(p("auto_zero_samples").value)
        self.zero_samples = []

        self.depth = None

        if str(p("depth_source").value) == "pressure":
            self.create_subscription(
                FluidPressure, p("pressure_topic").value, self.on_pressure, 10)
        else:
            self.create_subscription(Float64, p("depth_topic").value, self.on_depth, 10)
        self.create_subscription(Float64, "/depth_hold/target_depth", self.on_target, 10)
        self.elevator_pub = self.create_publisher(Float64, p("elevator_topic").value, 10)
        self.in_band_pub = self.create_publisher(Bool, "/depth_hold/in_band", 10)

        self.create_timer(1.0 / float(p("control_rate").value), self.control_step)
        self.get_logger().info(
            f"simple depth fin: target {self.target:.2f} m, band +-{self.threshold:.2f} m, "
            f"max fin {math.degrees(self.max_fin):.0f} deg"
        )

    def on_pressure(self, msg: FluidPressure):
        pressure = msg.fluid_pressure
        if self.auto_zero and len(self.zero_samples) < self.auto_zero_samples:
            # Still at the surface: collect the atmospheric baseline first.
            self.zero_samples.append(pressure)
            if len(self.zero_samples) == self.auto_zero_samples:
                self.surface_pressure = sum(self.zero_samples) / len(self.zero_samples)
                self.get_logger().info(
                    f"surface pressure zeroed at {self.surface_pressure:.0f} Pa")
            return
        self.depth = (pressure - self.surface_pressure) / (self.rho * self.g)

    def on_depth(self, msg: Float64):
        self.depth = msg.data

    def on_target(self, msg: Float64):
        self.target = max(0.0, msg.data)
        self.get_logger().info(f"new target depth {self.target:.2f} m")

    def control_step(self):
        if self.depth is None:
            return  # no depth yet: leave the fins alone
        error = self.target - self.depth
        elevator = self.elevator_sign * fin_angle_for(error, self.threshold, self.max_fin)
        self.elevator_pub.publish(Float64(data=float(elevator)))
        self.in_band_pub.publish(Bool(data=abs(error) <= self.threshold))


def main(args=None):
    rclpy.init(args=args)
    node = SimpleDepthFin()
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
