#!/usr/bin/env python3
"""Hold depth and keep the hull level, both on the horizontal fin pair."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Imu
from std_msgs.msg import Float64MultiArray


def clamp(value, limit):
    return max(-limit, min(limit, value))


def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def quaternion_to_roll(w, x, y, z):
    return math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))


def quaternion_to_pitch(w, x, y, z):
    return math.asin(clamp(2.0 * (w * y - z * x), 1.0))


class DepthRollControl(Node):
    def __init__(self):
        super().__init__("depth_roll_control")

        # Depth
        self.declare_parameter("target_depth", 1.0)      # m
        self.declare_parameter("tolerance", 0.2)         # m: 0.8 .. 1.2 is in band
        self.declare_parameter("max_pitch", 20.0)        # deg, nose-down limit
        self.declare_parameter("kp_depth", 20.0)         # deg of pitch per m of error
        self.declare_parameter("kp_pitch", 2.0)          # deg of elevator per deg of pitch error

        # Roll
        self.declare_parameter("roll_target", 0.0)       # deg, 0 = level
        self.declare_parameter("roll_threshold", 15.0)   # deg
        self.declare_parameter("max_aileron", 10.0)      # deg

        # Fins
        self.declare_parameter("max_fin_angle", 38.0)    # deg, the servo stop
        # Held back from depth so roll always keeps this much fin and can
        # keep correcting, slowly, even while depth is asking for everything.
        self.declare_parameter("roll_reserve", 5.0)      # deg
        self.declare_parameter("fin_pair_signs", [1.0, -1.0])   # [Left, Right]
        self.declare_parameter("elevator_sign", 1.0)
        self.declare_parameter("aileron_sign", 1.0)
        self.declare_parameter("control_rate", 20.0)     # Hz

        # Pressure -> depth. FluidPressure is in PASCALS: 101325 Pa == 1013.25 hPa.
        self.declare_parameter("fluid_density", 1025.0)
        self.declare_parameter("gravity", 9.80665)
        self.declare_parameter("surface_pressure_pa", 101325.0)

        p = self.get_parameter
        self.target_depth = float(p("target_depth").value)
        self.tolerance = float(p("tolerance").value)
        self.max_pitch = math.radians(float(p("max_pitch").value))
        self.kp_depth = float(p("kp_depth").value)
        self.kp_pitch = float(p("kp_pitch").value)
        self.roll_target = math.radians(float(p("roll_target").value))
        self.roll_threshold = math.radians(float(p("roll_threshold").value))
        self.max_aileron = math.radians(float(p("max_aileron").value))
        self.max_fin = math.radians(float(p("max_fin_angle").value))
        self.roll_reserve = math.radians(float(p("roll_reserve").value))
        self.max_elevator = self.max_fin - self.roll_reserve
        self.fin_pair_signs = list(p("fin_pair_signs").value)
        self.elevator_sign = float(p("elevator_sign").value)
        self.aileron_sign = float(p("aileron_sign").value)
        self.rho = float(p("fluid_density").value)
        self.g = float(p("gravity").value)
        self.surface_pressure = float(p("surface_pressure_pa").value)

        self.depth = None
        self.pitch = 0.0
        self.roll = 0.0
        self.state = "DIVING"

        self.current_pressure_subscription = self.create_subscription(
            FluidPressure, "/depth/pressure", self.pressure_callback, 10)
        self.imu_subscription = self.create_subscription(Imu, "/imu", self.imu_callback, 10)
        self.fin_pub = self.create_publisher(Float64MultiArray, "/fin_controller/commands", 10)

        self.create_timer(1.0 / float(p("control_rate").value), self.control_step)
        self.get_logger().info(
            f"depth+roll: target {self.target_depth:.2f} m +-{self.tolerance:.2f} m, "
            f"fin stop {math.degrees(self.max_fin):.0f} deg, elevator capped at "
            f"{math.degrees(self.max_elevator):.0f} deg "
            f"({math.degrees(self.roll_reserve):.0f} deg reserved for roll)")

    def pressure_callback(self, msg: FluidPressure):
        self.depth = (msg.fluid_pressure - self.surface_pressure) / (self.rho * self.g)

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        self.pitch = quaternion_to_pitch(q.w, q.x, q.y, q.z)
        self.roll = quaternion_to_roll(q.w, q.x, q.y, q.z)

    def depth_to_elevator(self):
        depth_error = self.target_depth - self.depth      # +ve: too shallow, go down
        if abs(depth_error) <= self.tolerance:
            self.state = "HOLDING"

        if self.state == "DIVING":
            # Full nose-down (or up), limited to max_pitch.
            desired_pitch = math.copysign(self.max_pitch, depth_error)
        else:
            desired_pitch = clamp(self.kp_depth * depth_error, self.max_pitch)

        # Desired pitch is +ve nose down, elevator is +ve nose up: sign flip.
        pitch_error = desired_pitch - self.pitch
        return clamp(-self.kp_pitch * pitch_error, self.max_elevator)

    def roll_to_aileron(self):
        error = self.roll_target - self.roll
        if error > self.roll_threshold:
            return self.max_aileron
        if error < -self.roll_threshold:
            return -self.max_aileron
        return map_range(error, -self.roll_threshold, self.roll_threshold,
                         -self.max_aileron, self.max_aileron)

    def control_step(self):
        if self.depth is None:
            return

        elevator = self.elevator_sign * self.depth_to_elevator()
        aileron = self.aileron_sign * self.roll_to_aileron()

        # Depth is already capped at max_elevator, so roll always has at
        # least roll_reserve of fin left and no fin exceeds the servo stop.
        aileron = clamp(aileron, self.max_fin - abs(elevator))

        s = self.fin_pair_signs
        msg = Float64MultiArray()
        msg.data = [
            s[0] * (elevator + aileron),   # Fin_Left
            s[1] * (elevator - aileron),   # Fin_Right
        ]
        self.fin_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthRollControl()
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
