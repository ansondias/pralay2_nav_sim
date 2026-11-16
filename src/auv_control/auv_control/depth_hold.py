#!/usr/bin/env python3
"""Depth-hold controller for the Pralay-2: dive to a target depth and stay there.

Hardware this is written for
    * Blue Robotics Bar30 depth/pressure sensor (MS5837). Publishes either
      sensor_msgs/FluidPressure (Pa) or a ready-made depth in metres,
      selectable with the `depth_source` parameter.
    * Bosch BNO086 IMU as sensor_msgs/Imu: orientation quaternion (pitch)
      and angular_velocity.y (pitch rate).
    * Four servo fins, +-45 deg. The LEFT and RIGHT fins are the elevator
      pair (pitch = up/down); the TOP and BOTTOM fins are the rudder pair
      (left/right). This node produces only the elevator angle; it is
      published on /depth_hold/elevator and mixed into the four-fin command
      by cmd_vel_to_actuators, which keeps steering (rudder) coming from
      cmd_vel. So the same fin_controller and the same wiring serve both.

What it does
    1. DIVING   : from the surface, pitch the nose down (limited to
                  `max_pitch`) until the depth first enters the tolerance
                  band around the target.
    2. HOLDING  : regulate depth to the target with a cascaded loop --
                  depth error -> desired pitch -> elevator angle -- and
                  report `in_band` while depth stays within
                  [target - tolerance, target + tolerance] (default
                  1.0 m +- 0.2 m, i.e. 0.8..1.2 m).
    3. SURFACING: if `surface_on_timeout` and the depth sensor goes silent,
                  fins go to full nose-up so the real AUV comes up.

Control law (all signs verified in a kinematic simulation, and every sign
is also a parameter so it can be flipped on the AUV without touching code):
    depth_error   = target_depth - depth              (m, +ve: too shallow)
    desired_pitch = clamp( Kp_d*e + Ki_d*int(e) + Kd_d*(-depth_rate), +-max_pitch )
                    (+ve = nose DOWN in the ROS FLU body frame)
    pitch_error   = desired_pitch - pitch
    elevator      = elevator_sign * clamp( Kp_p*pitch_error - Kd_p*pitch_rate, +-max_elevator )
                    (elevator +ve = trailing edge up = nose UP, matching
                     cmd_vel_to_actuators; hence the sign flip inside)
Fins only work with water flowing over them: with `speed_scaling` on, the
elevator command is scaled by (reference_speed / speed)^2 (clamped) using
the surge speed from odometry, so the loop stays stable across speeds.

Topics
    sub  depth sensor  (`pressure_topic` FluidPressure, or `depth_topic` Float64)
    sub  imu           (`imu_topic`, sensor_msgs/Imu)
    sub  odom          (`odom_topic`, nav_msgs/Odometry, only for speed scaling)
    sub  /depth_hold/target_depth   std_msgs/Float64  (m below surface, live re-target)
    sub  /depth_hold/enable         std_msgs/Bool
    pub  /depth_hold/elevator       std_msgs/Float64  (rad, elevator angle)
    pub  /depth_hold/depth          std_msgs/Float64  (filtered depth, m)
    pub  /depth_hold/state          std_msgs/String   ("DIVING", "HOLDING", "SURFACING", "DISABLED")
    pub  /depth_hold/in_band        std_msgs/Bool
    pub  /depth_setpoint            std_msgs/Float64  (only if sim_mode: forwards the target
                                                       to the Gazebo depth hold, since the
                                                       simulated fins generate no lift)

Run:  ros2 launch auv_control depth_hold.launch.py   (config/depth_hold.yaml)
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Imu
from std_msgs.msg import Bool, Float64, String


def quaternion_to_pitch(q: Quaternion) -> float:
    """Pitch (rad) of a body-frame quaternion, ROS FLU convention (+ = nose down)."""
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    return math.asin(sinp)


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class DepthHold(Node):
    def __init__(self):
        super().__init__("depth_hold")

        # ---- target and tolerance --------------------------------------
        self.declare_parameter("target_depth", 1.0)      # m below surface
        self.declare_parameter("tolerance", 0.2)         # m: 0.8 .. 1.2 is "in band"
        self.declare_parameter("enabled", True)

        # ---- sensors -----------------------------------------------------
        self.declare_parameter("depth_source", "pressure")   # "pressure" | "depth"
        self.declare_parameter("pressure_topic", "/bar30/pressure")
        self.declare_parameter("depth_topic", "/bar30/depth")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("fluid_density", 1025.0)      # kg/m^3 (997 fresh water)
        self.declare_parameter("gravity", 9.80665)
        # Surface (atmospheric) pressure. auto_zero averages the first
        # `auto_zero_samples` readings on start-up (the AUV must be at the
        # surface then); otherwise surface_pressure_pa is used as-is.
        self.declare_parameter("surface_pressure_pa", 101325.0)
        self.declare_parameter("auto_zero", True)
        self.declare_parameter("auto_zero_samples", 20)
        self.declare_parameter("depth_filter_alpha", 0.3)    # low-pass on depth, 0..1
        self.declare_parameter("sensor_timeout", 1.0)        # s without depth -> failsafe
        self.declare_parameter("surface_on_timeout", True)

        # ---- outer loop: depth -> desired pitch ---------------------------
        self.declare_parameter("kp_depth", 0.6)     # rad of pitch per m of error
        self.declare_parameter("ki_depth", 0.05)    # rad per m*s
        self.declare_parameter("kd_depth", 0.8)     # rad per (m/s) of depth rate
        self.declare_parameter("max_pitch", 0.35)   # rad (~20 deg) nose-down/up limit
        self.declare_parameter("integrator_band", 0.5)   # m: integrate only inside +-band

        # ---- inner loop: pitch -> elevator -------------------------------
        self.declare_parameter("kp_pitch", 2.0)     # rad elevator per rad pitch error
        self.declare_parameter("kd_pitch", 0.4)     # rad elevator per (rad/s) pitch rate
        self.declare_parameter("max_elevator", 0.70)  # rad (~40 deg): under the 45 deg servo stop
        self.declare_parameter("elevator_sign", 1.0)  # flip if the AUV climbs when it should dive
        self.declare_parameter("pitch_sign", 1.0)     # flip if the IMU reports nose-up as +ve

        # ---- speed scaling (fin authority ~ v^2) --------------------------
        self.declare_parameter("speed_scaling", True)
        self.declare_parameter("reference_speed", 1.0)   # m/s the gains are tuned at
        self.declare_parameter("min_speed_for_scaling", 0.3)
        self.declare_parameter("max_scale", 3.0)

        # ---- misc ----------------------------------------------------------
        self.declare_parameter("control_rate", 20.0)     # Hz
        self.declare_parameter("sim_mode", False)        # also publish /depth_setpoint for Gazebo

        p = self.get_parameter
        self.target = float(p("target_depth").value)
        self.tolerance = float(p("tolerance").value)
        self.enabled = bool(p("enabled").value)
        self.rho = float(p("fluid_density").value)
        self.g = float(p("gravity").value)
        self.surface_pressure = float(p("surface_pressure_pa").value)
        self.auto_zero = bool(p("auto_zero").value)
        self.auto_zero_samples = int(p("auto_zero_samples").value)
        self.alpha = float(p("depth_filter_alpha").value)
        self.sensor_timeout = float(p("sensor_timeout").value)
        self.surface_on_timeout = bool(p("surface_on_timeout").value)
        self.kp_d = float(p("kp_depth").value)
        self.ki_d = float(p("ki_depth").value)
        self.kd_d = float(p("kd_depth").value)
        self.max_pitch = float(p("max_pitch").value)
        self.integrator_band = float(p("integrator_band").value)
        self.kp_p = float(p("kp_pitch").value)
        self.kd_p = float(p("kd_pitch").value)
        self.max_elevator = float(p("max_elevator").value)
        self.elevator_sign = float(p("elevator_sign").value)
        self.pitch_sign = float(p("pitch_sign").value)
        self.speed_scaling = bool(p("speed_scaling").value)
        self.reference_speed = float(p("reference_speed").value)
        self.min_speed_for_scaling = float(p("min_speed_for_scaling").value)
        self.max_scale = float(p("max_scale").value)
        self.sim_mode = bool(p("sim_mode").value)

        # ---- state -----------------------------------------------------------
        self.depth = None            # filtered, m (+ down)
        self.depth_raw_prev = None
        self.depth_rate = 0.0        # m/s (+ = going deeper)
        self.last_depth_time = None
        self.zero_samples = []
        self.pitch = 0.0             # rad (+ nose down)
        self.pitch_rate = 0.0        # rad/s
        self.have_imu = False
        self.speed = None            # m/s surge
        self.integral = 0.0
        self.state = "DISABLED" if not self.enabled else "DIVING"
        self.reached_band_once = False

        # ---- ROS I/O -------------------------------------------------------
        if p("depth_source").value == "pressure":
            self.create_subscription(FluidPressure, p("pressure_topic").value, self.on_pressure, 10)
        else:
            self.create_subscription(Float64, p("depth_topic").value, self.on_depth_msg, 10)
        self.create_subscription(Imu, p("imu_topic").value, self.on_imu, 20)
        self.create_subscription(Odometry, p("odom_topic").value, self.on_odom, 10)
        self.create_subscription(Float64, "/depth_hold/target_depth", self.on_target, 10)
        self.create_subscription(Bool, "/depth_hold/enable", self.on_enable, 10)

        self.elevator_pub = self.create_publisher(Float64, "/depth_hold/elevator", 10)
        self.depth_pub = self.create_publisher(Float64, "/depth_hold/depth", 10)
        self.state_pub = self.create_publisher(String, "/depth_hold/state", 10)
        self.in_band_pub = self.create_publisher(Bool, "/depth_hold/in_band", 10)
        self.setpoint_pub = (
            self.create_publisher(Float64, "/depth_setpoint", 10) if self.sim_mode else None
        )

        self.dt = 1.0 / float(p("control_rate").value)
        self.create_timer(self.dt, self.control_step)
        self.get_logger().info(
            f"depth hold: target {self.target:.2f} m, band +-{self.tolerance:.2f} m, "
            f"max pitch {math.degrees(self.max_pitch):.0f} deg, max elevator "
            f"{math.degrees(self.max_elevator):.0f} deg"
        )

    # ------------------------------------------------------------------ inputs
    def on_pressure(self, msg: FluidPressure):
        pressure = msg.fluid_pressure
        if self.auto_zero and len(self.zero_samples) < self.auto_zero_samples:
            self.zero_samples.append(pressure)
            if len(self.zero_samples) == self.auto_zero_samples:
                self.surface_pressure = sum(self.zero_samples) / len(self.zero_samples)
                self.get_logger().info(f"surface pressure zeroed at {self.surface_pressure:.0f} Pa")
            return
        depth = (pressure - self.surface_pressure) / (self.rho * self.g)
        self.update_depth(depth)

    def on_depth_msg(self, msg: Float64):
        self.update_depth(msg.data)

    def update_depth(self, depth_raw: float):
        now = self.get_clock().now()
        if self.depth is None:
            self.depth = depth_raw
        else:
            self.depth = self.alpha * depth_raw + (1.0 - self.alpha) * self.depth
        if self.last_depth_time is not None and self.depth_raw_prev is not None:
            dt = (now - self.last_depth_time).nanoseconds * 1e-9
            if dt > 1e-3:
                rate = (self.depth - self.depth_raw_prev) / dt
                self.depth_rate = 0.5 * rate + 0.5 * self.depth_rate
        self.depth_raw_prev = self.depth
        self.last_depth_time = now

    def on_imu(self, msg: Imu):
        self.pitch = self.pitch_sign * quaternion_to_pitch(msg.orientation)
        self.pitch_rate = self.pitch_sign * msg.angular_velocity.y
        self.have_imu = True

    def on_odom(self, msg: Odometry):
        self.speed = msg.twist.twist.linear.x

    def on_target(self, msg: Float64):
        self.target = max(0.0, msg.data)
        self.integral = 0.0
        self.reached_band_once = False
        if self.enabled:
            self.state = "DIVING"
        self.get_logger().info(f"new target depth {self.target:.2f} m")

    def on_enable(self, msg: Bool):
        self.enabled = msg.data
        self.integral = 0.0
        self.state = "DIVING" if self.enabled else "DISABLED"

    # ---------------------------------------------------------------- control
    def control_step(self):
        if not self.enabled:
            self.publish(0.0, "DISABLED", False)
            return
        if self.sim_mode and self.setpoint_pub is not None:
            self.setpoint_pub.publish(Float64(data=self.target))

        now = self.get_clock().now()
        sensor_ok = (
            self.depth is not None
            and self.last_depth_time is not None
            and (now - self.last_depth_time).nanoseconds * 1e-9 < self.sensor_timeout
        )
        if not sensor_ok:
            if self.surface_on_timeout and self.depth is not None:
                # Lost the depth sensor: full nose-up, come to the surface.
                self.publish(self.elevator_sign * self.max_elevator, "SURFACING", False)
            else:
                self.publish(0.0, "DIVING", False)  # no data yet: fins neutral
            return

        error = self.target - self.depth            # +ve: too shallow, go down
        in_band = abs(error) <= self.tolerance
        if in_band:
            self.reached_band_once = True
        self.state = "HOLDING" if self.reached_band_once else "DIVING"

        # Outer loop: depth error -> desired pitch (+ nose down).
        if abs(error) <= self.integrator_band:
            self.integral = clamp(self.integral + error * self.dt, self.max_pitch / max(self.ki_d, 1e-6))
        else:
            self.integral = 0.0
        desired_pitch = clamp(
            self.kp_d * error + self.ki_d * self.integral - self.kd_d * self.depth_rate,
            self.max_pitch,
        )

        # Inner loop: pitch error -> elevator. Positive elevator = nose UP,
        # so a nose-down demand needs a negative elevator: the minus sign.
        if self.have_imu:
            pitch_error = desired_pitch - self.pitch
            elevator = -(self.kp_p * pitch_error - self.kd_p * self.pitch_rate)
        else:
            # No IMU: drive the elevator straight from the depth loop.
            elevator = -desired_pitch * self.kp_p

        if self.speed_scaling and self.speed is not None:
            v = max(abs(self.speed), self.min_speed_for_scaling)
            elevator *= min(self.max_scale, (self.reference_speed / v) ** 2)

        elevator = self.elevator_sign * clamp(elevator, self.max_elevator)
        self.publish(elevator, self.state, in_band)

    def publish(self, elevator: float, state: str, in_band: bool):
        self.elevator_pub.publish(Float64(data=float(elevator)))
        self.state_pub.publish(String(data=state))
        self.in_band_pub.publish(Bool(data=bool(in_band)))
        if self.depth is not None:
            self.depth_pub.publish(Float64(data=float(self.depth)))


def main(args=None):
    rclpy.init(args=args)
    node = DepthHold()
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
