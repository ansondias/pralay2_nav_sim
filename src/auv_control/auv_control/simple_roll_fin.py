#!/usr/bin/env python3
"""Simple proportional attitude-to-fin controller for the Pralay-2 (roll by default).

Unlike simple_depth_fin.py there is deliberately NO "full fin" regime here.
A turn banks the hull on its own, and the hull self-rights (centre of
buoyancy above centre of gravity), so a controller that slams the fins the
moment roll crosses a threshold fights a transient that was already going
to correct itself -- reactive, wasteful, and it steals rudder authority
mid-turn. Instead the fin follows a plain proportional law, kept calm by
a rate limit on the output rather than by any derivative term:

    error    = target_angle - angle            (rad, from the IMU)
    fin      = kp * error                      (more roll -> more fin)
    fin      = 0 inside +-deadband             (2 deg: close enough, stop
                                                correcting; the hull self-rights)
    fin      = clamp(fin, +-max_fin_angle)     (soft saturation, no step)
    fin      = slew-limited to max_fin_rate    (no sudden fin jumps)

kp is set so max_fin_angle is reached at `full_fin_at` degrees of error:
    kp = max_fin_angle / full_fin_at
so the response is proportional over the whole useful range and only
flattens out at the budget.

Parameters (angles in deg, rates in deg/s; converted to rad internally)
    target_angle     wanted attitude                       (default 0 = level)
    full_fin_at      error at which the fin reaches max    (default 30)
    deadband         no output inside +-this               (default 2)
    max_fin_angle    fin clamp = the roll budget           (default 10, so the
                                                           rudder pair keeps
                                                           35 of the 45 deg stop)
    max_fin_rate     slew limit on the output, deg/s       (default 30)
    axis             "roll" | "pitch"                      (default roll)

Wiring
    sub  imu_topic   sensor_msgs/Imu    orientation quaternion (/imu)
    pub  fin_topic   std_msgs/Float64   differential (aileron) fin angle, rad
                                        (/roll_hold/aileron)

The output is the *differential* command for a fin pair (one fin +, the
other -). cmd_vel_to_actuators mixes it into the horizontal pair as
`Fin_Left = elevator + aileron`, `Fin_Right = elevator - aileron`, so depth
gets the pair's average and roll its difference. How much of the fin stop
each may use depends on whether depth is holding: see share_elevator_pair
there. Fins only act with forward speed.

    ros2 run auv_control simple_roll_fin --ros-args -p full_fin_at:=30.0
"""

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
    return math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))


def quaternion_to_pitch(w: float, x: float, y: float, z: float) -> float:
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return math.asin(sinp)


class SimpleRollFin(Node):
    def __init__(self):
        super().__init__("simple_roll_fin")

        self.declare_parameter("target_angle", 0.0)      # deg
        self.declare_parameter("full_fin_at", 30.0)      # deg of error for max fin
        self.declare_parameter("deadband", 2.0)          # deg: within this, stop correcting
        self.declare_parameter("max_fin_angle", 10.0)    # deg, roll budget
        self.declare_parameter("max_fin_rate", 30.0)     # deg/s slew limit
        self.declare_parameter("axis", "roll")           # "roll" | "pitch"
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("fin_topic", "/roll_hold/aileron")
        self.declare_parameter("control_rate", 20.0)     # Hz
        self.declare_parameter("fin_sign", 1.0)          # flip if it rolls the wrong way

        p = self.get_parameter
        self.target = math.radians(float(p("target_angle").value))
        self.max_fin = math.radians(float(p("max_fin_angle").value))
        self.kp = self.max_fin / math.radians(float(p("full_fin_at").value))
        self.deadband = math.radians(float(p("deadband").value))
        self.axis = str(p("axis").value)
        self.fin_sign = float(p("fin_sign").value)
        self.dt = 1.0 / float(p("control_rate").value)
        self.max_step = math.radians(float(p("max_fin_rate").value)) * self.dt

        self.angle = None
        self.fin = 0.0

        self.create_subscription(Imu, p("imu_topic").value, self.on_imu, 20)
        self.fin_pub = self.create_publisher(Float64, p("fin_topic").value, 10)

        self.create_timer(self.dt, self.control_step)
        self.get_logger().info(
            f"simple {self.axis} fin: target {math.degrees(self.target):.1f} deg, "
            f"max fin {math.degrees(self.max_fin):.1f} deg at "
            f"{float(p('full_fin_at').value):.0f} deg error, "
            f"deadband {math.degrees(self.deadband):.1f} deg"
        )

    def on_imu(self, msg: Imu):
        q = msg.orientation
        if self.axis == "pitch":
            self.angle = quaternion_to_pitch(q.w, q.x, q.y, q.z)
        else:
            self.angle = quaternion_to_roll(q.w, q.x, q.y, q.z)

    def control_step(self):
        if self.angle is None:
            return  # no IMU yet: leave the fins alone
        error = self.target - self.angle
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
