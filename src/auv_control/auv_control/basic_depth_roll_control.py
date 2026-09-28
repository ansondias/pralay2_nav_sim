#!/usr/bin/env python3
"""Hold depth and keep the hull level, both on the horizontal fin pair.

Publishes Fin_Left / Fin_Right already in /fin_controller/commands units and
joint sign convention; thrustmaster.py forwards them untouched while they are
fresh. If depth or IMU data goes stale this node stops publishing, and
thrustmaster falls back to the joystick depth axis.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


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
        self.declare_parameter("target_depth", 0.0)      # m
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
        # Fin_Left axis is +y and Fin_Right is -y in the URDF, so the same
        # trailing-edge-up deflection is +angle on the left and -angle on the
        # right (the joystick path in thrustmaster.py does the same mirror).
        # Output is therefore already in joint convention - thrustmaster must
        # NOT negate the right fin again, or elevator and aileron swap.
        self.declare_parameter("fin_pair_signs", [1.0, -1.0])   # [Left, Right]
        self.declare_parameter("elevator_sign", 1.0)
        # +aileron = Fin_Left trailing edge up + Fin_Right trailing edge down
        # = left side pushed down = NEGATIVE roll (REP-103, +roll = starboard
        # down). roll_to_aileron() returns +aileron for +roll error, so the
        # sign must be -1 or it rolls the hull further over.
        self.declare_parameter("aileron_sign", -1.0)
        # /fin_controller/commands is not radians: propulsion_controller maps
        # servo_deg = 90 - cmd * 45, i.e. 1.0 = 45 deg (kFinDegPerUnit).
        self.declare_parameter("fin_deg_per_unit", 45.0)
        self.declare_parameter("control_rate", 20.0)     # Hz
        # Stop publishing if depth or IMU is older than this, so thrustmaster
        # falls back instead of holding a fin angle computed from dead data.
        self.declare_parameter("sensor_timeout_sec", 0.5)

        p = self.get_parameter
        self.target_depth = float(p("target_depth").value)
        self.tolerance = float(p("tolerance").value)
        self.max_pitch = math.radians(float(p("max_pitch").value))
        # deg/m -> rad/m. Used raw, 20 "deg"/m was 20 rad/m, which saturated
        # max_pitch at 1.7 cm of error and made HOLDING a bang-bang.
        self.kp_depth = math.radians(float(p("kp_depth").value))
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
        self.rad_per_unit = math.radians(float(p("fin_deg_per_unit").value))
        self.sensor_timeout_sec = float(p("sensor_timeout_sec").value)

        self.depth = None
        self.depth_time = None
        self.imu_time = None
        self.pitch = 0.0
        self.roll = 0.0
        self.state = "DIVING"
        self.stale_warned = False

        # /depth/pose comes from auv_fusion pressure_depth.py, which already
        # handles the driver's hPa-in-FluidPressure units and captures surface
        # pressure at startup. Reading raw /depth/pressure here as Pascals put
        # the vehicle ~10 m above the surface, so it never left DIVING.
        self.depth_subscription = self.create_subscription(
            PoseWithCovarianceStamped, "/depth/pose", self.depth_callback, 10)
        self.imu_subscription = self.create_subscription(Imu, "/imu", self.imu_callback, 10)

        self.left_fin_pub = self.create_publisher(Float64, '/left_fin_pub', 10)
        self.right_fin_pub = self.create_publisher(Float64, '/right_fin_pub', 10)

        self.create_timer(1.0 / float(p("control_rate").value), self.control_step)
        self.get_logger().info(
            f"depth+roll: target {self.target_depth:.2f} m +-{self.tolerance:.2f} m, "
            f"fin stop {math.degrees(self.max_fin):.0f} deg, elevator capped at "
            f"{math.degrees(self.max_elevator):.0f} deg "
            f"({math.degrees(self.roll_reserve):.0f} deg reserved for roll)")

    def depth_callback(self, msg: PoseWithCovarianceStamped):
        self.depth = -msg.pose.pose.position.z   # z is ENU, down is negative
        self.depth_time = self.get_clock().now()

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        self.pitch = quaternion_to_pitch(q.w, q.x, q.y, q.z)
        self.roll = quaternion_to_roll(q.w, q.x, q.y, q.z)
        self.imu_time = self.get_clock().now()

    def age(self, stamp):
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9

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
        if self.depth_time is None or self.imu_time is None:
            return
        if (self.age(self.depth_time) > self.sensor_timeout_sec
                or self.age(self.imu_time) > self.sensor_timeout_sec):
            if not self.stale_warned:
                self.get_logger().warn(
                    f"depth/IMU older than {self.sensor_timeout_sec:.1f}s - "
                    "not publishing fin commands until both resume.")
                self.stale_warned = True
            return
        self.stale_warned = False

        elevator = self.elevator_sign * self.depth_to_elevator()
        aileron = self.aileron_sign * self.roll_to_aileron()

        # Depth is already capped at max_elevator, so roll always has at
        # least roll_reserve of fin left and no fin exceeds the servo stop.
        aileron = clamp(aileron, self.max_fin - abs(elevator))

        s = self.fin_pair_signs
        left_fin_value = s[0] * (elevator + aileron)   # Fin_Left, rad
        right_fin_value = s[1] * (elevator - aileron)  # Fin_Right, rad

        self.get_logger().info(
            f"[{self.state}] depth {self.depth:.2f} m, pitch "
            f"{math.degrees(self.pitch):.1f}, roll {math.degrees(self.roll):.1f} -> "
            f"left {math.degrees(left_fin_value):.1f} deg, "
            f"right {math.degrees(right_fin_value):.1f} deg",
            throttle_duration_sec=1.0)

        left_fin_msg = Float64()
        left_fin_msg.data = left_fin_value / self.rad_per_unit
        right_fin_msg = Float64()
        right_fin_msg.data = right_fin_value / self.rad_per_unit

        self.left_fin_pub.publish(left_fin_msg)
        self.right_fin_pub.publish(right_fin_msg)


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

