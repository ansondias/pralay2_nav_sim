#!/usr/bin/env python3
"""Map /cmd_vel onto the Pralay-2's fin and propeller joint commands.

The vehicle is fin-steered with one stern propeller: a Twist is not something
the actuators understand directly, so this node turns it into

  * a rudder angle for the vertical fin pair  (Fin_Top / Fin_Bottom)
  * an elevator angle for the horizontal pair (Fin_Left / Fin_Right)
  * a propeller speed                          (T1_Joint)

and publishes them on the ros2_control forward-command topics that
auv_control/config/controllers.yaml defines:

  /fin_controller/commands       Float64MultiArray, 4 fins, controller order
                                 [Fin_Top, Fin_Left, Fin_Right, Fin_Bottom]
  /thruster_controller/commands  Float64MultiArray, 1 value

Those topics are the same on the real AUV and in Gazebo; only the numbers
differ (see the thruster_scale parameter). In the Gazebo build the hull is
actually moved by auv_current_plugin::ThrustControl (a cmd_vel -> force P
controller on base_link), so what this node drives is the *kinematic*
picture: fins deflect and the propeller spins the way the real hardware
would for that command. That is what RViz and the Gazebo GUI show, and what
the joint_state_broadcaster reports back on /joint_states.

Rudder angle uses the bicycle model the rest of the sandbox already assumes
(ThrustControl and Nav2's min_turning_radius): tan(delta) = L * omega / v.
Elevator angle is the commanded flight-path angle, atan2(vz, |vx|). Both are
clamped to +-max_fin_angle. If no cmd_vel arrives for cmd_timeout seconds
everything is commanded back to zero, matching what a lost link should do.

Roll shares the horizontal (elevator) pair with depth. A roll controller
(simple_roll_fin) publishes an aileron angle; the pair then moves together
by the elevator and apart by the aileron:

    Fin_Left  = elevator + aileron
    Fin_Right = elevator - aileron

so the pair's average is still exactly the elevator (depth unaffected) and
its difference is exactly 2*aileron (roll unaffected). The one place the
two can collide is the fin stop, and how that is shared depends on whether
depth is currently *holding* or *moving*, via /depth_hold/in_band:

  * in band (|depth error| <= threshold, holding): fixed budget --
    elevator <= elevator_hold_limit (35 deg), aileron <= aileron_hold_limit
    (10 deg). 35 + 10 = 45, so neither fin can ever be clipped and a roll
    correction can never leak into pitch.
  * out of band (diving / climbing): depth is not limited at all -- the
    elevator may use the whole max_fin_angle -- and roll only gets the
    headroom left over, max_fin_angle - |elevator|, shrinking to zero at
    full elevator. Depth is the mission-critical axis; getting to depth
    beats being level on the way.

With no aileron messages (roll node not running) the aileron is 0 and the
pair behaves exactly as before.

Sign conventions (all overridable by parameter):
  * rudder_sign = -1: a fin's trailing edge lies aft (-x); a +z rotation
    swings it to starboard, which yaws the nose to starboard, i.e. a
    negative body yaw rate. So positive omega needs a negative joint angle.
  * elevator_sign = +1: a +y rotation of the left fin lifts its trailing
    edge, pitching the nose up, so positive vz -> positive angle.
  * The URDF axes of the *paired* fins are mirrored (Top +z / Bottom -z,
    Left +y / Right -y), so the second fin of each pair gets the opposite
    joint value to deflect the same physical way: fin_pair_signs handles it.
  * thruster_sign = -1 keeps the direction the old thruster_visualizer used
    (fins_control.py), i.e. the propeller mesh spins "forward" for +x.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Float64MultiArray


class CmdVelToActuators(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_actuators")

        # Geometry / limits. Defaults match ros_pralay_2_v1's URDF
        # (hull_length from the vehicle spec, fin limits +-0.9076 rad).
        self.declare_parameter("hull_length", 2.4)
        # 0.80 rad, deliberately inside the URDF's +-0.9076 joint limit: in
        # Gazebo the fins are position-servoed through gz_ros2_control, and
        # commanding exactly the hard stop makes the servo fight the joint
        # limit constraint every step, which injects impulses into the hull
        # (it tumbled during a Nav2 Spin recovery with fins at full lock).
        self.declare_parameter("max_fin_angle", 0.80)
        # Propeller command per m/s of commanded surge. Gazebo: rad/s on the
        # T1 joint (50 rad/s per m/s -> ~100 rad/s / 950 rpm at 2 m/s).
        # Real AUV: the hardware interface wants a -1..1 duty, so use
        # 1/max_speed (= 0.485 for 2.06 m/s) there.
        self.declare_parameter("thruster_scale", 50.0)
        self.declare_parameter("max_thruster_cmd", 300.0)
        self.declare_parameter("rudder_sign", -1.0)
        self.declare_parameter("elevator_sign", 1.0)
        self.declare_parameter("thruster_sign", -1.0)
        # Per-fin multipliers in controller order [Top, Left, Right, Bottom].
        self.declare_parameter("fin_pair_signs", [1.0, 1.0, -1.0, -1.0])
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("cmd_timeout", 1.0)
        # Elevator override from the depth_hold node (rad). While messages
        # keep arriving within elevator_override_timeout the horizontal fin
        # pair follows depth_hold instead of cmd_vel.linear.z; the rudder
        # pair is unaffected, so Nav2/teleop steering and depth hold coexist.
        self.declare_parameter("elevator_topic", "/depth_hold/elevator")
        self.declare_parameter("elevator_override_timeout", 0.5)
        # Roll sharing the elevator pair (see module docstring). The hold
        # limits only apply while /depth_hold/in_band is True; they sum to
        # 45 deg so nothing can clip while holding depth.
        self.declare_parameter("aileron_topic", "/roll_hold/aileron")
        self.declare_parameter("in_band_topic", "/depth_hold/in_band")
        self.declare_parameter("elevator_hold_limit", math.radians(35.0))
        self.declare_parameter("aileron_hold_limit", math.radians(10.0))
        # Out of band (diving/climbing) the elevator may use up to this, the
        # full 45 deg, and roll only gets 45 - |elevator| of headroom.
        self.declare_parameter("elevator_full_limit", math.radians(45.0))
        self.declare_parameter("fin_commands_topic", "/fin_controller/commands")
        self.declare_parameter("thruster_commands_topic", "/thruster_controller/commands")

        self.hull_length = self.get_parameter("hull_length").value
        self.max_fin_angle = self.get_parameter("max_fin_angle").value
        self.thruster_scale = self.get_parameter("thruster_scale").value
        self.max_thruster_cmd = self.get_parameter("max_thruster_cmd").value
        self.rudder_sign = self.get_parameter("rudder_sign").value
        self.elevator_sign = self.get_parameter("elevator_sign").value
        self.thruster_sign = self.get_parameter("thruster_sign").value
        self.fin_pair_signs = list(self.get_parameter("fin_pair_signs").value)
        self.cmd_timeout = self.get_parameter("cmd_timeout").value
        if len(self.fin_pair_signs) != 4:
            raise ValueError("fin_pair_signs must have 4 entries [Top, Left, Right, Bottom]")

        self.fin_pub = self.create_publisher(
            Float64MultiArray, self.get_parameter("fin_commands_topic").value, 10
        )
        self.thruster_pub = self.create_publisher(
            Float64MultiArray, self.get_parameter("thruster_commands_topic").value, 10
        )
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.elevator_override = None
        self.elevator_override_time = None
        self.elevator_override_timeout = self.get_parameter("elevator_override_timeout").value
        self.create_subscription(
            Float64, self.get_parameter("elevator_topic").value, self.on_elevator, 10
        )
        self.aileron = 0.0
        self.aileron_time = None
        self.depth_in_band = False
        self.elevator_hold_limit = self.get_parameter("elevator_hold_limit").value
        self.aileron_hold_limit = self.get_parameter("aileron_hold_limit").value
        self.elevator_full_limit = self.get_parameter("elevator_full_limit").value
        self.create_subscription(
            Float64, self.get_parameter("aileron_topic").value, self.on_aileron, 10
        )
        self.create_subscription(
            Bool, self.get_parameter("in_band_topic").value, self.on_in_band, 10
        )

        self.last_cmd = Twist()
        self.last_cmd_time = None
        period = 1.0 / self.get_parameter("publish_rate").value
        self.create_timer(period, self.publish_commands)

        self.get_logger().info(
            "cmd_vel -> fin/thruster commands: L=%.2f m, max fin %.3f rad, "
            "thruster_scale %.2f" % (self.hull_length, self.max_fin_angle, self.thruster_scale)
        )

    def on_elevator(self, msg: Float64):
        self.elevator_override = float(msg.data)
        self.elevator_override_time = self.get_clock().now()

    def elevator_from_depth_hold(self):
        """Return the depth_hold elevator angle if fresh, else None."""
        if self.elevator_override_time is None:
            return None
        age = (self.get_clock().now() - self.elevator_override_time).nanoseconds * 1e-9
        return self.elevator_override if age <= self.elevator_override_timeout else None

    def on_aileron(self, msg: Float64):
        self.aileron = float(msg.data)
        self.aileron_time = self.get_clock().now()

    def on_in_band(self, msg: Bool):
        self.depth_in_band = bool(msg.data)

    def fresh_aileron(self):
        """Roll controller's aileron angle if fresh, else 0 (roll node not running)."""
        if self.aileron_time is None:
            return 0.0
        age = (self.get_clock().now() - self.aileron_time).nanoseconds * 1e-9
        return self.aileron if age <= self.elevator_override_timeout else 0.0

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

    def on_cmd_vel(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()
        self.publish_commands()

    def compute(self, cmd: Twist):
        vx = cmd.linear.x
        vz = cmd.linear.z
        wz = cmd.angular.z

        # Rudder: bicycle model. At vx == 0 the hull can't turn anyway
        # (ThrustControl zeros the yaw torque), but the fins still swing to
        # full lock in the requested direction, like a parked car.
        if abs(vx) > 1e-3:
            rudder = math.atan(self.hull_length * wz / abs(vx))
        elif abs(wz) > 1e-6:
            rudder = math.copysign(self.max_fin_angle, wz)
        else:
            rudder = 0.0
        rudder = self.rudder_sign * self._clamp(rudder, self.max_fin_angle)

        # Elevator: from depth_hold when it is running, else the commanded
        # flight-path angle from cmd_vel.linear.z.
        override = self.elevator_from_depth_hold()
        if override is not None:
            elevator = self._clamp(override, self.max_fin_angle)
        else:
            if abs(vz) > 1e-6:
                elevator = math.atan2(vz, abs(vx))
            else:
                elevator = 0.0
            elevator = self.elevator_sign * self._clamp(elevator, self.max_fin_angle)

        # Roll rides on the same pair: together = elevator, apart = aileron.
        elevator, aileron = self.share_elevator_pair(elevator, self.fresh_aileron())

        s = self.fin_pair_signs
        fins = [
            s[0] * rudder,               # Fin_Top
            s[1] * (elevator + aileron), # Fin_Left
            s[2] * (elevator - aileron), # Fin_Right
            s[3] * rudder,               # Fin_Bottom
        ]

        thruster = self.thruster_sign * self._clamp(
            vx * self.thruster_scale, self.max_thruster_cmd
        )
        return fins, thruster

    def publish_commands(self):
        cmd = self.last_cmd
        if self.last_cmd_time is not None:
            age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
            if age > self.cmd_timeout:
                cmd = Twist()
        fins, thruster = self.compute(cmd)

        fin_msg = Float64MultiArray()
        fin_msg.data = fins
        self.fin_pub.publish(fin_msg)

        thr_msg = Float64MultiArray()
        thr_msg.data = [thruster]
        self.thruster_pub.publish(thr_msg)

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
