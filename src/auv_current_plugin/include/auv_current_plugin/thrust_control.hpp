#ifndef AUV_CURRENT_PLUGIN__THRUST_CONTROL_HPP_
#define AUV_CURRENT_PLUGIN__THRUST_CONTROL_HPP_

#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/twist.pb.h>

#include <chrono>
#include <mutex>
#include <optional>
#include <string>

namespace auv_current_plugin
{

// Applies a body-frame surge force and a yaw torque toward a commanded
// cmd_vel (bridged in as gz.msgs.Twist), as a simple proportional velocity
// controller. Thrust is SURGE ONLY (along body +x, like a propeller): the
// hull cannot push sideways, so a cross-current genuinely sets it drifting
// and the path follower has to steer against it. An earlier version also
// applied a world-frame lateral force to cancel any sideways velocity;
// that both hid the current entirely and, because gz-sim applies wrenches
// at the link origin while the real hull's centre of mass is ~7 cm aft of
// it, produced a yaw moment the yaw loop could not cancel -- the vehicle
// spun on the spot under Nav2. All forces are now applied at the centre
// of mass (offset read from the link's <inertial>). With no live command
// (see cmd_timeout_) thrust is off entirely and the hull drifts. This is what actually moves a floating AUV with no ground
// contact -- gz-sim's DiffDrive won't (it needs wheel/ground friction) and
// its VelocityControl system directly overwrites the model's velocity
// every tick, which would silently cancel OceanCurrent's drag force.
// Applying a *force* instead means thrust and current genuinely combine
// through physics integration: the vehicle has to fight the current, and
// drifts with it when idle.
class ThrustControl
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager & eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override;

private:
  void OnCmdVel(const gz::msgs::Twist & msg);
  void OnDepthSetpoint(const gz::msgs::Double & msg);

  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Link link_{gz::sim::kNullEntity};

  gz::transport::Node node_;

  std::mutex mutex_;
  double cmd_linear_x_{0.0};
  double cmd_linear_z_{0.0};
  double cmd_angular_z_{0.0};
  // Wall-clock time of the last cmd_vel. After cmd_timeout_ seconds
  // without one the command is treated as zero (thrust off, depth target
  // frozen) -- same policy as auv_control's cmd_vel_to_actuators, and what
  // a lost link should do. Before this, a publisher that simply stopped
  // (teleop, a test `ros2 topic pub`) left the hull thrusting and, with
  // the depth hold integrating linear.z, climbing without bound. Nav2
  // publishes explicit zeros, so it is unaffected. Wall clock rather than
  // sim time because the subscriber runs on a transport thread.
  std::chrono::steady_clock::time_point last_cmd_time_{};
  double cmd_timeout_{1.0};  // s; <= 0 disables the timeout

  // Depth hold (a minimal depth autopilot). With auv_current_plugin::Buoyancy
  // the hull has real heave dynamics, and nothing else holds depth: any
  // sub-newton imbalance (rounded masses, current-induced lift) is an
  // unbounded drift of mm/s that ends on the seafloor or above the
  // surface. A real AUV's depth controller keeps it where it was told;
  // this does the same with a critically damped spring on world z. The
  // target starts at the spawn depth and integrates cmd_vel.linear.z
  // (m/s, positive up) so 3-D commands still mean something. Vertical
  // force is capped separately from the surge/yaw caps.
  bool depth_hold_{true};
  double depth_gain_{20.0};            // N per m of depth error
  double depth_damping_{30.0};         // N per (m/s) of heave velocity
  double max_vertical_force_{30.0};    // N
  double max_vertical_speed_{0.5};     // m/s, cap on cmd_vel.linear.z
  std::optional<double> target_z_;     // world z, set on first update
  // Absolute depth setpoints: a gz.msgs.Double on depth_setpoint_topic_
  // (default /model/<name>/depth_setpoint, bridged from ROS
  // /depth_setpoint) gives metres BELOW surface_z_, i.e. "go to 3 m depth"
  // -> target_z_ = surface_z_ - 3. surface_z_ should match OceanCurrent's.
  double surface_z_{5.0};
  std::optional<double> pending_depth_setpoint_;

  // Point (link frame, from the link origin) where every force here is
  // applied, so none of them turn into moments. Set <center_of_mass> to the
  // VEHICLE's composite centre of mass when the model has several links:
  // sdformat lumps fixed-joint children into base_link but the fins and
  // propeller on revolute joints stay separate, so the link's own CoM
  // (the default, read from its <inertial>) is ~8 cm ahead of the
  // vehicle's on the Pralay-2. Applying the 30 N depth-hold force there
  // gave a pitch moment comparable to the righting moment, and the hull
  // tumbled under Nav2.
  gz::math::Vector3d com_offset_{0.0, 0.0, 0.0};

  // Gains are deliberately conservative relative to the hull's mass/inertia
  // and the physics step size: an explicit-Euler-integrated P controller on
  // a rigid body is only stable while gain < 2*I/dt (I = mass or moment of
  // inertia, dt = physics step). hull_mass is now *derived* from hull_volume
  // at neutral-buoyancy density (auv_description/urdf/auv.urdf.xacro), not
  // an independent number: ~51.7 kg for the 2.4m x 16cm hull, yaw inertia
  // mass*hull_radius^2/2 ~0.165 kg*m^2. At dt=0.004s that bound is ~25800
  // N/(m/s) linear and ~83 N*m/(rad/s) angular -- a first version of this
  // plugin used an angular gain of 200, past the (smaller, 20kg-hull) bound
  // at the time, which diverged into a runaway spin and crashed the physics
  // solver on the very first rotation command. Re-check this bound if
  // hull_radius/hull_length (and so the derived hull_mass) change.
  double linear_gain_{40.0};    // N per (m/s) of velocity error
  double angular_gain_{0.5};    // N*m per (rad/s) of velocity error

  // Hard force/torque caps as a second line of defense, independent of
  // gain tuning (e.g. against a sudden large cmd_vel jump).
  double max_force_{200.0};   // N
  double max_torque_{20.0};   // N*m

  // Ackermann/fin-steering kinematic constraint: the vehicle is rated for
  // 4 kn (2.06 m/s) and cannot yaw faster than a bicycle-model turning
  // radius allows at the current commanded speed (omega <= v / r) -- in
  // particular it cannot rotate in place at v=0, unlike a thrust-vectored
  // AUV. Mirrors controller_server.FollowPath.min_turning_radius in
  // auv_navigation/config/nav2_params.yaml so Nav2's planned trajectories
  // and what the hull can actually do agree; enforced here too so this
  // holds regardless of what publishes cmd_vel (teleop included).
  double max_linear_speed_{2.06};    // m/s (4 kn)
  double min_turning_radius_{5.0};  // m (>= 2.4 m hull_length / tan(30 deg))
};

}  // namespace auv_current_plugin

#endif  // AUV_CURRENT_PLUGIN__THRUST_CONTROL_HPP_
