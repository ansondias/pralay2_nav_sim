#ifndef AUV_CURRENT_PLUGIN__THRUST_CONTROL_HPP_
#define AUV_CURRENT_PLUGIN__THRUST_CONTROL_HPP_

#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>

#include <mutex>
#include <string>

namespace auv_current_plugin
{

// Applies a body-frame surge/yaw-rate thrust force toward a commanded
// cmd_vel (bridged in as gz.msgs.Twist), as a simple proportional velocity
// controller. This is what actually moves a floating AUV with no ground
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

  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Link link_{gz::sim::kNullEntity};

  gz::transport::Node node_;

  std::mutex mutex_;
  double cmd_linear_x_{0.0};
  double cmd_angular_z_{0.0};

  // Gains are deliberately conservative relative to the hull's mass/inertia
  // and the physics step size: an explicit-Euler-integrated P controller on
  // a rigid body is only stable while gain < 2*I/dt (I = mass or moment of
  // inertia, dt = physics step). For this hull (~20 kg, yaw inertia ~0.22
  // kg*m^2) at dt=0.004s that bound is ~10000 N/(m/s) linear and ~110
  // N*m/(rad/s) angular -- a first version of this plugin used an angular
  // gain of 200, past that bound, which diverged into a runaway spin and
  // crashed the physics solver on the very first rotation command.
  double linear_gain_{40.0};    // N per (m/s) of velocity error
  double angular_gain_{0.5};    // N*m per (rad/s) of velocity error

  // Hard force/torque caps as a second line of defense, independent of
  // gain tuning (e.g. against a sudden large cmd_vel jump).
  double max_force_{200.0};   // N
  double max_torque_{20.0};   // N*m
};

}  // namespace auv_current_plugin

#endif  // AUV_CURRENT_PLUGIN__THRUST_CONTROL_HPP_
