#include "auv_current_plugin/thrust_control.hpp"

#include <algorithm>

#include <gz/plugin/Register.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

namespace auv_current_plugin
{

void ThrustControl::Configure(
  const gz::sim::Entity & entity,
  const std::shared_ptr<const sdf::Element> & sdf,
  gz::sim::EntityComponentManager & ecm,
  gz::sim::EventManager &)
{
  model_ = gz::sim::Model(entity);
  if (!model_.Valid(ecm)) {
    return;
  }

  std::string link_name = "base_link";
  if (sdf->HasElement("link_name")) {
    link_name = sdf->Get<std::string>("link_name");
  }
  link_ = gz::sim::Link(model_.LinkByName(ecm, link_name));
  link_.EnableVelocityChecks(ecm, true);

  if (sdf->HasElement("linear_gain")) {
    linear_gain_ = sdf->Get<double>("linear_gain");
  }
  if (sdf->HasElement("angular_gain")) {
    angular_gain_ = sdf->Get<double>("angular_gain");
  }
  if (sdf->HasElement("max_force")) {
    max_force_ = sdf->Get<double>("max_force");
  }
  if (sdf->HasElement("max_torque")) {
    max_torque_ = sdf->Get<double>("max_torque");
  }

  std::string topic = "/cmd_vel";
  if (sdf->HasElement("topic")) {
    topic = sdf->Get<std::string>("topic");
  }
  node_.Subscribe(topic, &ThrustControl::OnCmdVel, this);
}

void ThrustControl::OnCmdVel(const gz::msgs::Twist & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  cmd_linear_x_ = msg.linear().x();
  cmd_angular_z_ = msg.angular().z();
}

void ThrustControl::PreUpdate(
  const gz::sim::UpdateInfo & info,
  gz::sim::EntityComponentManager & ecm)
{
  if (info.paused || !link_.Valid(ecm)) {
    return;
  }

  double cmd_x;
  double cmd_yaw;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd_x = cmd_linear_x_;
    cmd_yaw = cmd_angular_z_;
  }

  const auto pose_opt = link_.WorldPose(ecm);
  const auto lin_vel_opt = link_.WorldLinearVelocity(ecm);
  const auto ang_vel_opt = link_.WorldAngularVelocity(ecm);
  if (!pose_opt || !lin_vel_opt || !ang_vel_opt) {
    return;
  }

  // Desired world-frame velocity, rotated from the body-frame command.
  const gz::math::Vector3d desired_world_vel =
    pose_opt->Rot() * gz::math::Vector3d(cmd_x, 0.0, 0.0);

  const gz::math::Vector3d lin_error = desired_world_vel - *lin_vel_opt;
  gz::math::Vector3d force(
    lin_error.X() * linear_gain_,
    lin_error.Y() * linear_gain_,
    0.0);  // horizontal thrust only, matches the fixed-depth simplification
  if (force.Length() > max_force_) {
    force = force.Normalized() * max_force_;
  }

  const double ang_error = cmd_yaw - ang_vel_opt->Z();
  double torque_z = ang_error * angular_gain_;
  torque_z = std::clamp(torque_z, -max_torque_, max_torque_);
  const gz::math::Vector3d torque(0.0, 0.0, torque_z);

  link_.AddWorldWrench(ecm, force, torque);
}

}  // namespace auv_current_plugin

GZ_ADD_PLUGIN(
  auv_current_plugin::ThrustControl,
  gz::sim::System,
  auv_current_plugin::ThrustControl::ISystemConfigure,
  auv_current_plugin::ThrustControl::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(auv_current_plugin::ThrustControl, "auv_current_plugin::ThrustControl")
