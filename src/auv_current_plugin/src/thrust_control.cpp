#include "auv_current_plugin/thrust_control.hpp"

#include <algorithm>
#include <chrono>

#include <gz/plugin/Register.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/components/Inertial.hh>

#include <iostream>

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
  if (const auto * inertial = ecm.Component<gz::sim::components::Inertial>(link_.Entity())) {
    com_offset_ = inertial->Data().Pose().Pos();
  }
  if (sdf->HasElement("center_of_mass")) {
    com_offset_ = sdf->Get<gz::math::Vector3d>("center_of_mass");
  }

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
  if (sdf->HasElement("max_linear_speed")) {
    max_linear_speed_ = sdf->Get<double>("max_linear_speed");
  }
  if (sdf->HasElement("min_turning_radius")) {
    min_turning_radius_ = sdf->Get<double>("min_turning_radius");
  }
  if (sdf->HasElement("depth_hold")) {
    depth_hold_ = sdf->Get<bool>("depth_hold");
  }
  if (sdf->HasElement("depth_gain")) {
    depth_gain_ = sdf->Get<double>("depth_gain");
  }
  if (sdf->HasElement("depth_damping")) {
    depth_damping_ = sdf->Get<double>("depth_damping");
  }
  if (sdf->HasElement("max_vertical_force")) {
    max_vertical_force_ = sdf->Get<double>("max_vertical_force");
  }
  if (sdf->HasElement("max_vertical_speed")) {
    max_vertical_speed_ = sdf->Get<double>("max_vertical_speed");
  }
  if (sdf->HasElement("cmd_timeout")) {
    cmd_timeout_ = sdf->Get<double>("cmd_timeout");
  }
  if (sdf->HasElement("surface_z")) {
    surface_z_ = sdf->Get<double>("surface_z");
  }

  std::string topic = "/cmd_vel";
  if (sdf->HasElement("topic")) {
    topic = sdf->Get<std::string>("topic");
  }
  node_.Subscribe(topic, &ThrustControl::OnCmdVel, this);

  std::string depth_topic = "/model/" + model_.Name(ecm) + "/depth_setpoint";
  if (sdf->HasElement("depth_setpoint_topic")) {
    depth_topic = sdf->Get<std::string>("depth_setpoint_topic");
  }
  node_.Subscribe(depth_topic, &ThrustControl::OnDepthSetpoint, this);

  std::cout << "[auv_current_plugin::ThrustControl] " << model_.Name(ecm) << "/" << link_name
            << ": cmd_vel on " << topic << ", depth setpoints (m below z=" << surface_z_
            << ") on " << depth_topic << ", forces at CoM offset " << com_offset_ << "\n";
}

void ThrustControl::OnDepthSetpoint(const gz::msgs::Double & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_depth_setpoint_ = msg.data();
}

void ThrustControl::OnCmdVel(const gz::msgs::Twist & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  cmd_linear_x_ = msg.linear().x();
  cmd_linear_z_ = msg.linear().z();
  cmd_angular_z_ = msg.angular().z();
  last_cmd_time_ = std::chrono::steady_clock::now();
}

void ThrustControl::PreUpdate(
  const gz::sim::UpdateInfo & info,
  gz::sim::EntityComponentManager & ecm)
{
  if (info.paused || !link_.Valid(ecm)) {
    return;
  }

  double cmd_x;
  double cmd_z;
  double cmd_yaw;
  bool cmd_active = true;
  std::optional<double> depth_setpoint;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd_x = cmd_linear_x_;
    cmd_z = cmd_linear_z_;
    cmd_yaw = cmd_angular_z_;
    if (cmd_timeout_ > 0.0) {
      const auto age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_cmd_time_).count();
      if (last_cmd_time_ == std::chrono::steady_clock::time_point{} || age > cmd_timeout_) {
        cmd_x = 0.0;
        cmd_z = 0.0;
        cmd_yaw = 0.0;
        cmd_active = false;
      }
    }
    depth_setpoint = pending_depth_setpoint_;
    pending_depth_setpoint_.reset();
  }

  // Ackermann/fin-steering kinematics: cap forward speed at the vehicle's
  // rated top speed, then cap yaw rate at what a min_turning_radius_ turn
  // allows at that speed (omega <= v / r). At cmd_x == 0 this forces
  // cmd_yaw to 0 too -- the hull cannot rotate in place, only turn while
  // moving, same as a car.
  cmd_x = std::clamp(cmd_x, -max_linear_speed_, max_linear_speed_);
  const double max_yaw_rate = std::abs(cmd_x) / min_turning_radius_;
  cmd_yaw = std::clamp(cmd_yaw, -max_yaw_rate, max_yaw_rate);

  const auto pose_opt = link_.WorldPose(ecm);
  const auto lin_vel_opt = link_.WorldLinearVelocity(ecm);
  const auto ang_vel_opt = link_.WorldAngularVelocity(ecm);
  if (!pose_opt || !lin_vel_opt || !ang_vel_opt) {
    return;
  }

  // Surge-only thrust along body +x: P control on the body-x component of
  // the world velocity. No sideways force -- the current can push the hull
  // off track and the path follower has to steer against it. With no live
  // command the propeller is simply off and the hull drifts.
  const gz::math::Vector3d body_x_world = pose_opt->Rot() * gz::math::Vector3d::UnitX;
  gz::math::Vector3d force = gz::math::Vector3d::Zero;
  if (cmd_active) {
    const double surge_vel = lin_vel_opt->Dot(body_x_world);
    const double surge_force =
      std::clamp((cmd_x - surge_vel) * linear_gain_, -max_force_, max_force_);
    force = body_x_world * surge_force;
  }

  // Depth hold: spring-damper on world z toward target_z_, which starts at
  // the depth the hull first had, follows cmd_vel.linear.z (m/s, +up), and
  // jumps to any absolute setpoint received on the depth topic.
  if (depth_hold_) {
    const double z = pose_opt->Pos().Z();
    if (!target_z_) {
      target_z_ = z;
    }
    if (depth_setpoint) {
      *target_z_ = surface_z_ - *depth_setpoint;
    }
    const double dt = std::chrono::duration<double>(info.dt).count();
    cmd_z = std::clamp(cmd_z, -max_vertical_speed_, max_vertical_speed_);
    *target_z_ += cmd_z * dt;
    const double fz = depth_gain_ * (*target_z_ - z) - depth_damping_ * lin_vel_opt->Z();
    force.Z() += std::clamp(fz, -max_vertical_force_, max_vertical_force_);
  }

  const double ang_error = cmd_yaw - ang_vel_opt->Z();
  double torque_z = ang_error * angular_gain_;
  torque_z = std::clamp(torque_z, -max_torque_, max_torque_);
  const gz::math::Vector3d torque(0.0, 0.0, torque_z);

  // At the centre of mass, so neither the surge nor the depth force turns
  // into a pitch/yaw moment (gz-sim's 3-arg overload applies at the link
  // origin, which for this hull is ahead of the CoM).
  link_.AddWorldWrench(ecm, force, torque, com_offset_);
}

}  // namespace auv_current_plugin

GZ_ADD_PLUGIN(
  auv_current_plugin::ThrustControl,
  gz::sim::System,
  auv_current_plugin::ThrustControl::ISystemConfigure,
  auv_current_plugin::ThrustControl::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(auv_current_plugin::ThrustControl, "auv_current_plugin::ThrustControl")
