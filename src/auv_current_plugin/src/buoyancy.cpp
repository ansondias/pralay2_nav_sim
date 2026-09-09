#include "auv_current_plugin/buoyancy.hpp"

#include <gz/plugin/Register.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/Inertial.hh>

#include <iostream>

namespace auv_current_plugin
{

void Buoyancy::Configure(
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
  if (!link_.Valid(ecm)) {
    std::cerr << "[auv_current_plugin::Buoyancy] link [" << link_name
              << "] not found in model [" << model_.Name(ecm) << "]; disabled\n";
    return;
  }

  if (sdf->HasElement("fluid_density")) {
    fluid_density_ = sdf->Get<double>("fluid_density");
  }
  if (sdf->HasElement("hull_volume")) {
    hull_volume_ = sdf->Get<double>("hull_volume");
  }
  if (sdf->HasElement("center_of_buoyancy_offset_z")) {
    center_of_buoyancy_offset_z_ = sdf->Get<double>("center_of_buoyancy_offset_z");
  }

  // Gravity: the world's value unless explicitly overridden, so the buoyant
  // force cancels the weight exactly (SDF's default is 9.8, not 9.81).
  if (sdf->HasElement("gravity_magnitude")) {
    gravity_magnitude_ = sdf->Get<double>("gravity_magnitude");
  } else {
    const auto world = gz::sim::worldEntity(entity, ecm);
    const auto * gravity = ecm.Component<gz::sim::components::Gravity>(world);
    if (gravity) {
      gravity_magnitude_ = gravity->Data().Length();
    }
  }

  // Centre of buoyancy (link frame, from the link origin).
  if (sdf->HasElement("center_of_buoyancy")) {
    center_of_buoyancy_ = sdf->Get<gz::math::Vector3d>("center_of_buoyancy");
  } else {
    gz::math::Vector3d com(0.0, 0.0, 0.0);
    const auto * inertial = ecm.Component<gz::sim::components::Inertial>(link_.Entity());
    if (inertial) {
      com = inertial->Data().Pose().Pos();
    }
    center_of_buoyancy_ = com + gz::math::Vector3d(0.0, 0.0, center_of_buoyancy_offset_z_);
  }

  std::cout << "[auv_current_plugin::Buoyancy] " << model_.Name(ecm) << "/" << link_name
            << ": F = " << fluid_density_ * hull_volume_ * gravity_magnitude_
            << " N (rho " << fluid_density_ << ", V " << hull_volume_
            << ", g " << gravity_magnitude_ << ") at CoB " << center_of_buoyancy_
            << " (link frame)\n";
}

void Buoyancy::PreUpdate(
  const gz::sim::UpdateInfo & info,
  gz::sim::EntityComponentManager & ecm)
{
  if (info.paused || !link_.Valid(ecm)) {
    return;
  }

  const double buoyant_force_magnitude = fluid_density_ * hull_volume_ * gravity_magnitude_;
  const gz::math::Vector3d force(0.0, 0.0, buoyant_force_magnitude);

  // Apply the vertical force at the CoB (link-frame offset from the link
  // origin; gz-sim rotates it into world frame and works out the resulting
  // moment about the link's own origin/CoM). With the CoB above the
  // composite CoG this is the metacentric righting effect: zero torque when
  // level, a torque opposing the tilt otherwise.
  link_.AddWorldWrench(ecm, force, gz::math::Vector3d::Zero, center_of_buoyancy_);
}

}  // namespace auv_current_plugin

GZ_ADD_PLUGIN(
  auv_current_plugin::Buoyancy,
  gz::sim::System,
  auv_current_plugin::Buoyancy::ISystemConfigure,
  auv_current_plugin::Buoyancy::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(auv_current_plugin::Buoyancy, "auv_current_plugin::Buoyancy")
