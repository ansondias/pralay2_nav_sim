#include "auv_current_plugin/ocean_current.hpp"

#include <gz/plugin/Register.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/components/Name.hh>

#include <cmath>

namespace auv_current_plugin
{

void OceanCurrent::Configure(
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

  if (sdf->HasElement("speed")) {
    speed_mps_ = sdf->Get<double>("speed");
  }
  if (sdf->HasElement("direction_north_deg")) {
    direction_north_deg_ = sdf->Get<double>("direction_north_deg");
  }
  if (sdf->HasElement("drag_coefficient")) {
    drag_coefficient_ = sdf->Get<double>("drag_coefficient");
  }
  if (sdf->HasElement("fluid_density")) {
    fluid_density_ = sdf->Get<double>("fluid_density");
  }
  if (sdf->HasElement("cross_section_area")) {
    cross_section_area_ = sdf->Get<double>("cross_section_area");
  }

  std::string topic = "/model/" + model_.Name(ecm) + "/ocean_current";
  if (sdf->HasElement("topic")) {
    topic = sdf->Get<std::string>("topic");
  }
  node_.Subscribe(topic, &OceanCurrent::OnCurrentMsg, this);
}

void OceanCurrent::OnCurrentMsg(const gz::msgs::Vector3d & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // x: current speed (m/s), y: direction current flows toward, as a
  // compass bearing in degrees clockwise from north (0 = to the north,
  // 90 = to the east) -- matches water_current_direction_north in
  // auv_planning's nav2 params. z is unused.
  speed_mps_ = msg.x();
  direction_north_deg_ = msg.y();
}

void OceanCurrent::PreUpdate(
  const gz::sim::UpdateInfo & info,
  gz::sim::EntityComponentManager & ecm)
{
  if (info.paused || !link_.Valid(ecm)) {
    return;
  }

  double speed;
  double direction_deg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    speed = speed_mps_;
    direction_deg = direction_north_deg_;
  }

  // World frame is ENU: +X east, +Y north. Bearing is clockwise from north.
  const double theta = direction_deg * M_PI / 180.0;
  const gz::math::Vector3d current_velocity(
    speed * std::sin(theta),
    speed * std::cos(theta),
    0.0);

  const auto link_velocity_opt = link_.WorldLinearVelocity(ecm);
  if (!link_velocity_opt) {
    return;
  }

  gz::math::Vector3d relative_velocity = current_velocity - *link_velocity_opt;
  relative_velocity.Z(0.0);  // horizontal current only

  const double rel_speed = relative_velocity.Length();
  if (rel_speed < 1e-6) {
    return;
  }

  // Quadratic drag: F = 0.5 * rho * Cd * A * |v_rel| * v_rel
  const double magnitude =
    0.5 * fluid_density_ * drag_coefficient_ * cross_section_area_ * rel_speed;
  const gz::math::Vector3d force = relative_velocity * magnitude;

  link_.AddWorldForce(ecm, force);
}

}  // namespace auv_current_plugin

GZ_ADD_PLUGIN(
  auv_current_plugin::OceanCurrent,
  gz::sim::System,
  auv_current_plugin::OceanCurrent::ISystemConfigure,
  auv_current_plugin::OceanCurrent::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(auv_current_plugin::OceanCurrent, "auv_current_plugin::OceanCurrent")
