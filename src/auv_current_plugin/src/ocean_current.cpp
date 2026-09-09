#include "auv_current_plugin/ocean_current.hpp"

#include <gz/plugin/Register.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Name.hh>

#include <chrono>
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
  if (const auto * inertial = ecm.Component<gz::sim::components::Inertial>(link_.Entity())) {
    force_point_ = inertial->Data().Pose().Pos();
  }
  if (sdf->HasElement("center_of_mass")) {
    force_point_ = sdf->Get<gz::math::Vector3d>("center_of_mass");
  }

  if (sdf->HasElement("speed")) {
    speed_mps_ = sdf->Get<double>("speed");
  }
  if (sdf->HasElement("direction_north_deg")) {
    direction_north_deg_ = sdf->Get<double>("direction_north_deg");
  }
  if (sdf->HasElement("axial_drag_coefficient")) {
    axial_drag_coefficient_ = sdf->Get<double>("axial_drag_coefficient");
  }
  if (sdf->HasElement("lateral_drag_coefficient")) {
    lateral_drag_coefficient_ = sdf->Get<double>("lateral_drag_coefficient");
  }
  if (sdf->HasElement("axial_area")) {
    axial_area_ = sdf->Get<double>("axial_area");
  }
  if (sdf->HasElement("lateral_area")) {
    lateral_area_ = sdf->Get<double>("lateral_area");
  }
  if (sdf->HasElement("fluid_density")) {
    fluid_density_ = sdf->Get<double>("fluid_density");
  }
  if (sdf->HasElement("surface_z")) {
    surface_z_ = sdf->Get<double>("surface_z");
  }
  if (sdf->HasElement("current_depth_decay_scale")) {
    current_depth_decay_scale_ = sdf->Get<double>("current_depth_decay_scale");
  }
  if (sdf->HasElement("enable_monsoon_cycle")) {
    enable_monsoon_cycle_ = sdf->Get<bool>("enable_monsoon_cycle");
  }
  if (sdf->HasElement("monsoon_period_sec")) {
    monsoon_period_sec_ = sdf->Get<double>("monsoon_period_sec");
  }
  if (sdf->HasElement("sw_monsoon_speed")) {
    sw_monsoon_speed_ = sdf->Get<double>("sw_monsoon_speed");
  }
  if (sdf->HasElement("sw_monsoon_direction_deg")) {
    sw_monsoon_direction_deg_ = sdf->Get<double>("sw_monsoon_direction_deg");
  }
  if (sdf->HasElement("ne_monsoon_speed")) {
    ne_monsoon_speed_ = sdf->Get<double>("ne_monsoon_speed");
  }
  if (sdf->HasElement("ne_monsoon_direction_deg")) {
    ne_monsoon_direction_deg_ = sdf->Get<double>("ne_monsoon_direction_deg");
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
  // auv_planning's nav2 params. z is unused. Only takes effect when
  // enable_monsoon_cycle is false -- see class comment.
  speed_mps_ = msg.x();
  direction_north_deg_ = msg.y();
}

gz::math::Vector3d OceanCurrent::SurfaceCurrentVelocity(double sim_time_sec) const
{
  double speed;
  double direction_deg;

  if (enable_monsoon_cycle_) {
    // Raised-cosine phase: smoothly eases between the two monsoon vectors
    // rather than snapping, still fully transitioning once per period.
    const double phase = std::fmod(sim_time_sec, monsoon_period_sec_) / monsoon_period_sec_;
    const double blend = 0.5 * (1.0 - std::cos(2.0 * M_PI * phase));

    const double sw_theta = sw_monsoon_direction_deg_ * M_PI / 180.0;
    const double ne_theta = ne_monsoon_direction_deg_ * M_PI / 180.0;
    const gz::math::Vector2d sw_vec(
      sw_monsoon_speed_ * std::sin(sw_theta), sw_monsoon_speed_ * std::cos(sw_theta));
    const gz::math::Vector2d ne_vec(
      ne_monsoon_speed_ * std::sin(ne_theta), ne_monsoon_speed_ * std::cos(ne_theta));

    // Blend the vector components directly (not the angles) so there's no
    // wraparound discontinuity crossing 0/360 deg.
    const gz::math::Vector2d blended = sw_vec * (1.0 - blend) + ne_vec * blend;
    return gz::math::Vector3d(blended.X(), blended.Y(), 0.0);
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    speed = speed_mps_;
    direction_deg = direction_north_deg_;
  }
  const double theta = direction_deg * M_PI / 180.0;
  return gz::math::Vector3d(speed * std::sin(theta), speed * std::cos(theta), 0.0);
}

void OceanCurrent::PreUpdate(
  const gz::sim::UpdateInfo & info,
  gz::sim::EntityComponentManager & ecm)
{
  if (info.paused || !link_.Valid(ecm)) {
    return;
  }

  const auto pose_opt = link_.WorldPose(ecm);
  const auto link_velocity_opt = link_.WorldLinearVelocity(ecm);
  if (!pose_opt || !link_velocity_opt) {
    return;
  }

  const double sim_time_sec =
    std::chrono::duration<double>(info.simTime).count();
  gz::math::Vector3d current_velocity = SurfaceCurrentVelocity(sim_time_sec);

  // Exponential depth decay below the assumed free surface (Ekman-layer
  // style): strongest at/above the surface, weaker deeper down. Current
  // direction itself is not varied with depth (see class comment).
  if (current_depth_decay_scale_ > 0.0) {
    const double depth_below_surface = surface_z_ - pose_opt->Pos().Z();
    if (depth_below_surface > 0.0) {
      const double attenuation = std::exp(-depth_below_surface / current_depth_decay_scale_);
      current_velocity *= attenuation;
    }
  }

  const gz::math::Vector3d relative_velocity_world = current_velocity - *link_velocity_opt;
  const double rel_speed_world = relative_velocity_world.Length();
  if (rel_speed_world < 1e-6) {
    return;
  }

  // Anisotropic quadratic drag: decompose the relative flow into the body
  // frame so axial (surge) and lateral (sway/heave) flow get independent
  // drag_coefficient/area, then rotate the resulting force back to world
  // frame. This also gives real heave (vertical) damping, since Z is a
  // lateral axis here -- needed now that Buoyancy gives the hull real
  // vertical dynamics.
  const gz::math::Vector3d v_rel_body = pose_opt->Rot().Inverse() * relative_velocity_world;

  const double axial_k = 0.5 * fluid_density_ * axial_drag_coefficient_ * axial_area_;
  const double lateral_k = 0.5 * fluid_density_ * lateral_drag_coefficient_ * lateral_area_;

  const gz::math::Vector3d force_body(
    axial_k * std::abs(v_rel_body.X()) * v_rel_body.X(),
    lateral_k * std::abs(v_rel_body.Y()) * v_rel_body.Y(),
    lateral_k * std::abs(v_rel_body.Z()) * v_rel_body.Z());

  const gz::math::Vector3d force_world = pose_opt->Rot() * force_body;

  // At the configured centre of mass (offset from the link origin), so the
  // drag is a pure force and not also a pitch/yaw moment.
  link_.AddWorldWrench(ecm, force_world, gz::math::Vector3d::Zero, force_point_);
}

}  // namespace auv_current_plugin

GZ_ADD_PLUGIN(
  auv_current_plugin::OceanCurrent,
  gz::sim::System,
  auv_current_plugin::OceanCurrent::ISystemConfigure,
  auv_current_plugin::OceanCurrent::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(auv_current_plugin::OceanCurrent, "auv_current_plugin::OceanCurrent")
