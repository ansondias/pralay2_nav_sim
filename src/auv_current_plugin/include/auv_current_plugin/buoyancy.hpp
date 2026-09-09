#ifndef AUV_CURRENT_PLUGIN__BUOYANCY_HPP_
#define AUV_CURRENT_PLUGIN__BUOYANCY_HPP_

#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/math/Vector3.hh>

namespace auv_current_plugin
{

// Real hydrostatics, replacing the old "gravity disabled" fake-neutral-
// buoyancy shortcut: applies an upward buoyant force rho_water * V * g at a
// configurable centre of buoyancy (CoB). With the CoB above the vehicle's
// centre of gravity this also yields the metacentric righting moment that
// gives passive pitch/roll stability. Requires <gravity>true</gravity> on
// the link -- this plugin only supplies the *counteracting* force.
//
// Parameters (all under the <plugin> element):
//   link_name           link the force acts on (default base_link)
//   fluid_density       kg/m^3 (default 1025, seawater)
//   hull_volume         displaced volume, m^3. For neutral buoyancy this is
//                       the *whole vehicle's* mass / fluid_density, not just
//                       this link's: the link carries every other link
//                       through its joints.
//   center_of_buoyancy  "x y z", metres, in the link frame, measured from
//                       the LINK ORIGIN (not its centre of mass). This is
//                       where the force is applied. For a level, stable
//                       hull put it directly above the vehicle's composite
//                       centre of gravity. If omitted, the link's own
//                       centre of mass (from its <inertial>) plus
//                       center_of_buoyancy_offset_z is used -- correct only
//                       for single-link vehicles like auv_description's
//                       placeholder torpedo.
//   center_of_buoyancy_offset_z
//                       fallback vertical CoB-above-CoM margin, m (default
//                       0.03), only used when center_of_buoyancy is absent.
//   gravity_magnitude   m/s^2. Normally omitted: the world's own <gravity>
//                       is read from the ECM so buoyancy and weight cancel
//                       exactly. Only set it to deliberately trim the
//                       vehicle heavy/light.
//
// Why the application point matters: gz-sim's Link::AddWorldWrench applies
// a force at the link *origin*, and for a multi-link model (the real
// Pralay-2 export: hull + 4 fins + propeller, tail-heavy by ~0.13 m) the
// composite CoG is nowhere near that origin. A first version applied the
// force at the origin and the hull pitched to ~80 deg nose-up and rose
// steadily in the very first test run.
class Buoyancy
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
  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Link link_{gz::sim::kNullEntity};

  double fluid_density_{1025.0};   // kg/m^3, seawater
  double hull_volume_{0.0504};     // m^3 (placeholder 2.4m x 16cm hull)
  double gravity_magnitude_{9.8};  // m/s^2; overwritten from the world unless set

  // Centre of buoyancy in the link frame, from the link origin. Resolved in
  // Configure (explicit parameter, or link CoM + offset_z fallback).
  gz::math::Vector3d center_of_buoyancy_{0.0, 0.0, 0.03};
  double center_of_buoyancy_offset_z_{0.03};
};

}  // namespace auv_current_plugin

#endif  // AUV_CURRENT_PLUGIN__BUOYANCY_HPP_
