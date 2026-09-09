#ifndef AUV_CURRENT_PLUGIN__OCEAN_CURRENT_HPP_
#define AUV_CURRENT_PLUGIN__OCEAN_CURRENT_HPP_

#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/vector3d.pb.h>

#include <mutex>
#include <string>

namespace auv_current_plugin
{

// Applies an anisotropic quadratic-drag wrench that pulls a link toward a
// configurable ocean current velocity, with three layers of environmental
// realism on top of a flat scalar current:
//
//  1. Depth-dependent strength: the current decays exponentially below
//     surface_z (an Ekman-layer-style profile), via current_depth_decay_scale.
//  2. A monsoon cycle: when enable_monsoon_cycle is true, the surface
//     current blends between an sw_monsoon_* and ne_monsoon_* vector over
//     monsoon_period_sec, modelling the Indian Ocean's seasonal reversal
//     (compressed to a period short enough to observe in one sim run --
//     NOT real seasonal timescale). When false, the static speed/direction
//     below (settable live over Gazebo Transport) is used instead, as before.
//  3. Anisotropic drag: axial (surge, body +X) vs lateral (sway/heave,
//     body Y/Z) get independent drag_coefficient/area, since a long thin
//     hull resists broadside flow far more than nose-on flow. This also
//     supplies real heave (vertical) damping, which the old isotropic,
//     horizontal-only model didn't have -- needed now that Buoyancy gives
//     the hull real vertical dynamics instead of gravity being disabled.
//
// Current speed/direction (whichever source is active) can still be
// mirrored into the auv_planning nav2 params
// (water_current_speed / water_current_direction_north) for a scenario,
// though with depth decay and the monsoon cycle active that's only ever an
// approximation of what the sim is actually applying at a given moment.
class OceanCurrent
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
  void OnCurrentMsg(const gz::msgs::Vector3d & msg);

  // World-frame (ENU) surface current vector at the current sim time,
  // before depth decay: the monsoon blend if enabled, else the static
  // speed_mps_/direction_north_deg_ pair.
  gz::math::Vector3d SurfaceCurrentVelocity(double sim_time_sec) const;

  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Link link_{gz::sim::kNullEntity};

  // Where the drag force is applied (link frame, from the link origin):
  // <center_of_mass>, else the link's own <inertial> CoM. For a multi-link
  // vehicle give the composite CoM, see thrust_control.hpp -- a lateral
  // drag force applied ahead of it is a yaw moment the (deliberately
  // simple) yaw loop then has to fight.
  gz::math::Vector3d force_point_{0.0, 0.0, 0.0};

  gz::transport::Node node_;

  mutable std::mutex mutex_;
  double speed_mps_{0.0};
  double direction_north_deg_{0.0};

  // Anisotropic hull drag: axial = along the body's long axis (surge,
  // nose/tail-on flow), lateral = broadside (sway/heave, sides/top/bottom).
  // Defaults match the 2.4m x 16cm hull in auv_description/urdf/auv.urdf.xacro:
  // axial_area = frontal circle = pi*0.08^2 ~= 0.0201 m^2; lateral_area =
  // length x diameter rectangle = 2.4*0.16 = 0.384 m^2. Keep in sync with
  // hull_frontal_area/hull_lateral_area in that file if the hull is resized.
  double axial_drag_coefficient_{0.3};    // streamlined nose-on flow
  double lateral_drag_coefficient_{1.2};  // bluff-body broadside flow
  double axial_area_{0.0201};             // m^2
  double lateral_area_{0.384};            // m^2
  double fluid_density_{1025.0};          // kg/m^3, nominal seawater

  // Depth-dependent current strength: exponential decay below surface_z.
  // No real bathymetry-referenced "sea surface" exists elsewhere in this
  // sandbox (see auv_gazebo/worlds/ocean_arena.sdf), so surface_z is an
  // assumed value, not derived from anything; current_depth_decay_scale
  // <= 0 disables depth decay entirely (current uniform with depth, the
  // old behavior).
  double surface_z_{5.0};                    // m, assumed free-surface height
  double current_depth_decay_scale_{10.0};   // m, e-folding depth scale

  // Monsoon cycle: blends sw_monsoon_*/ne_monsoon_* current vectors over
  // monsoon_period_sec using a smooth (raised-cosine) phase, vector-blended
  // (not angle-interpolated) to avoid any compass-bearing wraparound bugs.
  // Direction is assumed depth-invariant; only magnitude decays with depth
  // (see above) -- a real Ekman spiral also veers direction with depth,
  // which this does not model.
  bool enable_monsoon_cycle_{true};
  double monsoon_period_sec_{120.0};       // compressed cycle, not 1 real year
  double sw_monsoon_speed_{0.6};           // m/s, SW monsoon (Jun-Sep): stronger
  double sw_monsoon_direction_deg_{45.0};  // toward NE, e.g. Somali-coast-like flow
  double ne_monsoon_speed_{0.25};          // m/s, NE monsoon (winter): calmer
  double ne_monsoon_direction_deg_{225.0}; // toward SW, reversed flow
};

}  // namespace auv_current_plugin

#endif  // AUV_CURRENT_PLUGIN__OCEAN_CURRENT_HPP_
