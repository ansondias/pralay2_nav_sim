#ifndef AUV_CURRENT_PLUGIN__OCEAN_CURRENT_HPP_
#define AUV_CURRENT_PLUGIN__OCEAN_CURRENT_HPP_

#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/vector3d.pb.h>

#include <mutex>
#include <string>

namespace auv_current_plugin
{

// Applies a quadratic-drag horizontal wrench that pulls a link toward a
// configurable ocean current velocity. Current speed/direction can be set
// at load time via SDF or updated live over a Gazebo Transport topic, so
// the same speed/direction values used here can be mirrored into the
// auv_planning nav2 params (water_current_speed / water_current_direction_north)
// for a scenario. This is a deliberately simple stand-in for real
// hydrodynamics (no buoyancy/added-mass modelling) -- see auv_gazebo/README.
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

  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Link link_{gz::sim::kNullEntity};

  gz::transport::Node node_;

  std::mutex mutex_;
  double speed_mps_{0.0};
  double direction_north_deg_{0.0};

  double drag_coefficient_{1.0};
  double fluid_density_{1025.0};   // kg/m^3, nominal seawater
  double cross_section_area_{0.3}; // m^2, frontal area of the hull
};

}  // namespace auv_current_plugin

#endif  // AUV_CURRENT_PLUGIN__OCEAN_CURRENT_HPP_
