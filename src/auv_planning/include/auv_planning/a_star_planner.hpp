#ifndef AUV_PLANNING__A_STAR_PLANNER_HPP_
#define AUV_PLANNING__A_STAR_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace auv_planning
{

// Grid cell used during the A* search. `g` is the accumulated path cost
// from the start, `h` is the heuristic estimate to the goal.
struct AStarCell
{
  int x{0};
  int y{0};
  double g{0.0};
  double h{0.0};
  int parent_index{-1};

  double F() const {return g + h;}
};

// A* global planner ("GridBased" plugin) that is aware of a horizontal
// water current: steps that go with the current are made cheaper, steps
// that go against it are made more expensive, so the planned path drifts
// toward current-assisted routes as water_current_speed grows. This is an
// intentionally simple, from-scratch implementation meant as a starting
// point for a custom planning algorithm -- see auv_planning/README.md.
class AStarPlanner : public nav2_core::GlobalPlanner
{
public:
  AStarPlanner() = default;
  ~AStarPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  // Cost multiplier for a step taken in direction (dx, dy) (grid units),
  // given the configured current. 1.0 = no effect; < 1.0 = cheaper
  // (with the current); > 1.0 = more expensive (against the current).
  double currentCostMultiplier(int dx, int dy) const;

  double octileHeuristic(int x0, int y0, int x1, int y1) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string global_frame_;
  std::string name_;

  double water_current_speed_{0.0};
  double water_current_direction_north_{0.0};
  double current_cost_weight_{0.5};
  bool allow_unknown_{true};
};

}  // namespace auv_planning

#endif  // AUV_PLANNING__A_STAR_PLANNER_HPP_
