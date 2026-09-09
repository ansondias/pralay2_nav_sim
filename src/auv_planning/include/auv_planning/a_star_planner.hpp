#ifndef A_STAR_PLANNER_HPP
#define A_STAR_PLANNER_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_msgs/action/smooth_path.hpp"

namespace auv_planning
{
struct GraphNode
{
    int x;
    int y;
    int cost;
    double heuristic;
    std::shared_ptr<GraphNode> prev;

    GraphNode() : GraphNode(0,0) {}

    GraphNode(int in_x, int in_y) : x(in_x), y(in_y), cost(0){}

    bool operator>(const GraphNode & other) const { 
        return cost + heuristic > other.cost + other.heuristic;
    }

    bool operator==(const GraphNode & other) const {
        return x == other.x && y == other.y;
    }

    GraphNode operator+(std::pair<int, int> const & other) {
        GraphNode res(x + other.first, y + other.second);
        return res;
    }
};

class AStarPlanner : public nav2_core::GlobalPlanner
{
public:
  AStarPlanner() = default;
  ~AStarPlanner() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_util::LifecycleNode::SharedPtr node_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string global_frame_, name_;
  double water_current_speed_;
  double water_current_direction_north_;
  double auv_cruise_speed_{1.0};
  double current_cost_weight_{1.0};
  double obstacle_cost_weight_{3.0};
  int lethal_cost_threshold_{99};

  // Kinematic search (see createPlan). The vehicle is fin-steered and
  // cannot turn tighter than minimum_turning_radius_, so the search carries
  // a heading and expands along arcs of that radius instead of stepping
  // between grid cells. Must match controller_server.FollowPath's
  // min_turning_radius in auv_navigation/config/nav2_params.yaml and
  // auv_current_plugin::ThrustControl's clamp.
  double minimum_turning_radius_{4.16};
  // Arc length of one motion primitive, m. Longer is faster to search and
  // coarser; it must stay well under the turning radius or a single
  // primitive sweeps too much heading to collision-check sensibly.
  double primitive_length_{1.0};
  // Heading discretisation used only to decide when two states are "the
  // same" for the closed set. Too few and genuinely different headings get
  // merged; too many and the search never converges on a cell.
  int heading_bins_{36};
  // How close (m) the search has to get to the goal position. A vehicle
  // with a 4 m turning circle cannot hit a point exactly, and Nav2's goal
  // checker is 2 m anyway.
  double goal_tolerance_{1.0};
  // Safety valve so a hopeless goal fails in bounded time instead of
  // starving the 1 Hz replanning loop.
  int max_expansions_{200000};
  // Smoothing is off by default now: the arcs the search emits already
  // respect the turning radius, and simple_smoother would pull curvature
  // back below it.
  bool smooth_path_{false};
  // One warning per plan when the start is inside the inflated zone.
  mutable bool escaping_logged_{false};

  rclcpp_action::Client<nav2_msgs::action::SmoothPath>::SharedPtr smooth_client_;

  bool poseOnMap(const GraphNode & node);
  
  GraphNode worldToGrid(const geometry_msgs::msg::Pose & pose);
  
  geometry_msgs::msg::Pose gridToWorld(const GraphNode & node);
  
  unsigned int poseToCell(const GraphNode & node);

  double manhattanDistance(const GraphNode &node, const GraphNode &goal_node);

  // A pose in the kinematic search: continuous position and heading, so
  // arcs are not quantised to the grid. The grid is only used for
  // collision checking and for the closed set.
  struct State
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  // Cost of travelling `arc_length` metres in world direction (dir_x, dir_y),
  // in metres of "equivalent distance": the length scaled by the travel-time
  // effect of the water current, plus a penalty for the costmap cost of the
  // cells crossed. See a_star_planner.cpp.
  double arcCost(double dir_x, double dir_y, double arc_length, unsigned char cell_cost) const;

  // Admissible heuristic: straight-line distance scaled by the best-case
  // (fully with-current) multiplier. Ignores the turning constraint, which
  // only ever makes the true path longer, so it stays admissible.
  double heuristic(const State & state, double goal_x, double goal_y) const;

  // True if every sampled point along the arc from `from`, turning at
  // `curvature` (1/m, 0 = straight) for `arc_length` metres, is on the map
  // and below the lethal cost threshold. Writes the arc's end pose to `end`
  // and the worst cell cost encountered to `worst_cost`.
  bool arcIsClear(
    const State & from, double curvature, double arc_length,
    State & end, unsigned char & worst_cost, unsigned char threshold) const;
};

}  // namespace auv_planning

#endif  // A_STAR_PLANNER_HPP