#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

#include "auv_planning/a_star_planner.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace auv_planning
{
void AStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  smooth_client_ = rclcpp_action::create_client<nav2_msgs::action::SmoothPath>(node_, "smooth_path");



// // Parameter initialization
// nav2::declare_parameter_if_not_declared(node_, name_ + ".water_current_speed", rclcpp::ParameterValue(0.0));
// node_->get_parameter(name_ + ".water_current_speed", water_current_speed_);

// // Parameter initialization
// nav2::declare_parameter_if_not_declared(node_, name_ + ".water_current_direction_north", rclcpp::ParameterValue(0.0));
// node_->get_parameter(name_ + ".water_current_direction_north", water_current_direction_north_);


// NEW / FIX:
auto node = parent.lock();
if (!node) {
  throw std::runtime_error("Failed to lock node reference");
}

// Option A: Standard ROS 2 parameter declaration (Recommended)
if (!node->has_parameter(name_ + ".water_current_speed")) {
  node->declare_parameter(name_ + ".water_current_speed", 0.0);
}

if (!node->has_parameter(name_ + ".water_current_direction_north")) {
  node->declare_parameter(name_ + ".water_current_direction_north", 0.0);
}


node->get_parameter(name_ + ".water_current_speed", water_current_speed_);
node->get_parameter(name_ + ".water_current_direction_north", water_current_direction_north_);

// Current-aware cost model knobs (see stepCost) and the impassable threshold.
if (!node->has_parameter(name_ + ".auv_cruise_speed")) {
  node->declare_parameter(name_ + ".auv_cruise_speed", 1.0);
}
if (!node->has_parameter(name_ + ".current_cost_weight")) {
  node->declare_parameter(name_ + ".current_cost_weight", 1.0);
}
if (!node->has_parameter(name_ + ".obstacle_cost_weight")) {
  node->declare_parameter(name_ + ".obstacle_cost_weight", 3.0);
}
if (!node->has_parameter(name_ + ".lethal_cost_threshold")) {
  node->declare_parameter(name_ + ".lethal_cost_threshold", 99);
}
node->get_parameter(name_ + ".auv_cruise_speed", auv_cruise_speed_);
node->get_parameter(name_ + ".current_cost_weight", current_cost_weight_);
node->get_parameter(name_ + ".obstacle_cost_weight", obstacle_cost_weight_);
node->get_parameter(name_ + ".lethal_cost_threshold", lethal_cost_threshold_);

// Kinematic search: the hull cannot turn tighter than this, so the search
// expands along arcs of it instead of stepping cell to cell.
if (!node->has_parameter(name_ + ".minimum_turning_radius")) {
  node->declare_parameter(name_ + ".minimum_turning_radius", 4.16);
}
if (!node->has_parameter(name_ + ".primitive_length")) {
  node->declare_parameter(name_ + ".primitive_length", 1.0);
}
if (!node->has_parameter(name_ + ".heading_bins")) {
  node->declare_parameter(name_ + ".heading_bins", 36);
}
if (!node->has_parameter(name_ + ".goal_tolerance")) {
  node->declare_parameter(name_ + ".goal_tolerance", 1.0);
}
if (!node->has_parameter(name_ + ".max_expansions")) {
  node->declare_parameter(name_ + ".max_expansions", 200000);
}
if (!node->has_parameter(name_ + ".smooth_path")) {
  node->declare_parameter(name_ + ".smooth_path", false);
}
node->get_parameter(name_ + ".minimum_turning_radius", minimum_turning_radius_);
node->get_parameter(name_ + ".primitive_length", primitive_length_);
node->get_parameter(name_ + ".heading_bins", heading_bins_);
node->get_parameter(name_ + ".goal_tolerance", goal_tolerance_);
node->get_parameter(name_ + ".max_expansions", max_expansions_);
node->get_parameter(name_ + ".smooth_path", smooth_path_);

RCLCPP_INFO(
  node_->get_logger(),
  "AStarPlanner: kinematic search, min turning radius %.2f m, primitive %.2f m, "
  "%d heading bins; current %.2f m/s toward %.0f deg, cruise %.2f m/s, current weight %.2f, "
  "obstacle weight %.2f, lethal threshold %d",
  minimum_turning_radius_, primitive_length_, heading_bins_,
  water_current_speed_, water_current_direction_north_, auv_cruise_speed_,
  current_cost_weight_, obstacle_cost_weight_, lethal_cost_threshold_);





}

void AStarPlanner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type AStarPlanner",
    name_.c_str());
}

void AStarPlanner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type AStarPlanner",
    name_.c_str());
  if (!smooth_client_->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
  }
}

void AStarPlanner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type AStarPlanner",
    name_.c_str());
}

namespace
{
/// Wrap an angle into [0, 2*pi).
double wrapTwoPi(double angle)
{
  angle = std::fmod(angle, 2.0 * M_PI);
  return angle < 0.0 ? angle + 2.0 * M_PI : angle;
}
}  // namespace

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = node_->now();

  // Heading-aware ("hybrid") A*. The plain 8-connected grid search this
  // replaced could emit a 45 or 90 degree corner inside one 0.2 m cell.
  // Nothing downstream can drive that: the controller clamps yaw rate to
  // v / min_turning_radius and simply swings wide of the corner, which is
  // how the hull ended up in the reefs. Here a state is (x, y, heading) and
  // the only moves available are arcs the vehicle can actually follow --
  // straight, hard left, hard right, all at minimum_turning_radius_ -- so
  // every path returned is drivable by construction.
  const unsigned int nx = costmap_->getSizeInCellsX();
  const unsigned int ny = costmap_->getSizeInCellsY();
  const int nth = std::max(4, heading_bins_);

  const double goal_x = goal.pose.position.x;
  const double goal_y = goal.pose.position.y;

  unsigned int gmx, gmy;
  if (!costmap_->worldToMap(goal_x, goal_y, gmx, gmy)) {
    RCLCPP_ERROR(node_->get_logger(), "AStarPlanner: goal is outside the costmap");
    return path;
  }
  if (costmap_->getCost(gmx, gmy) >= lethal_cost_threshold_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "AStarPlanner: goal cell cost %d >= %d, i.e. inside the inflated zone of an obstacle; "
      "pick a goal further from obstacles and walls",
      costmap_->getCost(gmx, gmy), lethal_cost_threshold_);
    return path;
  }

  State start_state;
  start_state.x = start.pose.position.x;
  start_state.y = start.pose.position.y;
  start_state.yaw = tf2::getYaw(start.pose.orientation);

  unsigned int smx, smy;
  if (!costmap_->worldToMap(start_state.x, start_state.y, smx, smy)) {
    RCLCPP_ERROR(node_->get_logger(), "AStarPlanner: start is outside the costmap");
    return path;
  }

  // Discrete key of a continuous state: which cell, and which heading bin.
  // Only used to decide when two states are close enough to count as the
  // same node; the arcs themselves stay continuous.
  auto key = [&](const State & st) -> long {
    unsigned int cx, cy;
    if (!costmap_->worldToMap(st.x, st.y, cx, cy)) {
      return -1;
    }
    const int ib =
      static_cast<int>(wrapTwoPi(st.yaw) / (2.0 * M_PI) * nth) % nth;
    return (static_cast<long>(ib) * ny + cy) * nx + cx;
  };

  const size_t n_keys = static_cast<size_t>(nx) * ny * nth;
  std::vector<double> g_best(n_keys, std::numeric_limits<double>::infinity());
  std::vector<char> closed(n_keys, 0);

  std::vector<State> states;
  std::vector<int> parent;
  std::vector<double> g_state;
  states.reserve(4096);
  parent.reserve(4096);
  g_state.reserve(4096);

  struct QueueEntry
  {
    double f;
    int index;
    bool operator>(const QueueEntry & o) const {return f > o.f;}
  };
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;

  const long start_key = key(start_state);
  if (start_key < 0) {
    RCLCPP_ERROR(node_->get_logger(), "AStarPlanner: start is outside the costmap");
    return path;
  }
  states.push_back(start_state);
  parent.push_back(-1);
  g_state.push_back(0.0);
  g_best[start_key] = 0.0;
  open.push({heuristic(start_state, goal_x, goal_y), 0});

  // The three moves a fin-steered hull has: straight ahead, and the
  // tightest turn it can make either way. Reverse is deliberately absent;
  // the controller is not configured to drive paths backwards.
  const double kappa = 1.0 / minimum_turning_radius_;
  const double curvatures[3] = {0.0, kappa, -kappa};

  int expansions = 0;
  int goal_index = -1;
  escaping_logged_ = false;
  while (!open.empty() && rclcpp::ok()) {
    const QueueEntry current = open.top();
    open.pop();

    const State state = states[current.index];
    const long k = key(state);
    if (k < 0 || closed[k]) {
      continue;   // stale entry: a cheaper route to this cell/heading won
    }
    closed[k] = 1;
    ++expansions;

    if (cancel_checker && (expansions % 500 == 0) && cancel_checker()) {
      RCLCPP_WARN(node_->get_logger(), "AStarPlanner: planning cancelled");
      return path;
    }

    if (std::hypot(state.x - goal_x, state.y - goal_y) <= goal_tolerance_) {
      goal_index = current.index;
      break;
    }
    if (expansions >= max_expansions_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "AStarPlanner: gave up after %d expansions; with a %.1f m turning radius the goal may "
        "not be reachable without leaving the arena",
        expansions, minimum_turning_radius_);
      break;
    }

    // Normally an arc may not enter a cell at or above the threshold. But
    // if this state is already inside the inflated zone -- the hull drifts
    // with the current while idle, and can be sitting in one when a goal
    // arrives -- refusing every arc would leave it unable to plan its way
    // out at all. From such a state, allow anything short of a real
    // obstacle (254) or unknown space (255) so it can escape; the cost
    // penalty still pushes it back toward open water.
    unsigned char threshold = static_cast<unsigned char>(lethal_cost_threshold_);
    {
      unsigned int cx, cy;
      if (costmap_->worldToMap(state.x, state.y, cx, cy) &&
        costmap_->getCost(cx, cy) >= lethal_cost_threshold_)
      {
        threshold = 254;   // nav2_costmap_2d::LETHAL_OBSTACLE
        if (!escaping_logged_) {
          escaping_logged_ = true;
          RCLCPP_WARN(
            node_->get_logger(),
            "AStarPlanner: start is inside the inflated zone (cost %d); planning an escape",
            costmap_->getCost(cx, cy));
        }
      }
    }

    for (const double curvature : curvatures) {
      State end;
      unsigned char worst_cost = 0;
      if (!arcIsClear(state, curvature, primitive_length_, end, worst_cost, threshold)) {
        continue;
      }
      const long ke = key(end);
      if (ke < 0 || closed[ke]) {
        continue;
      }
      const double dx = end.x - state.x;
      const double dy = end.y - state.y;
      const double chord = std::hypot(dx, dy);
      if (chord < 1e-9) {
        continue;
      }
      const double tentative =
        g_state[current.index] + arcCost(dx / chord, dy / chord, primitive_length_, worst_cost);
      if (tentative < g_best[ke]) {
        g_best[ke] = tentative;
        states.push_back(end);
        parent.push_back(current.index);
        g_state.push_back(tentative);
        open.push({tentative + heuristic(end, goal_x, goal_y), static_cast<int>(states.size()) - 1});
      }
    }
  }

  if (goal_index < 0) {
    RCLCPP_WARN(
      node_->get_logger(),
      "AStarPlanner: no drivable path from (%.1f, %.1f) to (%.1f, %.1f) after %d expansions",
      start_state.x, start_state.y, goal_x, goal_y, expansions);
    return path;
  }

  std::vector<State> chain;
  for (int i = goal_index; i >= 0; i = parent[i]) {
    chain.push_back(states[i]);
  }
  std::reverse(chain.begin(), chain.end());

  for (const auto & st : chain) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = st.x;
    ps.pose.position.y = st.y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, st.yaw);
    ps.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(ps);
  }
  // Finish exactly on the goal. The search stops within goal_tolerance_ of
  // it, so this last segment is short; the controller's lookahead is longer
  // than it and rounds it off.
  geometry_msgs::msg::PoseStamped goal_pose = goal;
  goal_pose.header = path.header;
  path.poses.push_back(goal_pose);

  RCLCPP_INFO(
    node_->get_logger(),
    "AStarPlanner: drivable path, %zu poses, equivalent length %.1f m (%d expansions)",
    path.poses.size(), g_state[goal_index], expansions);

  // Off by default: the arcs already respect the turning radius and
  // simple_smoother would pull curvature back below it.
  if (smooth_path_ && smooth_client_->action_server_is_ready() && path.poses.size() > 2) {
    nav2_msgs::action::SmoothPath::Goal path_smooth;
    path_smooth.path = path;
    path_smooth.check_for_collisions = true;
    path_smooth.smoother_id = "simple_smoother";
    path_smooth.max_smoothing_duration.sec = 2;
    auto future = smooth_client_->async_send_goal(path_smooth);
    if (future.wait_for(std::chrono::seconds(3)) == std::future_status::ready) {
      auto goal_handle = future.get();
      if (goal_handle) {
        auto result_future = smooth_client_->async_get_result(goal_handle);
        if (result_future.wait_for(std::chrono::seconds(3)) == std::future_status::ready) {
          auto result_path = result_future.get();
          if (result_path.code == rclcpp_action::ResultCode::SUCCEEDED &&
            result_path.result->was_completed)
          {
            path = result_path.result->path;
          }
        }
      }
    }
  }

  return path;
}

bool AStarPlanner::arcIsClear(
  const State & from, double curvature, double arc_length,
  State & end, unsigned char & worst_cost, unsigned char threshold) const
{
  // Sample at half a cell so no obstacle cell can be stepped over. The
  // costmap is already inflated by the hull's radius, so checking the
  // centreline is checking the whole vehicle.
  const double res = costmap_->getResolution();
  const int steps = std::max(2, static_cast<int>(std::ceil(arc_length / (0.5 * res))));
  const double ds = arc_length / steps;

  State s = from;
  worst_cost = 0;
  for (int i = 0; i < steps; ++i) {
    s.yaw += curvature * ds;
    s.x += ds * std::cos(s.yaw);
    s.y += ds * std::sin(s.yaw);

    unsigned int mx, my;
    if (!costmap_->worldToMap(s.x, s.y, mx, my)) {
      return false;   // left the map
    }
    const unsigned char c = costmap_->getCost(mx, my);
    if (c >= threshold) {
      return false;
    }
    worst_cost = std::max(worst_cost, c);
  }
  s.yaw = wrapTwoPi(s.yaw);
  end = s;
  return true;
}

double AStarPlanner::arcCost(
  double dir_x, double dir_y, double arc_length, unsigned char cell_cost) const
{
  // Travel-time model of the current. World frame is ENU (+x east, +y
  // north); water_current_direction_north is the compass bearing the
  // current flows TOWARD, the same convention as
  // auv_current_plugin::OceanCurrent. Ground speed along the arc is the
  // cruise speed plus the current's component along it, so an arc against
  // the current takes longer and costs more.
  double multiplier = 1.0;
  if (water_current_speed_ > 0.0 && current_cost_weight_ > 0.0 && auv_cruise_speed_ > 0.0) {
    const double theta = water_current_direction_north_ * M_PI / 180.0;
    const double along =
      (dir_x * std::sin(theta) + dir_y * std::cos(theta)) *
      water_current_speed_ * current_cost_weight_;
    const double ground_speed = std::max(0.2 * auv_cruise_speed_, auv_cruise_speed_ + along);
    multiplier = auv_cruise_speed_ / ground_speed;
  }

  // Prefer the middle of a gap to its inflated edges, without refusing the
  // gap: at the lethal threshold this adds obstacle_cost_weight_ metres of
  // equivalent distance per metre travelled.
  const double penalty =
    obstacle_cost_weight_ * arc_length * static_cast<double>(cell_cost) / lethal_cost_threshold_;

  return arc_length * multiplier + penalty;
}

double AStarPlanner::heuristic(const State & state, double goal_x, double goal_y) const
{
  const double dist = std::hypot(state.x - goal_x, state.y - goal_y);
  // Best case for any arc is running fully with the current. Scaling the
  // straight-line distance by that never overestimates, and neither does
  // ignoring the turning constraint (which only lengthens the true path),
  // so the search stays optimal under this cost model.
  double best_multiplier = 1.0;
  if (water_current_speed_ > 0.0 && current_cost_weight_ > 0.0 && auv_cruise_speed_ > 0.0) {
    best_multiplier = auv_cruise_speed_ /
      (auv_cruise_speed_ + water_current_speed_ * current_cost_weight_);
  }
  return dist * best_multiplier;
}

double AStarPlanner::manhattanDistance(const GraphNode & node, const GraphNode & goal_node)
{
  return std::abs(node.x - goal_node.x) + std::abs(node.y - goal_node.y);
}

bool AStarPlanner::poseOnMap(const GraphNode & node)
{
  return node.x < static_cast<int>(costmap_->getSizeInCellsX()) && node.x >= 0 &&
         node.y < static_cast<int>(costmap_->getSizeInCellsY()) && node.y >= 0;
}

GraphNode AStarPlanner::worldToGrid(const geometry_msgs::msg::Pose & pose)
{
  const int grid_x = static_cast<int>(std::floor(
      (pose.position.x - costmap_->getOriginX()) / costmap_->getResolution()));
  const int grid_y = static_cast<int>(std::floor(
      (pose.position.y - costmap_->getOriginY()) / costmap_->getResolution()));
  return GraphNode(grid_x, grid_y);
}

geometry_msgs::msg::Pose AStarPlanner::gridToWorld(const GraphNode & node)
{
  // Cell centre, not corner (the corner is half a cell = 0.1 m off).
  geometry_msgs::msg::Pose pose;
  pose.position.x = (node.x + 0.5) * costmap_->getResolution() + costmap_->getOriginX();
  pose.position.y = (node.y + 0.5) * costmap_->getResolution() + costmap_->getOriginY();
  pose.orientation.w = 1.0;
  return pose;
}

unsigned int AStarPlanner::poseToCell(const GraphNode & node)
{
  // Row-major index: width, not the map's world-frame origin.
  return costmap_->getSizeInCellsX() * node.y + node.x;
}

}  // namespace auv_planning

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(auv_planning::AStarPlanner, nav2_core::GlobalPlanner)