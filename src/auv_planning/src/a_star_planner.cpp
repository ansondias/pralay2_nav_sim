#include "auv_planning/a_star_planner.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace auv_planning
{

namespace
{
// 8-connected neighbour offsets.
constexpr int kNeighbours[8][2] = {
  {1, 0}, {-1, 0}, {0, 1}, {0, -1},
  {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
};

int CellIndex(int x, int y, int width)
{
  return y * width + x;
}
}  // namespace

void AStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  auto node = parent.lock();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".water_current_speed", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".water_current_direction_north", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".current_cost_weight", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));

  node->get_parameter(name_ + ".water_current_speed", water_current_speed_);
  node->get_parameter(name_ + ".water_current_direction_north", water_current_direction_north_);
  node->get_parameter(name_ + ".current_cost_weight", current_cost_weight_);
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);

  RCLCPP_INFO(
    node->get_logger(),
    "AStarPlanner[%s]: water_current_speed=%.2f m/s, "
    "water_current_direction_north=%.1f deg, current_cost_weight=%.2f",
    name_.c_str(), water_current_speed_, water_current_direction_north_, current_cost_weight_);
}

void AStarPlanner::cleanup() {}
void AStarPlanner::activate() {}
void AStarPlanner::deactivate() {}

double AStarPlanner::currentCostMultiplier(int dx, int dy) const
{
  if (water_current_speed_ <= 0.0 || (dx == 0 && dy == 0)) {
    return 1.0;
  }

  // World frame is ENU: +X east, +Y north. direction_north is the compass
  // bearing (clockwise from north) the current flows toward -- kept in the
  // same convention as auv_current_plugin's OceanCurrent Gazebo plugin so a
  // scenario's current can be mirrored between sim and planner params.
  const double theta = water_current_direction_north_ * M_PI / 180.0;
  const double current_x = std::sin(theta);
  const double current_y = std::cos(theta);

  const double step_len = std::sqrt(static_cast<double>(dx * dx + dy * dy));
  const double step_x = dx / step_len;
  const double step_y = dy / step_len;

  // +1 = moving with the current, -1 = moving straight into it.
  const double alignment = step_x * current_x + step_y * current_y;

  const double strength = std::tanh(water_current_speed_) * current_cost_weight_;
  const double multiplier = 1.0 - strength * alignment;

  return std::clamp(multiplier, 0.2, 2.0);
}

double AStarPlanner::octileHeuristic(int x0, int y0, int x1, int y1) const
{
  const double dx = std::abs(x1 - x0);
  const double dy = std::abs(y1 - y0);
  constexpr double kD1 = 1.0;
  constexpr double kD2 = M_SQRT2;
  return kD1 * (dx + dy) + (kD2 - 2 * kD1) * std::min(dx, dy);
}

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path path;
  auto node = node_.lock();
  path.header.frame_id = global_frame_;
  path.header.stamp = node ? node->now() : rclcpp::Clock().now();

  unsigned int start_mx, start_my, goal_mx, goal_my;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my)) {
    throw std::runtime_error("AStarPlanner: start pose is outside of the costmap bounds");
  }
  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my)) {
    throw std::runtime_error("AStarPlanner: goal pose is outside of the costmap bounds");
  }

  const int width = static_cast<int>(costmap_->getSizeInCellsX());
  const int height = static_cast<int>(costmap_->getSizeInCellsY());
  const int start_x = static_cast<int>(start_mx);
  const int start_y = static_cast<int>(start_my);
  const int goal_x = static_cast<int>(goal_mx);
  const int goal_y = static_cast<int>(goal_my);

  std::vector<AStarCell> pool;
  pool.reserve(static_cast<size_t>(width) * static_cast<size_t>(height) / 4 + 16);

  auto cmp = [&pool](int a, int b) {return pool[a].F() > pool[b].F();};
  std::priority_queue<int, std::vector<int>, decltype(cmp)> open(cmp);

  std::unordered_map<int, double> best_g;
  std::unordered_map<int, int> came_from_index;

  AStarCell start_cell;
  start_cell.x = start_x;
  start_cell.y = start_y;
  start_cell.g = 0.0;
  start_cell.h = octileHeuristic(start_x, start_y, goal_x, goal_y);
  start_cell.parent_index = -1;
  pool.push_back(start_cell);
  open.push(0);
  best_g[CellIndex(start_x, start_y, width)] = 0.0;

  int goal_pool_index = -1;
  int expansions = 0;

  while (!open.empty()) {
    if ((++expansions % 500) == 0 && cancel_checker()) {
      throw std::runtime_error("AStarPlanner: planning was cancelled");
    }

    const int current_index = open.top();
    open.pop();
    const AStarCell current = pool[current_index];
    const int current_cell_index = CellIndex(current.x, current.y, width);

    if (auto it = best_g.find(current_cell_index);
      it != best_g.end() && current.g > it->second + 1e-9)
    {
      continue;  // stale queue entry
    }

    if (current.x == goal_x && current.y == goal_y) {
      goal_pool_index = current_index;
      break;
    }

    for (const auto & offset : kNeighbours) {
      const int nx = current.x + offset[0];
      const int ny = current.y + offset[1];
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
        continue;
      }

      const unsigned char cost = costmap_->getCost(nx, ny);
      if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
        cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        continue;
      }
      if (!allow_unknown_ && cost == nav2_costmap_2d::NO_INFORMATION) {
        continue;
      }

      const double step_cost =
        std::sqrt(static_cast<double>(offset[0] * offset[0] + offset[1] * offset[1])) *
        currentCostMultiplier(offset[0], offset[1]) *
        (1.0 + static_cast<double>(cost) / 255.0);

      const double tentative_g = current.g + step_cost;
      const int neighbour_cell_index = CellIndex(nx, ny, width);

      auto it = best_g.find(neighbour_cell_index);
      if (it != best_g.end() && tentative_g >= it->second - 1e-9) {
        continue;
      }

      best_g[neighbour_cell_index] = tentative_g;

      AStarCell neighbour;
      neighbour.x = nx;
      neighbour.y = ny;
      neighbour.g = tentative_g;
      neighbour.h = octileHeuristic(nx, ny, goal_x, goal_y);
      neighbour.parent_index = current_index;

      pool.push_back(neighbour);
      open.push(static_cast<int>(pool.size()) - 1);
    }
  }

  if (goal_pool_index < 0) {
    throw std::runtime_error("AStarPlanner: no valid path could be found");
  }

  std::vector<geometry_msgs::msg::PoseStamped> reversed;
  for (int idx = goal_pool_index; idx != -1; idx = pool[idx].parent_index) {
    double wx, wy;
    costmap_->mapToWorld(pool[idx].x, pool[idx].y, wx, wy);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.orientation.w = 1.0;
    reversed.push_back(pose);
  }
  std::reverse(reversed.begin(), reversed.end());

  for (size_t i = 0; i < reversed.size(); ++i) {
    if (i + 1 < reversed.size()) {
      const double dx = reversed[i + 1].pose.position.x - reversed[i].pose.position.x;
      const double dy = reversed[i + 1].pose.position.y - reversed[i].pose.position.y;
      tf2::Quaternion q;
      q.setRPY(0, 0, std::atan2(dy, dx));
      reversed[i].pose.orientation = tf2::toMsg(q);
    } else if (i > 0) {
      reversed[i].pose.orientation = reversed[i - 1].pose.orientation;
    }
    path.poses.push_back(reversed[i]);
  }

  path.poses.front().pose.orientation = start.pose.orientation;
  path.poses.back().pose.orientation = goal.pose.orientation;

  return path;
}

}  // namespace auv_planning

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(auv_planning::AStarPlanner, nav2_core::GlobalPlanner)
