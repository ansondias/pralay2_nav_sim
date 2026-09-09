// Mission sequencer: dive to depth, hold it, then run to a target location.
//
// The depth controller (auv_control's depth_hold) and Nav2 are independent:
// send a Nav2 goal and the hull starts driving toward it straight away, at
// whatever depth it happens to be. This node puts them in order, which is
// what a real dive-and-transit mission does:
//
//   1. WAIT_CONTROLLER : wait until depth_hold is publishing, then hand it
//                        the target depth once (publishing it repeatedly
//                        would keep resetting its integrator).
//   2. DIVING          : wait for /depth_hold/in_band, and keep waiting
//                        until it has stayed in band for settle_time, so a
//                        single overshooting pass through the band does not
//                        count as "at depth".
//   3. AT_DEPTH        : on station, waiting for a goal.
//   4. NAVIGATING      : forward the goal to Nav2. Depth hold keeps running
//                        throughout: it owns the elevator pair while Nav2
//                        only commands surge and yaw through cmd_vel, so
//                        the two never fight. Excursions out of the depth
//                        band during the transit are counted and reported.
//                        When the goal finishes it returns to AT_DEPTH,
//                        still holding depth, ready for the next goal.
//
// Goals come from RViz, not from parameters: the "2D Goal Pose" tool in
// auv_bringup/rviz/nav2_default_view.rviz is pointed at goal_topic
// (/mission/goal_pose) instead of Nav2's own /goal_pose, so every goal
// arrives here first and is released only once the vehicle is at depth. A
// goal clicked during the dive is not lost -- it is held and sent the
// moment the depth settles. Publishing straight to /goal_pose still drives
// Nav2 directly, bypassing this gate, which is occasionally useful for
// debugging but is not the mission path.
//
// Run:  ros2 launch auv_bringup mission.launch.py     (then click a goal in RViz)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

namespace auv_bringup
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class DiveAndNavigate : public rclcpp::Node
{
public:
  DiveAndNavigate()
  : Node("dive_and_navigate")
  {
    target_depth_ = declare_parameter("target_depth", 1.0);       // m below surface
    // Where goals arrive from. Deliberately NOT /goal_pose: bt_navigator
    // subscribes to that one and would start the transit immediately, at
    // whatever depth the hull happened to be.
    const std::string goal_topic =
      declare_parameter("goal_topic", std::string("/mission/goal_pose"));
    // How long the depth must stay inside the band before the transit
    // starts. One brief pass through 0.8..1.2 m on the way down is not
    // "at depth"; a few seconds of it is.
    settle_time_ = declare_parameter("settle_time", 5.0);
    // Give up waiting to reach depth after this long and say so, rather
    // than hanging silently forever.
    dive_timeout_ = declare_parameter("dive_timeout", 120.0);

    target_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/depth_hold/target_depth", rclcpp::QoS(1).transient_local());
    enable_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/depth_hold/enable", rclcpp::QoS(1).transient_local());

    state_sub_ = create_subscription<std_msgs::msg::String>(
      "/depth_hold/state", 10,
      [this](const std_msgs::msg::String::SharedPtr m) {
        depth_state_ = m->data;
        controller_seen_ = true;
      });
    in_band_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/depth_hold/in_band", 10,
      [this](const std_msgs::msg::Bool::SharedPtr m) {onInBand(m->data);});
    depth_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/depth_hold/depth", 10,
      [this](const std_msgs::msg::Float64::SharedPtr m) {depth_ = m->data;});

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr m) {onGoal(*m);});

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    started_ = now();
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::milliseconds(500),
      std::bind(&DiveAndNavigate::step, this));

    RCLCPP_INFO(
      get_logger(),
      "mission: dive to %.2f m, hold it for %.1f s, then run to goals sent on %s "
      "(RViz \"2D Goal Pose\")",
      target_depth_, settle_time_, goal_topic.c_str());
  }

private:
  enum class Phase {WAIT_CONTROLLER, DIVING, AT_DEPTH, NAVIGATING, FAILED};

  void onGoal(const geometry_msgs::msg::PoseStamped & goal)
  {
    pending_goal_ = goal;
    if (phase_ == Phase::AT_DEPTH || phase_ == Phase::NAVIGATING) {
      RCLCPP_INFO(
        get_logger(), "goal (%.1f, %.1f) received at depth, starting the transit",
        goal.pose.position.x, goal.pose.position.y);
      sendGoal();
    } else {
      RCLCPP_INFO(
        get_logger(),
        "goal (%.1f, %.1f) received while still reaching depth; held until the hull "
        "settles at %.2f m",
        goal.pose.position.x, goal.pose.position.y, target_depth_);
    }
  }

  void onInBand(bool in_band)
  {
    if (in_band && !in_band_) {
      in_band_since_ = now();
    }
    if (!in_band) {
      in_band_since_.reset();
      if (phase_ == Phase::NAVIGATING) {
        ++excursions_;
        RCLCPP_WARN(
          get_logger(), "depth left the band during transit (%.2f m), excursion %d",
          depth_, excursions_);
      }
    }
    in_band_ = in_band;
  }

  void step()
  {
    switch (phase_) {
      case Phase::WAIT_CONTROLLER:
        if (controller_seen_) {
          // The controller is alive: hand it the target once. Its own
          // subscriber resets the integrator on every message, so this
          // must not be repeated.
          std_msgs::msg::Bool on;
          on.data = true;
          enable_pub_->publish(on);
          std_msgs::msg::Float64 t;
          t.data = target_depth_;
          target_pub_->publish(t);
          RCLCPP_INFO(get_logger(), "depth controller up, commanded %.2f m", target_depth_);
          phase_ = Phase::DIVING;
        } else if ((now() - started_).seconds() > 30.0) {
          RCLCPP_ERROR(
            get_logger(),
            "no /depth_hold/state after 30 s: is auv_control's depth_hold running?");
          phase_ = Phase::FAILED;
        }
        break;

      case Phase::DIVING: {
          const double held = in_band_since_ ? (now() - *in_band_since_).seconds() : 0.0;
          if (in_band_ && held >= settle_time_) {
            phase_ = Phase::AT_DEPTH;
            RCLCPP_INFO(
              get_logger(),
              "at depth: %.2f m held for %.1f s. %s",
              depth_, held,
              pending_goal_ ? "Releasing the goal that arrived during the dive." :
              "Waiting for a goal (RViz \"2D Goal Pose\").");
            if (pending_goal_) {
              sendGoal();
            }
          } else if ((now() - started_).seconds() > dive_timeout_) {
            RCLCPP_ERROR(
              get_logger(),
              "did not settle at %.2f m within %.0f s (depth %.2f m, controller %s); "
              "not starting the transit",
              target_depth_, dive_timeout_, depth_, depth_state_.c_str());
            phase_ = Phase::FAILED;
          }
          break;
        }

      case Phase::AT_DEPTH:
      case Phase::NAVIGATING:
      case Phase::FAILED:
        break;
    }
  }

  void sendGoal()
  {
    if (!pending_goal_) {
      return;
    }
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "navigate_to_pose action server not available; is Nav2 up?");
      phase_ = Phase::FAILED;
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose = *pending_goal_;
    // RViz stamps the click with the fixed frame; make sure it is one Nav2
    // can plan in, and re-stamp so a goal held through a long dive is not
    // rejected as stale.
    if (goal.pose.header.frame_id.empty()) {
      goal.pose.header.frame_id = "map";
    }
    goal.pose.header.stamp = now();
    const double gx = goal.pose.pose.position.x;
    const double gy = goal.pose.pose.position.y;
    pending_goal_.reset();

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
    opts.goal_response_callback = [this](GoalHandle::SharedPtr handle) {
        if (!handle) {
          RCLCPP_ERROR(get_logger(), "Nav2 rejected the goal");
          phase_ = Phase::FAILED;
        } else {
          RCLCPP_INFO(get_logger(), "Nav2 accepted the goal, running the transit at depth");
        }
      };
    opts.result_callback = [this](const GoalHandle::WrappedResult & result) {
        const bool ok = result.code == rclcpp_action::ResultCode::SUCCEEDED;
        RCLCPP_INFO(
          get_logger(),
          "%s. Depth at finish %.2f m, %d excursion(s) out of the band during transit. "
          "Holding depth, ready for the next goal.",
          ok ? "Reached the target location" : "Nav2 did not reach the target",
          depth_, excursions_);
        excursions_ = 0;
        // Whatever Nav2 made of it, the hull is still at depth and can take
        // another goal; only a depth failure is terminal for this node.
        phase_ = Phase::AT_DEPTH;
      };

    RCLCPP_INFO(get_logger(), "sending goal (%.1f, %.1f) to Nav2", gx, gy);
    nav_client_->async_send_goal(goal, opts);
    phase_ = Phase::NAVIGATING;
  }

  double target_depth_, settle_time_, dive_timeout_;
  std::optional<geometry_msgs::msg::PoseStamped> pending_goal_;

  Phase phase_{Phase::WAIT_CONTROLLER};
  bool controller_seen_{false};
  bool in_band_{false};
  std::optional<rclcpp::Time> in_band_since_;
  rclcpp::Time started_;
  double depth_{0.0};
  std::string depth_state_{"?"};
  int excursions_{0};

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr in_band_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depth_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace auv_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<auv_bringup::DiveAndNavigate>());
  rclcpp::shutdown();
  return 0;
}
