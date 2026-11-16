// Mission sequencer for the Pralay-2: the twelve-phase workflow.
//
//   1 DEPLOY          inert: depth hold disabled, no cmd_vel, so fins centre
//                     and the propeller stays off.
//   2 SENSOR_CHECK    depth reading present (after the Bar30 surface zero),
//                     IMU fresh, odometry fresh. Fail: stay inert.
//   3 PLAN            while no goal is set, hold station (holdStation: the
//                     current otherwise carries the hull out of the arena in
//                     a minute). Then ask Nav2's planner for a path to the
//                     goal and check it: V - C above the margin on every leg,
//                     and, if a battery reading exists, energy for the route
//                     plus reserve. No-go: end state, never armed.
//   4 ARM             enable depth hold, holding the current depth.
//   5 CRUISE          command surge along the first leg until measured speed
//                     is enough for the fins to have authority. A fin-steered
//                     hull cannot dive standing still; this phase is what the
//                     earlier version lacked.
//   6 DIVE            command the target depth, still cruising along leg 1.
//   7 SETTLE          in band for settle_time.
//   8 TRANSIT         hand the goal to Nav2 (surge + yaw); depth hold owns
//                     the elevator throughout.
//   9 DELIVERY_DEPTH  (enable_delivery) re-target depth to the delivery depth.
//  10 TERMINAL        (enable_delivery) slow closure to contact_range, aligned.
//  11 DELIVER         (enable_delivery) release, confirm, reset the depth
//                     integrator so the trim shift from the lost mass is not
//                     fought as an error.
//  12 END_STATE       report the outcome, hold station (holdStation again,
//                     unless parked); a new goal starts a continuation at
//                     PLAN, a new mission_id resets to DEPLOY.
//
// MONITORS run every cycle from CRUISE onward: sensor freshness, depth and
// pitch bounds, energy against the remaining route, V - C toward the goal.
// A trip goes to abort. The vehicle is one-way, so abort never retreats:
// on_abort is "hold" (stay at depth) or "park" (thrust off, fins neutral,
// depth hold off). Abort never releases the payload.
//
// Goals come from RViz on goal_topic (/mission/goal_pose). Nav2's own
// /goal_pose is deliberately not used: bt_navigator would act on it at once.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"   // fromMsg() behind tf2::getYaw
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace auv_bringup
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using ComputePath = nav2_msgs::action::ComputePathToPose;
using PathHandle = rclcpp_action::ClientGoalHandle<ComputePath>;

namespace
{
double wrapPi(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}
double pitchOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
}
}  // namespace

class DiveAndNavigate : public rclcpp::Node
{
public:
  DiveAndNavigate()
  : Node("dive_and_navigate"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    // ---- mission ---------------------------------------------------------------
    target_depth_ = declare_parameter("target_depth", 1.0);
    settle_time_ = declare_parameter("settle_time", 5.0);
    const std::string goal_topic =
      declare_parameter("goal_topic", std::string("/mission/goal_pose"));
    const std::string mission_id_topic =
      declare_parameter("mission_id_topic", std::string("/mission/id"));
    require_arm_command_ = declare_parameter("require_arm_command", false);
    const std::string arm_topic = declare_parameter("arm_topic", std::string("/mission/arm"));
    on_abort_ = declare_parameter("on_abort", std::string("hold"));   // hold | park

    // ---- sensors / freshness ------------------------------------------------------
    const std::string imu_topic = declare_parameter("imu_topic", std::string("/imu/data"));
    const std::string odom_topic =
      declare_parameter("odom_topic", std::string("/odometry/filtered"));
    sensor_timeout_ = declare_parameter("sensor_timeout", 1.0);
    sensor_check_timeout_ = declare_parameter("sensor_check_timeout", 60.0);

    // ---- feasibility (phase 3) and live V - C monitor --------------------------------
    max_speed_ = declare_parameter("max_attainable_speed", 2.06);
    current_speed_ = declare_parameter("water_current_speed", 0.3);
    current_dir_deg_ = declare_parameter("water_current_direction_north", 135.0);
    speed_margin_ = declare_parameter("min_speed_margin_vs_current", 0.3);
    hull_power_w_ = declare_parameter("hull_power_w", 60.0);
    reserve_fraction_ = declare_parameter("energy_reserve_fraction", 0.2);
    energy_wh_ = declare_parameter("energy_remaining_wh", -1.0);   // < 0: unknown, checks skipped
    const std::string energy_topic =
      declare_parameter("energy_topic", std::string("/battery/energy_wh"));

    // ---- cruise / dive (phases 5-7) ------------------------------------------------
    cruise_speed_ = declare_parameter("cruise_speed", 1.0);
    min_fin_speed_ = declare_parameter("min_speed_for_fins", 0.3);
    // How long after entering TRANSIT a goal rejection is still retried
    // (Nav2's navigator activating late) before it becomes an abort.
    nav_reject_timeout_ = declare_parameter("nav_reject_timeout", 20.0);
    cruise_timeout_ = declare_parameter("cruise_timeout", 30.0);
    dive_timeout_ = declare_parameter("dive_timeout", 120.0);
    min_turning_radius_ = declare_parameter("min_turning_radius", 5.0);
    kp_yaw_ = declare_parameter("kp_yaw", 1.0);

    // ---- station keeping while waiting for a goal (PLAN, END_STATE) --------------------
    station_keeping_ = declare_parameter("station_keeping", true);
    station_radius_ = declare_parameter("station_radius", 3.0);   // m off station before returning
    station_inner_radius_ = declare_parameter("station_inner_radius", 1.5);   // m, return done

    // ---- monitors -------------------------------------------------------------------
    max_mission_depth_ = declare_parameter("max_mission_depth", 4.5);
    abort_pitch_ = declare_parameter("abort_pitch", 0.70);   // rad, ~40 deg

    // ---- delivery (phases 9-11), off by default ----------------------------------------
    enable_delivery_ = declare_parameter("enable_delivery", false);
    delivery_depth_ = declare_parameter("delivery_depth", 1.0);
    terminal_speed_ = declare_parameter("terminal_speed", 0.3);
    contact_range_ = declare_parameter("contact_range", 1.0);
    align_tolerance_ = declare_parameter("align_tolerance", 0.3);
    approach_bearing_ = declare_parameter("approach_bearing", std::nan(""));   // NaN: toward goal
    terminal_timeout_ = declare_parameter("terminal_timeout", 90.0);
    const std::string release_topic =
      declare_parameter("release_topic", std::string("/payload/release"));
    const std::string release_confirm_topic =
      declare_parameter("release_confirm_topic", std::string("/payload/released"));
    release_timeout_ = declare_parameter("release_timeout", 10.0);

    // ---- I/O ---------------------------------------------------------------------------
    const auto latched = rclcpp::QoS(1).transient_local();
    enable_pub_ = create_publisher<std_msgs::msg::Bool>("/depth_hold/enable", latched);
    target_pub_ = create_publisher<std_msgs::msg::Float64>("/depth_hold/target_depth", latched);
    reset_pub_ = create_publisher<std_msgs::msg::Bool>("/depth_hold/reset_integrator", 10);
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    release_pub_ = create_publisher<std_msgs::msg::Bool>(release_topic, latched);
    outcome_pub_ = create_publisher<std_msgs::msg::String>("/mission/outcome", latched);
    phase_pub_ = create_publisher<std_msgs::msg::String>("/mission/phase", latched);

    state_sub_ = create_subscription<std_msgs::msg::String>(
      "/depth_hold/state", 10,
      [this](std_msgs::msg::String::SharedPtr m) {depth_state_ = m->data; controller_seen_ = true;});
    in_band_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/depth_hold/in_band", 10, [this](std_msgs::msg::Bool::SharedPtr m) {onInBand(m->data);});
    depth_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/depth_hold/depth", 10,
      [this](std_msgs::msg::Float64::SharedPtr m) {depth_ = m->data; depth_time_ = now();});
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, 20,
      [this](sensor_msgs::msg::Imu::SharedPtr m) {pitch_ = pitchOf(m->orientation); imu_time_ = now();});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      [this](nav_msgs::msg::Odometry::SharedPtr m) {
        speed_ = m->twist.twist.linear.x;
        sway_ = m->twist.twist.linear.y;   // body-frame: the current's push across the hull
        odom_time_ = now();
      });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic, 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {onGoal(*m);});
    mission_id_sub_ = create_subscription<std_msgs::msg::String>(
      mission_id_topic, 10, [this](std_msgs::msg::String::SharedPtr m) {onMissionId(m->data);});
    arm_sub_ = create_subscription<std_msgs::msg::Bool>(
      arm_topic, 10, [this](std_msgs::msg::Bool::SharedPtr m) {arm_command_ = m->data;});
    energy_sub_ = create_subscription<std_msgs::msg::Float64>(
      energy_topic, 10, [this](std_msgs::msg::Float64::SharedPtr m) {energy_wh_ = m->data;});
    release_confirm_sub_ = create_subscription<std_msgs::msg::Bool>(
      release_confirm_topic, 10,
      [this](std_msgs::msg::Bool::SharedPtr m) {if (m->data) {release_confirmed_ = true;}});

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    path_client_ = rclcpp_action::create_client<ComputePath>(this, "compute_path_to_pose");

    enterPhase(Phase::DEPLOY);
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::milliseconds(100), std::bind(&DiveAndNavigate::step, this));

    RCLCPP_INFO(
      get_logger(),
      "mission: depth %.2f m, cruise %.2f m/s, current %.2f m/s toward %.0f deg, delivery %s; "
      "goals on %s", target_depth_, cruise_speed_, current_speed_, current_dir_deg_,
      enable_delivery_ ? "ON" : "off", goal_topic.c_str());
  }

private:
  enum class Phase
  {
    DEPLOY, SENSOR_CHECK, PLAN, ARM, CRUISE, DIVE, SETTLE, TRANSIT,
    DELIVERY_DEPTH, TERMINAL, DELIVER, END_STATE
  };

  static const char * name(Phase p)
  {
    switch (p) {
      case Phase::DEPLOY: return "1 DEPLOY";
      case Phase::SENSOR_CHECK: return "2 SENSOR_CHECK";
      case Phase::PLAN: return "3 PLAN";
      case Phase::ARM: return "4 ARM";
      case Phase::CRUISE: return "5 CRUISE";
      case Phase::DIVE: return "6 DIVE";
      case Phase::SETTLE: return "7 SETTLE";
      case Phase::TRANSIT: return "8 TRANSIT";
      case Phase::DELIVERY_DEPTH: return "9 DELIVERY_DEPTH";
      case Phase::TERMINAL: return "10 TERMINAL";
      case Phase::DELIVER: return "11 DELIVER";
      case Phase::END_STATE: return "12 END_STATE";
    }
    return "?";
  }

  // ------------------------------------------------------------------ helpers
  double age(const std::optional<rclcpp::Time> & t) const
  {
    return t ? (now() - *t).seconds() : 1e9;
  }
  double sincePhase() const {return (now() - phase_since_).seconds();}

  void enterPhase(Phase p)
  {
    phase_ = p;
    phase_since_ = now();
    station_.reset();   // a waiting phase re-captures it on its first cycle
    returning_ = false;
    std_msgs::msg::String m;
    m.data = name(p);
    phase_pub_->publish(m);
    RCLCPP_INFO(get_logger(), "phase -> %s", name(p));
  }

  void setDepthHold(bool on)
  {
    std_msgs::msg::Bool m;
    m.data = on;
    enable_pub_->publish(m);
  }
  void setTargetDepth(double d)
  {
    std_msgs::msg::Float64 m;
    m.data = std::max(0.0, d);
    target_pub_->publish(m);
  }
  void stopThrust()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist());   // zeros; plugin/mapper also time out
  }

  std::optional<std::pair<double, double>> vehiclePosition() const
  {
    try {
      const auto t = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
      return std::make_pair(t.transform.translation.x, t.transform.translation.y);
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }
  std::optional<double> vehicleYaw() const
  {
    try {
      const auto t = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
      return tf2::getYaw(t.transform.rotation);
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  /// Ground speed available along a world-frame unit direction: V + (current . dir).
  double groundSpeedAlong(double dx, double dy) const
  {
    const double th = current_dir_deg_ * M_PI / 180.0;
    const double along = (dx * std::sin(th) + dy * std::cos(th)) * current_speed_;
    return max_speed_ + along;
  }

  /// Steer toward a world heading at a given surge speed (phases 5-7 and 10).
  /// A negative speed backs toward it: the stern, not the bow, is aligned.
  void steerToward(double bearing, double speed)
  {
    geometry_msgs::msg::Twist t;
    t.linear.x = speed;
    if (const auto yaw = vehicleYaw()) {
      const double err = wrapPi(bearing - *yaw - (speed < 0.0 ? M_PI : 0.0));
      const double max_rate = std::abs(speed) / min_turning_radius_;
      t.angular.z = std::clamp(kp_yaw_ * err, -max_rate, max_rate);
    }
    cmd_pub_->publish(t);
  }

  /// Waiting for a goal: shuttle about where the wait began (see shuttle), and
  /// if the current still carries the hull station_radius_ off, steer it back
  /// (astern if the station is behind: a U-turn at min_turning_radius_ needs
  /// room a corner of the arena does not have).
  void holdStation()
  {
    if (!station_keeping_) {return;}
    const auto p = vehiclePosition();
    if (!p) {return;}
    if (!station_) {station_ = p; returning_ = false;}
    const double dx = station_->first - p->first;
    const double dy = station_->second - p->second;
    const double dist = std::hypot(dx, dy);
    if (!returning_) {
      if (dist > station_radius_) {
        returning_ = true;
        station_astern_ = false;
        RCLCPP_INFO(get_logger(), "station: carried %.1f m off, returning", dist);
      } else {
        shuttle(dx, dy);
        return;
      }
    } else if (dist < station_inner_radius_) {
      returning_ = false;
      RCLCPP_INFO(get_logger(), "station: back within %.1f m, holding", dist);
      shuttle(dx, dy);
      return;
    }
    const double bearing = std::atan2(dy, dx);
    if (const auto yaw = vehicleYaw()) {   // hysteresis around abeam, or it chatters
      const double off = std::abs(wrapPi(bearing - *yaw));
      if (off > 100.0 * M_PI / 180.0) {station_astern_ = true;}
      if (off < 80.0 * M_PI / 180.0) {station_astern_ = false;}
    }
    steerToward(bearing, station_astern_ ? -kStationSpeed : cruise_speed_);
  }

  /// Holding: creep +-1 m along the hull axis at kStationSpeed so the fins keep
  /// authority, and yaw to null the sway, which settles the bow into the
  /// current (a braked hull weathervanes beam-on and is carried off).
  void shuttle(double dx, double dy)
  {
    const auto yaw = vehicleYaw();
    if (!yaw) {stopThrust(); return;}
    const double ahead = std::cos(*yaw) * dx + std::sin(*yaw) * dy;
    if (shuttle_dir_ == 0) {shuttle_dir_ = ahead >= 0.0 ? 1 : -1;}
    if (shuttle_dir_ > 0 && ahead < -1.0) {shuttle_dir_ = -1;}
    if (shuttle_dir_ < 0 && ahead > 1.0) {shuttle_dir_ = 1;}
    geometry_msgs::msg::Twist t;
    t.linear.x = shuttle_dir_ * kStationSpeed;
    const double max_rate = kStationSpeed / min_turning_radius_;
    t.angular.z = std::clamp(-kSwayGain * sway_, -max_rate, max_rate);
    cmd_pub_->publish(t);
  }

  void onInBand(bool in_band)
  {
    if (in_band && !in_band_) {in_band_since_ = now();}
    if (!in_band) {
      in_band_since_.reset();
      if (phase_ == Phase::TRANSIT || phase_ == Phase::TERMINAL) {
        ++excursions_;
        RCLCPP_WARN(get_logger(), "depth left the band (%.2f m), excursion %d", depth_, excursions_);
      }
    }
    in_band_ = in_band;
  }

  void onGoal(const geometry_msgs::msg::PoseStamped & g)
  {
    goal_ = g;
    if (goal_->header.frame_id.empty()) {goal_->header.frame_id = "map";}
    RCLCPP_INFO(
      get_logger(), "goal (%.1f, %.1f) received in %s", g.pose.position.x, g.pose.position.y,
      name(phase_));
    returning_ = false;
    if (phase_ == Phase::END_STATE) {
      // Continuation: already armed and at depth, so the plan/cruise/dive
      // gates pass quickly; a new mission_id is what forces a full reset.
      resetRun();
      enterPhase(Phase::PLAN);
    }
  }

  void onMissionId(const std::string & id)
  {
    if (id == mission_id_) {return;}
    RCLCPP_INFO(get_logger(), "new mission_id '%s': full reset to DEPLOY", id.c_str());
    mission_id_ = id;
    cancelNav();
    stopThrust();
    resetRun();
    goal_.reset();
    armed_ = false;
    parked_ = false;
    outcome_sent_ = false;
    enterPhase(Phase::DEPLOY);
  }

  void resetRun()
  {
    path_.reset();
    plan_pending_ = false;
    excursions_ = 0;
    nav_result_.reset();
    nav_handle_.reset();
    distance_remaining_ = -1.0;
    release_confirmed_ = false;
    release_sent_ = false;
    outcome_sent_ = false;
    target_sent_ = false;
    in_band_since_.reset();
    std_msgs::msg::Bool r;
    r.data = true;
    reset_pub_->publish(r);
  }

  void cancelNav()
  {
    if (nav_handle_) {
      nav_client_->async_cancel_goal(nav_handle_);
      nav_handle_.reset();
    }
  }

  void finish(const std::string & outcome)
  {
    if (!outcome_sent_) {
      std_msgs::msg::String m;
      m.data = outcome;
      outcome_pub_->publish(m);
      outcome_sent_ = true;
      RCLCPP_INFO(
        get_logger(), "OUTCOME: %s (depth %.2f m, %d excursion(s))", outcome.c_str(), depth_,
        excursions_);
    }
    enterPhase(Phase::END_STATE);
  }

  void abort(const std::string & reason)
  {
    RCLCPP_ERROR(get_logger(), "ABORT: %s", reason.c_str());
    cancelNav();
    stopThrust();
    if (on_abort_ == "park") {
      setDepthHold(false);   // fins neutral, propeller off, hull drifts
      parked_ = true;
    }
    // "hold": depth hold stays on; the hull holds depth where it is.
    finish("aborted: " + reason);
  }

  // --------------------------------------------------------------- monitors
  /// Runs every cycle from CRUISE onward. Returns false if it aborted.
  bool monitors()
  {
    if (age(depth_time_) > sensor_timeout_) {abort("depth sensor stale"); return false;}
    if (age(imu_time_) > sensor_timeout_) {abort("IMU stale"); return false;}
    if (age(odom_time_) > sensor_timeout_) {abort("odometry stale"); return false;}
    if (depth_ > max_mission_depth_) {abort("depth beyond max_mission_depth"); return false;}
    if (std::abs(pitch_) > abort_pitch_) {abort("pitch beyond abort_pitch"); return false;}

    if (goal_) {
      if (const auto p = vehiclePosition()) {
        const double dx = goal_->pose.position.x - p->first;
        const double dy = goal_->pose.position.y - p->second;
        const double dist = std::hypot(dx, dy);
        if (dist > 1e-3) {
          const double gs = groundSpeedAlong(dx / dist, dy / dist);
          if (gs < speed_margin_) {
            abort("V - C below margin toward the goal (current now exceeds attainable headway)");
            return false;
          }
          if (energy_wh_ >= 0.0) {
            const double remaining = distance_remaining_ > 0.0 ? distance_remaining_ : dist;
            const double need_wh = hull_power_w_ * (remaining / gs) / 3600.0;
            if (energy_wh_ < need_wh * (1.0 + reserve_fraction_)) {
              abort("energy below what the remaining route needs plus reserve");
              return false;
            }
          }
        }
      }
    }
    return true;
  }

  // ------------------------------------------------------------ phase 3 plan
  void requestPlan()
  {
    // Both halves of Nav2 have to be up before the vehicle is allowed to
    // arm: planner_server for this feasibility check and bt_navigator for
    // the transit. Nav2's lifecycle bringup activates them in order, so on
    // a slow machine the planner can be ready a while before the
    // navigator; planning then arming against a half-started stack ended
    // in "Nav2 rejected the goal" at depth.
    if (!path_client_->wait_for_action_server(std::chrono::seconds(1)) ||
      !nav_client_->wait_for_action_server(std::chrono::seconds(1)))
    {
      if (sincePhase() > 60.0) {finish("no-go: Nav2 not available");}
      return;
    }
    ComputePath::Goal g;
    g.goal = *goal_;
    g.goal.header.stamp = now();
    g.use_start = false;   // plan from the vehicle's current pose
    rclcpp_action::Client<ComputePath>::SendGoalOptions o;
    o.result_callback = [this](const PathHandle::WrappedResult & r) {
        plan_pending_ = false;
        if (r.code != rclcpp_action::ResultCode::SUCCEEDED || r.result->path.poses.size() < 2) {
          finish("no-go: no drivable path to the goal");
          return;
        }
        path_ = r.result->path;
        evaluatePlan();
      };
    plan_pending_ = true;
    path_client_->async_send_goal(g, o);
    RCLCPP_INFO(get_logger(), "planning to (%.1f, %.1f)", g.goal.pose.position.x, g.goal.pose.position.y);
  }

  void evaluatePlan()
  {
    // Walk the path: V - C on every leg, and the energy the route needs.
    double route_time = 0.0, route_len = 0.0, worst_gs = 1e9;
    const auto & ps = path_->poses;
    for (size_t i = 1; i < ps.size(); ++i) {
      const double dx = ps[i].pose.position.x - ps[i - 1].pose.position.x;
      const double dy = ps[i].pose.position.y - ps[i - 1].pose.position.y;
      const double len = std::hypot(dx, dy);
      if (len < 1e-6) {continue;}
      const double gs = groundSpeedAlong(dx / len, dy / len);
      worst_gs = std::min(worst_gs, gs);
      route_len += len;
      route_time += len / std::max(gs, 1e-3);
    }
    const double need_wh = hull_power_w_ * route_time / 3600.0;
    RCLCPP_INFO(
      get_logger(),
      "plan: %.1f m, worst V - C %.2f m/s (margin %.2f), est. %.0f s, needs %.2f Wh%s",
      route_len, worst_gs, speed_margin_, route_time, need_wh,
      energy_wh_ >= 0.0 ? "" : " (no battery reading: energy check skipped)");

    if (worst_gs < speed_margin_) {
      finish("no-go: a leg has V - C below the margin; the current cannot be beaten there");
      return;
    }
    if (energy_wh_ >= 0.0 && energy_wh_ < need_wh * (1.0 + reserve_fraction_)) {
      finish("no-go: energy below route requirement plus reserve");
      return;
    }
    // First-leg heading, from the first pose ~2 m along the path.
    size_t k = 1;
    while (k + 1 < ps.size() &&
      std::hypot(ps[k].pose.position.x - ps[0].pose.position.x,
        ps[k].pose.position.y - ps[0].pose.position.y) < 2.0) {++k;}
    leg1_bearing_ = std::atan2(
      ps[k].pose.position.y - ps[0].pose.position.y,
      ps[k].pose.position.x - ps[0].pose.position.x);
    enterPhase(Phase::ARM);
  }

  // ---------------------------------------------------------- phase 8 nav2
  void sendNavGoal()
  {
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(10))) {
      abort("navigate_to_pose not available");
      return;
    }
    NavigateToPose::Goal g;
    g.pose = *goal_;
    g.pose.header.stamp = now();
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions o;
    o.goal_response_callback = [this](NavHandle::SharedPtr h) {
        if (h) {nav_handle_ = h; nav_retry_at_.reset(); return;}
        // bt_navigator answers "Action server is inactive" until its
        // lifecycle activation completes, which can land after the
        // planner's. That is transient: keep steering along leg 1 and try
        // again, and only abort once nav_reject_timeout_ has elapsed.
        if (sincePhase() < nav_reject_timeout_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Nav2 rejected the goal (navigator still activating?); retrying");
          nav_retry_at_ = now() + rclcpp::Duration::from_seconds(1.0);
        } else {
          abort("Nav2 rejected the goal");
        }
      };
    o.feedback_callback = [this](NavHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> f) {
        distance_remaining_ = f->distance_remaining;
      };
    o.result_callback = [this](const NavHandle::WrappedResult & r) {
        nav_result_ = r.code;
        nav_handle_.reset();
      };
    nav_client_->async_send_goal(g, o);
    RCLCPP_INFO(get_logger(), "transit: goal handed to Nav2");
  }

  // ------------------------------------------------------------------ step
  void step()
  {
    if (phase_ >= Phase::CRUISE && phase_ != Phase::END_STATE && !monitors()) {return;}

    switch (phase_) {
      case Phase::DEPLOY:
        setDepthHold(false);
        if (controller_seen_) {enterPhase(Phase::SENSOR_CHECK);}
        else if (sincePhase() > 30.0) {finish("no-go: depth controller never came up");}
        break;

      case Phase::SENSOR_CHECK: {
          const bool depth_ok = age(depth_time_) < sensor_timeout_;
          const bool imu_ok = age(imu_time_) < sensor_timeout_;
          const bool odom_ok = age(odom_time_) < sensor_timeout_;
          if (depth_ok && imu_ok && odom_ok) {
            RCLCPP_INFO(get_logger(), "sensors: depth %.2f m, IMU and odometry fresh", depth_);
            enterPhase(Phase::PLAN);
          } else if (sincePhase() > sensor_check_timeout_) {
            RCLCPP_ERROR_THROTTLE(
              get_logger(), *get_clock(), 10000,
              "sensor check failing: depth %s, IMU %s, odometry %s; staying inert",
              depth_ok ? "ok" : "MISSING", imu_ok ? "ok" : "MISSING", odom_ok ? "ok" : "MISSING");
          }
          break;
        }

      case Phase::PLAN:
        if (!goal_) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "waiting for a goal (RViz \"2D Goal Pose\")");
          holdStation();
        } else if (!plan_pending_ && !path_) {
          requestPlan();
        }
        break;

      case Phase::ARM:
        if (require_arm_command_ && !arm_command_) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "checks passed; waiting for arm command");
          break;
        }
        armed_ = true;
        setDepthHold(true);
        setTargetDepth(depth_);   // hold the current depth until DIVE
        enterPhase(Phase::CRUISE);
        break;

      case Phase::CRUISE:
        steerToward(leg1_bearing_, cruise_speed_);
        if (std::abs(speed_) >= min_fin_speed_) {
          RCLCPP_INFO(get_logger(), "cruise: %.2f m/s along leg 1, fins have authority", speed_);
          enterPhase(Phase::DIVE);
        } else if (sincePhase() > cruise_timeout_) {
          abort("could not reach cruise speed");
        }
        break;

      case Phase::DIVE:
        if (!target_sent_) {setTargetDepth(target_depth_); target_sent_ = true;}
        steerToward(leg1_bearing_, cruise_speed_);
        if (in_band_) {enterPhase(Phase::SETTLE);}
        else if (sincePhase() > dive_timeout_) {abort("did not reach depth");}
        break;

      case Phase::SETTLE:
        steerToward(leg1_bearing_, cruise_speed_);
        if (in_band_ && in_band_since_ && (now() - *in_band_since_).seconds() >= settle_time_) {
          RCLCPP_INFO(get_logger(), "settled at %.2f m", depth_);
          enterPhase(Phase::TRANSIT);
          sendNavGoal();
        } else if (sincePhase() > dive_timeout_) {
          abort("did not settle at depth");
        }
        break;

      case Phase::TRANSIT:
        if (nav_retry_at_) {
          steerToward(leg1_bearing_, cruise_speed_);   // keep way on while waiting
          if (now() >= *nav_retry_at_) {nav_retry_at_.reset(); sendNavGoal();}
          break;
        }
        if (nav_result_) {
          if (*nav_result_ == rclcpp_action::ResultCode::SUCCEEDED) {
            nav_result_.reset();
            if (enable_delivery_) {
              target_sent_ = false;
              enterPhase(Phase::DELIVERY_DEPTH);
            } else {
              finish("reached the goal");
            }
          } else {
            abort("Nav2 did not reach the goal");
          }
        }
        break;

      case Phase::DELIVERY_DEPTH:
        if (!target_sent_) {setTargetDepth(delivery_depth_); target_sent_ = true;}
        if (in_band_ && in_band_since_ && (now() - *in_band_since_).seconds() >= settle_time_) {
          enterPhase(Phase::TERMINAL);
        } else if (sincePhase() > dive_timeout_) {
          abort("did not settle at delivery depth");
        }
        break;

      case Phase::TERMINAL: {
          const auto p = vehiclePosition();
          const auto yaw = vehicleYaw();
          if (!p || !yaw) {break;}
          const double dx = goal_->pose.position.x - p->first;
          const double dy = goal_->pose.position.y - p->second;
          const double range = std::hypot(dx, dy);
          const double bearing = std::isnan(approach_bearing_) ? std::atan2(dy, dx) : approach_bearing_;
          const bool aligned = std::abs(wrapPi(bearing - *yaw)) <= align_tolerance_;
          if (range <= contact_range_ && aligned) {
            stopThrust();
            enterPhase(Phase::DELIVER);
          } else if (sincePhase() > terminal_timeout_) {
            abort("terminal approach timed out");
          } else {
            steerToward(bearing, range <= contact_range_ ? 0.0 : terminal_speed_);
          }
          break;
        }

      case Phase::DELIVER:
        if (!release_sent_) {
          std_msgs::msg::Bool r;
          r.data = true;
          release_pub_->publish(r);
          release_sent_ = true;
          RCLCPP_INFO(get_logger(), "release commanded");
        }
        if (release_confirmed_) {
          std_msgs::msg::Bool z;
          z.data = true;
          reset_pub_->publish(z);   // trim changed with the lost mass
          finish("delivered");
        } else if (sincePhase() > release_timeout_) {
          abort("release not confirmed");
        }
        break;

      case Phase::END_STATE:
        if (!parked_) {holdStation();}   // new goal -> PLAN, new mission_id -> DEPLOY
        break;
    }
  }

  // ---- parameters -------------------------------------------------------------------
  double target_depth_, settle_time_, sensor_timeout_, sensor_check_timeout_;
  bool require_arm_command_;
  std::string on_abort_;
  double max_speed_, current_speed_, current_dir_deg_, speed_margin_;
  double hull_power_w_, reserve_fraction_, energy_wh_;
  double cruise_speed_, min_fin_speed_, cruise_timeout_, dive_timeout_, min_turning_radius_, kp_yaw_;
  bool station_keeping_;
  double station_radius_, station_inner_radius_;
  double max_mission_depth_, abort_pitch_;
  bool enable_delivery_;
  double delivery_depth_, terminal_speed_, contact_range_, align_tolerance_, approach_bearing_;
  double terminal_timeout_, release_timeout_;

  // ---- state ----------------------------------------------------------------------------
  Phase phase_{Phase::DEPLOY};
  rclcpp::Time phase_since_;
  std::string mission_id_;
  bool controller_seen_{false}, armed_{false}, arm_command_{false}, parked_{false};
  std::optional<std::pair<double, double>> station_;   // where the current wait began
  bool returning_{false}, station_astern_{false};
  int shuttle_dir_{0};   // +1 ahead, -1 astern
  double sway_{0.0};     // m/s, body y from odometry
  static constexpr double kStationSpeed = 0.5;   // m/s either way: the velocity smoother's reverse limit
  static constexpr double kSwayGain = 1.0;       // rad/s of yaw per m/s of sway
  std::string depth_state_{"?"};
  double depth_{0.0}, pitch_{0.0}, speed_{0.0};
  std::optional<rclcpp::Time> depth_time_, imu_time_, odom_time_, in_band_since_;
  bool in_band_{false};
  int excursions_{0};
  std::optional<geometry_msgs::msg::PoseStamped> goal_;
  std::optional<nav_msgs::msg::Path> path_;
  bool plan_pending_{false};
  double leg1_bearing_{0.0};
  bool target_sent_{false};
  NavHandle::SharedPtr nav_handle_;
  std::optional<rclcpp_action::ResultCode> nav_result_;
  std::optional<rclcpp::Time> nav_retry_at_;   // set while a rejected goal is being retried
  double nav_reject_timeout_;
  double distance_remaining_{-1.0};
  bool release_sent_{false}, release_confirmed_{false}, outcome_sent_{false};

  // ---- ROS ------------------------------------------------------------------------------
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_, reset_pub_, release_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr outcome_pub_, phase_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_, mission_id_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr in_band_sub_, arm_sub_, release_confirm_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depth_sub_, energy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<ComputePath>::SharedPtr path_client_;
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
