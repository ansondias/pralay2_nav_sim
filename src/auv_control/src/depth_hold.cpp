// Depth-hold controller for the Pralay-2: dive to a target depth and stay there.
//
// C++ port of auv_control/depth_hold.py (kept alongside as `depth_hold_py`
// for reference). Same parameters, same topics, same control law -- this is
// the version the launch file starts, since the real vehicle runs this loop
// at a fixed rate next to the ros2_control hardware and a compiled node has
// no interpreter jitter or GIL to share with the rest of the stack.
//
// Hardware this is written for
//   * Blue Robotics Bar30 depth/pressure sensor (MS5837). Publishes either
//     sensor_msgs/FluidPressure (Pa) or a ready-made depth in metres,
//     selected with the `depth_source` parameter.
//   * Bosch BNO086 IMU as sensor_msgs/Imu: orientation quaternion (pitch)
//     and angular_velocity.y (pitch rate).
//   * Four servo fins, +-45 deg. The LEFT and RIGHT fins are the elevator
//     pair (pitch = up/down); the TOP and BOTTOM fins are the rudder pair
//     (left/right). This node produces only the elevator angle; it is
//     published on /depth_hold/elevator and mixed into the four-fin command
//     by cmd_vel_to_actuators, which keeps steering (rudder) coming from
//     cmd_vel. So the same fin_controller and the same wiring serve both.
//
// What it does
//   1. DIVING    : from the surface, pitch the nose down (limited to
//                  `max_pitch`) until the depth first enters the tolerance
//                  band around the target.
//   2. HOLDING   : regulate depth to the target with a cascaded loop --
//                  depth error -> desired pitch -> elevator angle -- and
//                  report `in_band` while depth stays within
//                  [target - tolerance, target + tolerance] (default
//                  1.0 m +- 0.2 m, i.e. 0.8 .. 1.2 m).
//   3. SURFACING : if `surface_on_timeout` and the depth sensor goes silent,
//                  fins go to full nose-up so the real AUV comes up.
//
// Control law (every sign is also a parameter, so it can be flipped on the
// AUV without touching code):
//   depth_error   = target_depth - depth               (m, +ve: too shallow)
//   desired_pitch = clamp(Kp_d*e + Ki_d*int(e) - Kd_d*depth_rate, +-max_pitch)
//                   (+ve = nose DOWN in the ROS FLU body frame)
//   pitch_error   = desired_pitch - pitch
//   elevator      = elevator_sign * clamp(-(Kp_p*pitch_error - Kd_p*pitch_rate),
//                                         +-max_elevator)
//                   (elevator +ve = trailing edge up = nose UP, matching
//                    cmd_vel_to_actuators; hence the leading minus)
// Fins only bite with water flowing over them: with `speed_scaling` on, the
// elevator command is scaled by (reference_speed / speed)^2 (clamped) from
// the surge speed in odometry, so the loop stays stable across speeds.
//
// Gains, limits, signs and the target are all settable live with
// `ros2 param set /depth_hold <name> <value>` -- pool tuning without a
// restart. Everything else needs a relaunch.
//
// Run:  ros2 launch auv_control depth_hold.launch.py   (config/depth_hold.yaml)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/quaternion.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

namespace auv_control
{

/// Pitch (rad) of a body-frame quaternion, ROS FLU convention (+ = nose down).
double quaternionToPitch(const geometry_msgs::msg::Quaternion & q)
{
  const double sinp = std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0);
  return std::asin(sinp);
}

/// Symmetric clamp to +-limit.
double clampSym(double value, double limit)
{
  return std::clamp(value, -limit, limit);
}

class DepthHold : public rclcpp::Node
{
public:
  DepthHold()
  : Node("depth_hold")
  {
    // ---- target and tolerance ---------------------------------------------
    target_ = declare_parameter("target_depth", 1.0);        // m below surface
    tolerance_ = declare_parameter("tolerance", 0.2);        // m: 0.8 .. 1.2 in band
    enabled_ = declare_parameter("enabled", true);

    // ---- sensors ------------------------------------------------------------
    const std::string depth_source = declare_parameter("depth_source", std::string("pressure"));
    const std::string pressure_topic =
      declare_parameter("pressure_topic", std::string("/bar30/pressure"));
    const std::string depth_topic = declare_parameter("depth_topic", std::string("/bar30/depth"));
    const std::string imu_topic = declare_parameter("imu_topic", std::string("/imu/data"));
    const std::string odom_topic =
      declare_parameter("odom_topic", std::string("/odometry/filtered"));
    rho_ = declare_parameter("fluid_density", 1025.0);       // kg/m^3 (997 fresh water)
    g_ = declare_parameter("gravity", 9.80665);
    // Surface (atmospheric) pressure. auto_zero averages the first
    // `auto_zero_samples` readings on start-up (the AUV must be at the
    // surface then); otherwise surface_pressure_pa is used as-is.
    surface_pressure_ = declare_parameter("surface_pressure_pa", 101325.0);
    auto_zero_ = declare_parameter("auto_zero", true);
    auto_zero_samples_ = declare_parameter("auto_zero_samples", 20);
    alpha_ = declare_parameter("depth_filter_alpha", 0.3);   // low-pass on depth, 0..1
    sensor_timeout_ = declare_parameter("sensor_timeout", 1.0);  // s -> failsafe
    // IMU freshness. Without this a frozen IMU was trusted forever: the
    // inner loop kept closing on a stale pitch instead of falling back to
    // the depth-only path below.
    imu_timeout_ = declare_parameter("imu_timeout", 1.0);
    surface_on_timeout_ = declare_parameter("surface_on_timeout", true);

    // ---- outer loop: depth -> desired pitch -----------------------------------
    kp_d_ = declare_parameter("kp_depth", 0.6);      // rad of pitch per m of error
    ki_d_ = declare_parameter("ki_depth", 0.05);     // rad per m*s
    kd_d_ = declare_parameter("kd_depth", 0.8);      // rad per (m/s) of depth rate
    max_pitch_ = declare_parameter("max_pitch", 0.35);        // rad (~20 deg)
    integrator_band_ = declare_parameter("integrator_band", 0.5);  // m

    // ---- inner loop: pitch -> elevator ----------------------------------------
    kp_p_ = declare_parameter("kp_pitch", 2.0);      // rad elevator per rad pitch error
    kd_p_ = declare_parameter("kd_pitch", 0.4);      // rad elevator per (rad/s)
    max_elevator_ = declare_parameter("max_elevator", 0.70);  // rad (~40 deg), servo stops at 45
    elevator_sign_ = declare_parameter("elevator_sign", 1.0);
    pitch_sign_ = declare_parameter("pitch_sign", 1.0);

    // ---- speed scaling (fin authority ~ v^2) -----------------------------------
    speed_scaling_ = declare_parameter("speed_scaling", true);
    reference_speed_ = declare_parameter("reference_speed", 1.0);
    min_speed_for_scaling_ = declare_parameter("min_speed_for_scaling", 0.3);
    max_scale_ = declare_parameter("max_scale", 3.0);

    // ---- misc --------------------------------------------------------------------
    const double control_rate = declare_parameter("control_rate", 20.0);   // Hz
    sim_mode_ = declare_parameter("sim_mode", false);

    state_ = enabled_ ? "DIVING" : "DISABLED";

    // ---- ROS I/O ------------------------------------------------------------------
    if (depth_source == "pressure") {
      pressure_sub_ = create_subscription<sensor_msgs::msg::FluidPressure>(
        pressure_topic, 10, std::bind(&DepthHold::onPressure, this, std::placeholders::_1));
    } else {
      depth_sub_ = create_subscription<std_msgs::msg::Float64>(
        depth_topic, 10, std::bind(&DepthHold::onDepthMsg, this, std::placeholders::_1));
    }
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, 20, std::bind(&DepthHold::onImu, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10, std::bind(&DepthHold::onOdom, this, std::placeholders::_1));
    target_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/depth_hold/target_depth", 10, std::bind(&DepthHold::onTarget, this, std::placeholders::_1));
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/depth_hold/enable", 10, std::bind(&DepthHold::onEnable, this, std::placeholders::_1));
    // Zero the depth integrator on demand: the mission sequencer sends this
    // after a payload release (trim changed) and on a per-mission reset.
    reset_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/depth_hold/reset_integrator", 10,
      [this](const std_msgs::msg::Bool::SharedPtr m) {if (m->data) {integral_ = 0.0;}});

    elevator_pub_ = create_publisher<std_msgs::msg::Float64>("/depth_hold/elevator", 10);
    depth_pub_ = create_publisher<std_msgs::msg::Float64>("/depth_hold/depth", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/depth_hold/state", 10);
    in_band_pub_ = create_publisher<std_msgs::msg::Bool>("/depth_hold/in_band", 10);
    if (sim_mode_) {
      // The simulated fins generate no lift (the hull is moved by
      // auv_current_plugin::ThrustControl), so under Gazebo the target is
      // also handed to that plugin's own depth hold.
      setpoint_pub_ = create_publisher<std_msgs::msg::Float64>("/depth_setpoint", 10);
    }

    dt_ = 1.0 / control_rate;
    // Node clock, not wall clock, so use_sim_time works under Gazebo.
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(dt_),
      std::bind(&DepthHold::controlStep, this));

    params_cb_ = add_on_set_parameters_callback(
      std::bind(&DepthHold::onParameterChange, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "depth hold: target %.2f m, band +-%.2f m, max pitch %.0f deg, max elevator %.0f deg",
      target_, tolerance_, max_pitch_ * 180.0 / M_PI, max_elevator_ * 180.0 / M_PI);
  }

private:
  // ------------------------------------------------------------------- inputs
  void onPressure(const sensor_msgs::msg::FluidPressure::SharedPtr msg)
  {
    const double pressure = msg->fluid_pressure;
    if (auto_zero_ && static_cast<int>(zero_samples_.size()) < auto_zero_samples_) {
      zero_samples_.push_back(pressure);
      if (static_cast<int>(zero_samples_.size()) == auto_zero_samples_) {
        double sum = 0.0;
        for (const double p : zero_samples_) {
          sum += p;
        }
        surface_pressure_ = sum / zero_samples_.size();
        RCLCPP_INFO(get_logger(), "surface pressure zeroed at %.0f Pa", surface_pressure_);
      }
      return;
    }
    updateDepth((pressure - surface_pressure_) / (rho_ * g_));
  }

  void onDepthMsg(const std_msgs::msg::Float64::SharedPtr msg) {updateDepth(msg->data);}

  void updateDepth(double depth_raw)
  {
    const rclcpp::Time now = this->now();
    if (!depth_) {
      depth_ = depth_raw;
    } else {
      depth_ = alpha_ * depth_raw + (1.0 - alpha_) * *depth_;
    }
    if (last_depth_time_ && depth_prev_) {
      const double dt = (now - *last_depth_time_).seconds();
      if (dt > 1e-3) {
        const double rate = (*depth_ - *depth_prev_) / dt;
        depth_rate_ = 0.5 * rate + 0.5 * depth_rate_;   // light smoothing
      }
    }
    depth_prev_ = depth_;
    last_depth_time_ = now;
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    pitch_ = pitch_sign_ * quaternionToPitch(msg->orientation);
    pitch_rate_ = pitch_sign_ * msg->angular_velocity.y;
    last_imu_time_ = this->now();
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {speed_ = msg->twist.twist.linear.x;}

  void onTarget(const std_msgs::msg::Float64::SharedPtr msg)
  {
    target_ = std::max(0.0, msg->data);
    integral_ = 0.0;
    reached_band_once_ = false;
    if (enabled_) {
      state_ = "DIVING";
    }
    RCLCPP_INFO(get_logger(), "new target depth %.2f m", target_);
  }

  void onEnable(const std_msgs::msg::Bool::SharedPtr msg)
  {
    enabled_ = msg->data;
    integral_ = 0.0;
    state_ = enabled_ ? "DIVING" : "DISABLED";
  }

  /// Live re-tuning: gains, limits, signs, target and enable only. Anything
  /// structural (topics, rate, depth source) still needs a relaunch.
  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & p : params) {
      const std::string & n = p.get_name();
      if (n == "target_depth") {
        target_ = std::max(0.0, p.as_double());
        integral_ = 0.0;
        reached_band_once_ = false;
      } else if (n == "tolerance") {tolerance_ = p.as_double();
      } else if (n == "enabled") {enabled_ = p.as_bool(); integral_ = 0.0;
      } else if (n == "kp_depth") {kp_d_ = p.as_double();
      } else if (n == "ki_depth") {ki_d_ = p.as_double(); integral_ = 0.0;
      } else if (n == "kd_depth") {kd_d_ = p.as_double();
      } else if (n == "max_pitch") {max_pitch_ = p.as_double();
      } else if (n == "integrator_band") {integrator_band_ = p.as_double();
      } else if (n == "kp_pitch") {kp_p_ = p.as_double();
      } else if (n == "kd_pitch") {kd_p_ = p.as_double();
      } else if (n == "max_elevator") {max_elevator_ = p.as_double();
      } else if (n == "elevator_sign") {elevator_sign_ = p.as_double();
      } else if (n == "pitch_sign") {pitch_sign_ = p.as_double();
      } else if (n == "speed_scaling") {speed_scaling_ = p.as_bool();
      } else if (n == "reference_speed") {reference_speed_ = p.as_double();
      } else if (n == "min_speed_for_scaling") {min_speed_for_scaling_ = p.as_double();
      } else if (n == "max_scale") {max_scale_ = p.as_double();
      } else if (n == "surface_on_timeout") {surface_on_timeout_ = p.as_bool();
      } else if (n == "sensor_timeout") {sensor_timeout_ = p.as_double();
      } else if (n == "imu_timeout") {imu_timeout_ = p.as_double();
      } else if (n == "fluid_density") {rho_ = p.as_double();
      } else if (n == "depth_filter_alpha") {alpha_ = p.as_double();
      }
    }
    return result;
  }

  // ------------------------------------------------------------------ control
  void controlStep()
  {
    if (!enabled_) {
      publish(0.0, "DISABLED", false);
      return;
    }
    if (setpoint_pub_) {
      std_msgs::msg::Float64 sp;
      sp.data = target_;
      setpoint_pub_->publish(sp);
    }

    const bool sensor_ok = depth_ && last_depth_time_ &&
      (this->now() - *last_depth_time_).seconds() < sensor_timeout_;
    if (!sensor_ok) {
      if (surface_on_timeout_ && depth_) {
        // Lost the depth sensor after having had it: full nose-up, surface.
        publish(elevator_sign_ * max_elevator_, "SURFACING", false);
      } else {
        publish(0.0, "DIVING", false);   // no data yet: fins neutral
      }
      return;
    }

    const double error = target_ - *depth_;      // +ve: too shallow, go down
    const bool in_band = std::abs(error) <= tolerance_;
    if (in_band) {
      reached_band_once_ = true;
    }
    state_ = reached_band_once_ ? "HOLDING" : "DIVING";

    // Outer loop: depth error -> desired pitch (+ nose down). The integrator
    // only runs inside integrator_band, so it cannot wind up during the dive,
    // and is clamped so it alone can never demand more than max_pitch.
    if (std::abs(error) <= integrator_band_) {
      integral_ = clampSym(integral_ + error * dt_, max_pitch_ / std::max(ki_d_, 1e-6));
    } else {
      integral_ = 0.0;
    }
    const double desired_pitch = clampSym(
      kp_d_ * error + ki_d_ * integral_ - kd_d_ * depth_rate_, max_pitch_);

    // Inner loop: pitch error -> elevator. Positive elevator = nose UP, so a
    // nose-down demand needs a negative elevator: hence the leading minus.
    // IMU counts only while fresh; a stale one falls through to depth-only.
    const bool have_imu = last_imu_time_ &&
      (this->now() - *last_imu_time_).seconds() < imu_timeout_;
    if (!have_imu && last_imu_time_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "IMU stale, driving elevator from depth loop only");
    }
    double elevator;
    if (have_imu) {
      const double pitch_error = desired_pitch - pitch_;
      elevator = -(kp_p_ * pitch_error - kd_p_ * pitch_rate_);
    } else {
      // No IMU: drive the elevator straight from the depth loop.
      elevator = -desired_pitch * kp_p_;
    }

    if (speed_scaling_ && speed_) {
      const double v = std::max(std::abs(*speed_), min_speed_for_scaling_);
      elevator *= std::min(max_scale_, std::pow(reference_speed_ / v, 2.0));
    }

    publish(elevator_sign_ * clampSym(elevator, max_elevator_), state_, in_band);
  }

  void publish(double elevator, const std::string & state, bool in_band)
  {
    std_msgs::msg::Float64 e;
    e.data = elevator;
    elevator_pub_->publish(e);

    std_msgs::msg::String s;
    s.data = state;
    state_pub_->publish(s);

    std_msgs::msg::Bool b;
    b.data = in_band;
    in_band_pub_->publish(b);

    if (depth_) {
      std_msgs::msg::Float64 d;
      d.data = *depth_;
      depth_pub_->publish(d);
    }
  }

  // ---- parameters (all live-tunable ones are non-const) ----------------------
  double target_, tolerance_;
  bool enabled_;
  double rho_, g_, surface_pressure_, alpha_, sensor_timeout_;
  bool auto_zero_, surface_on_timeout_;
  int auto_zero_samples_;
  double kp_d_, ki_d_, kd_d_, max_pitch_, integrator_band_;
  double kp_p_, kd_p_, max_elevator_, elevator_sign_, pitch_sign_;
  bool speed_scaling_;
  double reference_speed_, min_speed_for_scaling_, max_scale_;
  bool sim_mode_;
  double dt_;

  // ---- state -------------------------------------------------------------------
  std::optional<double> depth_;              // filtered, m (+ down)
  std::optional<double> depth_prev_;
  double depth_rate_{0.0};                   // m/s (+ = going deeper)
  std::optional<rclcpp::Time> last_depth_time_;
  std::vector<double> zero_samples_;
  double pitch_{0.0};                        // rad (+ nose down)
  double pitch_rate_{0.0};                   // rad/s
  std::optional<rclcpp::Time> last_imu_time_;
  double imu_timeout_{1.0};
  std::optional<double> speed_;              // m/s surge
  double integral_{0.0};
  std::string state_;
  bool reached_band_once_{false};

  // ---- ROS -----------------------------------------------------------------------
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr elevator_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr depth_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr in_band_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr setpoint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_cb_;
};

}  // namespace auv_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<auv_control::DepthHold>());
  rclcpp::shutdown();
  return 0;
}
