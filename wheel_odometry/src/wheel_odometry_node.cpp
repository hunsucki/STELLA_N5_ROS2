#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

namespace
{
double angle_diff(double target, double source)
{
  return std::atan2(std::sin(target - source), std::cos(target - source));
}

bool quaternion_to_yaw(const geometry_msgs::msg::Quaternion & q, double & yaw)
{
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!std::isfinite(norm) || norm < 1.0e-9)
  {
    return false;
  }

  const double x = q.x / norm;
  const double y = q.y / norm;
  const double z = q.z / norm;
  const double w = q.w / norm;
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  yaw = std::atan2(siny_cosp, cosy_cosp);
  return std::isfinite(yaw);
}

struct ImuYawSample
{
  rclcpp::Time stamp;
  double yaw;
};
}  // namespace

class WheelOdometryNode : public rclcpp::Node
{
public:
  WheelOdometryNode()
  : Node("wheel_odometry")
  {
    enabled_ = declare_parameter<bool>("enabled", true);
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.0875);
    wheel_separation_ = declare_parameter<double>("wheel_separation", 0.36);
    wheel_topic_ = declare_parameter<std::string>("wheel_topic", "/wheel/encoders");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    left_wheel_name_ = declare_parameter<std::string>("left_wheel_name", "left_wheel");
    right_wheel_name_ = declare_parameter<std::string>("right_wheel_name", "right_wheel");
    use_imu_orientation_ = declare_parameter<bool>("use_imu_orientation", true);
    relative_imu_yaw_ = declare_parameter<bool>("relative_imu_yaw", false);
    imu_timeout_sec_ = declare_parameter<double>("imu_timeout_sec", 0.25);
    imu_history_duration_sec_ =
      declare_parameter<double>("imu_history_duration_sec", 1.0);
    imu_correction_time_constant_sec_ =
      declare_parameter<double>("imu_correction_time_constant_sec", 0.5);
    max_imu_correction_rate_rad_s_ =
      declare_parameter<double>("max_imu_correction_rate_rad_s", 1.0);
    max_encoder_interval_sec_ =
      declare_parameter<double>("max_encoder_interval_sec", 0.5);
    max_wheel_speed_m_s_ = declare_parameter<double>("max_wheel_speed_m_s", 2.0);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    publish_yaw_diagnostics_ = declare_parameter<bool>("publish_yaw_diagnostics", true);

    pose_xy_variance_ = declare_parameter<double>("pose_xy_variance", 0.0025);
    pose_yaw_variance_ = declare_parameter<double>("pose_yaw_variance", 0.00121847);
    twist_linear_variance_ = declare_parameter<double>("twist_linear_variance", 0.01);
    twist_angular_variance_ = declare_parameter<double>("twist_angular_variance", 0.01);

    if (wheel_radius_ <= 0.0 || wheel_separation_ <= 0.0)
    {
      throw std::invalid_argument("wheel_radius and wheel_separation must be positive");
    }
    if (
      imu_timeout_sec_ < 0.0 || imu_history_duration_sec_ <= 0.0 ||
      imu_correction_time_constant_sec_ < 0.0 || max_imu_correction_rate_rad_s_ < 0.0 ||
      max_encoder_interval_sec_ < 0.0 || max_wheel_speed_m_s_ < 0.0)
    {
      throw std::invalid_argument(
              "Timeouts, rate limits and maximum wheel speed must be valid non-negative values");
    }

    yaw_diagnostics_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "wheel_odometry/yaw_diagnostics", rclcpp::QoS(10));
    if (enabled_)
    {
      create_output_publishers();
    }

    wheel_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      wheel_topic_, rclcpp::QoS(10),
      std::bind(&WheelOdometryNode::wheel_callback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::QoS(10),
      std::bind(&WheelOdometryNode::imu_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "wheel_odometry %s: wheel_radius=%.4f m, separation=%.4f m, wheel=%s, imu=%s, IMU correction tau=%.3f s",
      enabled_ ? "enabled" : "disabled", wheel_radius_, wheel_separation_,
      wheel_topic_.c_str(), imu_topic_.c_str(), imu_correction_time_constant_sec_);
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring unstamped IMU message");
      return;
    }

    double yaw = 0.0;
    if (!quaternion_to_yaw(msg->orientation, yaw))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring invalid IMU orientation");
      return;
    }

    const rclcpp::Time stamp(msg->header.stamp);
    if (!imu_yaw_history_.empty())
    {
      if (stamp < imu_yaw_history_.back().stamp)
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Ignoring out-of-order IMU timestamp");
        return;
      }
      if (stamp == imu_yaw_history_.back().stamp)
      {
        imu_yaw_history_.back().yaw = yaw;
        return;
      }
    }

    imu_yaw_history_.push_back({stamp, yaw});
    while (
      imu_yaw_history_.size() > 2 &&
      (stamp - imu_yaw_history_[1].stamp).seconds() > imu_history_duration_sec_)
    {
      imu_yaw_history_.pop_front();
    }
  }

  bool wheel_positions(
    const sensor_msgs::msg::JointState & msg, double & left, double & right) const
  {
    if (msg.name.size() != msg.position.size())
    {
      return false;
    }

    bool found_left = false;
    bool found_right = false;
    for (std::size_t index = 0; index < msg.name.size(); ++index)
    {
      if (msg.name[index] == left_wheel_name_)
      {
        left = msg.position[index];
        found_left = true;
      }
      else if (msg.name[index] == right_wheel_name_)
      {
        right = msg.position[index];
        found_right = true;
      }
    }
    return found_left && found_right && std::isfinite(left) && std::isfinite(right);
  }

  bool imu_yaw_at(
    const rclcpp::Time & stamp, double & yaw, rclcpp::Time & measurement_stamp)
  {
    if (!use_imu_orientation_)
    {
      return false;
    }
    if (imu_yaw_history_.empty())
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for stamped IMU orientation");
      return false;
    }

    const auto after = std::upper_bound(
      imu_yaw_history_.begin(), imu_yaw_history_.end(), stamp,
      [](const rclcpp::Time & target, const ImuYawSample & sample) {
        return target < sample.stamp;
      });

    // Never use a future-only sample.  If samples bracket the encoder time,
    // interpolate across the shortest yaw arc.  Otherwise use the newest
    // causal sample within the configured timeout.
    if (after == imu_yaw_history_.begin())
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for an IMU sample at or before the encoder timestamp");
      return false;
    }

    const auto before = std::prev(after);
    const double past_age = (stamp - before->stamp).seconds();
    if (imu_timeout_sec_ > 0.0 && past_age > imu_timeout_sec_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "IMU orientation is stale relative to encoder sample (%.3f sec)", past_age);
      return false;
    }

    double raw_yaw = before->yaw;
    measurement_stamp = before->stamp;
    if (after != imu_yaw_history_.end())
    {
      const double future_age = (after->stamp - stamp).seconds();
      const double sample_interval = (after->stamp - before->stamp).seconds();
      const bool future_is_fresh = imu_timeout_sec_ <= 0.0 || future_age <= imu_timeout_sec_;
      if (sample_interval > 0.0 && future_is_fresh)
      {
        const double ratio = std::clamp(
          (stamp - before->stamp).seconds() / sample_interval, 0.0, 1.0);
        raw_yaw = std::atan2(
          std::sin(before->yaw + ratio * angle_diff(after->yaw, before->yaw)),
          std::cos(before->yaw + ratio * angle_diff(after->yaw, before->yaw)));
        measurement_stamp = stamp;
      }
    }

    if (relative_imu_yaw_)
    {
      if (!imu_reference_initialized_)
      {
        initial_imu_yaw_ = raw_yaw;
        imu_reference_initialized_ = true;
      }
      yaw = angle_diff(raw_yaw, initial_imu_yaw_);
    }
    else
    {
      yaw = raw_yaw;
    }
    return true;
  }

  void wheel_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const bool requested_enabled = get_parameter("enabled").as_bool();
    if (requested_enabled != enabled_)
    {
      enabled_ = requested_enabled;
      encoder_initialized_ = false;
      imu_reference_initialized_ = false;
      last_used_imu_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      if (enabled_)
      {
        create_output_publishers();
      }
      else
      {
        odom_pub_.reset();
        tf_broadcaster_.reset();
      }
      RCLCPP_WARN(get_logger(), "wheel_odometry %s", enabled_ ? "enabled" : "disabled");
    }
    if (!enabled_)
    {
      return;
    }

    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring unstamped wheel encoder message");
      return;
    }

    double left_position = 0.0;
    double right_position = 0.0;
    if (!wheel_positions(*msg, left_position, right_position))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Wheel message must contain %s and %s positions",
        left_wheel_name_.c_str(), right_wheel_name_.c_str());
      return;
    }

    const rclcpp::Time stamp(msg->header.stamp);
    double imu_yaw = 0.0;
    rclcpp::Time imu_measurement_stamp(0, 0, stamp.get_clock_type());
    const bool have_imu_yaw = imu_yaw_at(stamp, imu_yaw, imu_measurement_stamp);

    if (!encoder_initialized_)
    {
      previous_left_position_ = left_position;
      previous_right_position_ = right_position;
      previous_encoder_stamp_ = stamp;
      if (use_imu_orientation_ && !have_imu_yaw)
      {
        return;
      }
      yaw_ = use_imu_orientation_ ? imu_yaw : 0.0;
      wheel_yaw_ = yaw_;
      previous_yaw_ = yaw_;
      if (have_imu_yaw)
      {
        last_used_imu_stamp_ = imu_measurement_stamp;
      }
      encoder_initialized_ = true;
      publish_odometry(stamp, 0.0, 0.0);
      publish_yaw_diagnostics(
        stamp, have_imu_yaw ? imu_yaw : std::numeric_limits<double>::quiet_NaN());
      return;
    }

    const double dt = (stamp - previous_encoder_stamp_).seconds();
    if (dt <= 0.0)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Non-increasing encoder timestamp");
      return;
    }
    if (max_encoder_interval_sec_ > 0.0 && dt > max_encoder_interval_sec_)
    {
      RCLCPP_WARN(
        get_logger(), "Encoder interval %.3f sec exceeds %.3f sec; resetting baseline",
        dt, max_encoder_interval_sec_);
      previous_left_position_ = left_position;
      previous_right_position_ = right_position;
      previous_encoder_stamp_ = stamp;
      return;
    }

    const double left_distance = (left_position - previous_left_position_) * wheel_radius_;
    const double right_distance = (right_position - previous_right_position_) * wheel_radius_;
    const double left_speed = left_distance / dt;
    const double right_speed = right_distance / dt;
    if (
      !std::isfinite(left_speed) || !std::isfinite(right_speed) ||
      (max_wheel_speed_m_s_ > 0.0 &&
      (std::abs(left_speed) > max_wheel_speed_m_s_ ||
      std::abs(right_speed) > max_wheel_speed_m_s_)))
    {
      RCLCPP_WARN(
        get_logger(),
        "Rejecting implausible encoder change: left=%.3f m/s right=%.3f m/s; resetting baseline",
        left_speed, right_speed);
      previous_left_position_ = left_position;
      previous_right_position_ = right_position;
      previous_encoder_stamp_ = stamp;
      return;
    }

    const double center_distance = 0.5 * (left_distance + right_distance);
    const double wheel_yaw_change = (right_distance - left_distance) / wheel_separation_;
    wheel_yaw_ = std::atan2(
      std::sin(wheel_yaw_ + wheel_yaw_change),
      std::cos(wheel_yaw_ + wheel_yaw_change));
    const double predicted_yaw = std::atan2(
      std::sin(yaw_ + wheel_yaw_change),
      std::cos(yaw_ + wheel_yaw_change));

    const bool new_imu_measurement =
      have_imu_yaw && imu_measurement_stamp > last_used_imu_stamp_;
    if (use_imu_orientation_ && new_imu_measurement)
    {
      const double correction_alpha = imu_correction_time_constant_sec_ > 0.0
        ? 1.0 - std::exp(-dt / imu_correction_time_constant_sec_)
        : 1.0;
      double imu_correction = correction_alpha * angle_diff(imu_yaw, predicted_yaw);
      if (max_imu_correction_rate_rad_s_ > 0.0)
      {
        const double max_correction = max_imu_correction_rate_rad_s_ * dt;
        imu_correction = std::max(
          -max_correction, std::min(max_correction, imu_correction));
      }
      yaw_ = std::atan2(
        std::sin(predicted_yaw + imu_correction),
        std::cos(predicted_yaw + imu_correction));
      last_used_imu_stamp_ = imu_measurement_stamp;
    }
    else
    {
      yaw_ = predicted_yaw;
    }

    const double yaw_change = angle_diff(yaw_, previous_yaw_);
    const double integration_yaw = previous_yaw_ + 0.5 * yaw_change;
    x_ += center_distance * std::cos(integration_yaw);
    y_ += center_distance * std::sin(integration_yaw);

    const double linear_velocity = center_distance / dt;
    // Pose yaw includes a slow IMU drift correction.  Report measured wheel
    // angular velocity instead so a heading correction is not presented to
    // Nav2 as physical robot rotation.
    const double angular_velocity = wheel_yaw_change / dt;
    publish_odometry(stamp, linear_velocity, angular_velocity);
    publish_yaw_diagnostics(
      stamp, have_imu_yaw ? imu_yaw : std::numeric_limits<double>::quiet_NaN());

    previous_left_position_ = left_position;
    previous_right_position_ = right_position;
    previous_encoder_stamp_ = stamp;
    previous_yaw_ = yaw_;
  }

  void publish_odometry(
    const rclcpp::Time & stamp, double linear_velocity, double angular_velocity)
  {
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw_);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.x = orientation.x();
    odom.pose.pose.orientation.y = orientation.y();
    odom.pose.pose.orientation.z = orientation.z();
    odom.pose.pose.orientation.w = orientation.w();
    odom.twist.twist.linear.x = linear_velocity;
    odom.twist.twist.angular.z = angular_velocity;

    odom.pose.covariance.fill(0.0);
    odom.pose.covariance[0] = pose_xy_variance_;
    odom.pose.covariance[7] = pose_xy_variance_;
    odom.pose.covariance[14] = 1.0e6;
    odom.pose.covariance[21] = 1.0e6;
    odom.pose.covariance[28] = 1.0e6;
    odom.pose.covariance[35] = pose_yaw_variance_;

    odom.twist.covariance.fill(0.0);
    odom.twist.covariance[0] = twist_linear_variance_;
    odom.twist.covariance[7] = 1.0e6;
    odom.twist.covariance[14] = 1.0e6;
    odom.twist.covariance[21] = 1.0e6;
    odom.twist.covariance[28] = 1.0e6;
    odom.twist.covariance[35] = twist_angular_variance_;

    odom_pub_->publish(odom);

    if (publish_tf_)
    {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odom.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = x_;
      transform.transform.translation.y = y_;
      transform.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  void create_output_publishers()
  {
    if (!odom_pub_)
    {
      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(10));
    }
    if (publish_tf_ && !tf_broadcaster_)
    {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
  }

  void publish_yaw_diagnostics(const rclcpp::Time & stamp, double imu_yaw)
  {
    if (!publish_yaw_diagnostics_)
    {
      return;
    }

    geometry_msgs::msg::Vector3Stamped diagnostics;
    diagnostics.header.stamp = stamp;
    diagnostics.header.frame_id = odom_frame_;
    diagnostics.vector.x = wheel_yaw_;
    diagnostics.vector.y = imu_yaw;
    diagnostics.vector.z = yaw_;
    yaw_diagnostics_pub_->publish(diagnostics);
  }

  bool enabled_{true};
  bool publish_tf_{true};
  bool use_imu_orientation_{true};
  bool relative_imu_yaw_{false};
  bool imu_reference_initialized_{false};
  bool encoder_initialized_{false};
  bool publish_yaw_diagnostics_{true};

  double wheel_radius_{0.0875};
  double wheel_separation_{0.36};
  double imu_timeout_sec_{0.25};
  double imu_history_duration_sec_{1.0};
  double max_encoder_interval_sec_{0.5};
  double max_wheel_speed_m_s_{2.0};
  double imu_correction_time_constant_sec_{0.5};
  double max_imu_correction_rate_rad_s_{1.0};
  double pose_xy_variance_{0.0025};
  double pose_yaw_variance_{0.00121847};
  double twist_linear_variance_{0.01};
  double twist_angular_variance_{0.01};
  double initial_imu_yaw_{0.0};
  double previous_left_position_{0.0};
  double previous_right_position_{0.0};
  double previous_yaw_{0.0};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double wheel_yaw_{0.0};

  std::string wheel_topic_;
  std::string imu_topic_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string left_wheel_name_;
  std::string right_wheel_name_;

  std::deque<ImuYawSample> imu_yaw_history_;
  rclcpp::Time last_used_imu_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time previous_encoder_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr yaw_diagnostics_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr wheel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
