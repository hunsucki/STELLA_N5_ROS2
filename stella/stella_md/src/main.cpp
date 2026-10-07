#include "main.hpp"
#include "MW_value.hpp"
#include "MW_serial.hpp"
#include "stella.hpp"
#include "encoder_unwrap.hpp"
#include "serial_deadline.hpp"
#include "serial.h"

extern serial::Serial _serial;
#include <algorithm>
#include <cmath>
#include <math.h>

#define convertor_d2r (M_PI / 180.0)

static bool RUN = false;

namespace
{
double angle_diff(double target, double source)
{
  return std::atan2(std::sin(target - source), std::cos(target - source));
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}
}

inline int Limit_i (int v, int lo, int hi)
{
	if(abs(v) > lo && abs(v) < hi) return v;
	
  else return 0;
}

inline double pulse2meter()
{
  double meter= ((2 * M_PI * Differential_MobileRobot.wheel_radius) / Differential_MobileRobot.gear_ratio / MyMotorConfiguration.encoder_ppr[0]);

  return meter; //엔코더 역상
}

stellaN5_node::stellaN5_node() : Node("stella_md_node", rclcpp::NodeOptions().enable_rosout(false))
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  serial_transaction_timeout_ms_ = this->declare_parameter<int>("serial_transaction_timeout_ms", 100);
  if (serial_transaction_timeout_ms_ < 10 || serial_transaction_timeout_ms_ > 500) {
    throw std::invalid_argument("serial_transaction_timeout_ms must be in [10, 500]");
  }
  auto serial_timeout = serial::Timeout::simpleTimeout(serial_transaction_timeout_ms_);
  _serial.setTimeout(serial_timeout);
  {
    // Start from a stopped controller even after an unclean previous exit.
    motor_serial::Deadline deadline{std::chrono::milliseconds(serial_transaction_timeout_ms_)};
    dual_m_command(dual_m_command_select::m_lav, 0.0, 0.0);
  }

  use_imu_data_orientation_ = this->declare_parameter<bool>("use_imu_data_orientation", false);
  imu_timeout_sec_ = this->declare_parameter<double>("imu_timeout_sec", 0.0);
  use_imu_yaw_filter_ = this->declare_parameter<bool>("use_imu_yaw_filter", false);
  imu_yaw_max_rate_ = this->declare_parameter<double>("imu_yaw_max_rate", 2.0);
  imu_yaw_filter_tau_sec_ = this->declare_parameter<double>("imu_yaw_filter_tau_sec", 0.0);
  imu_yaw_jump_warn_threshold_ = this->declare_parameter<double>("imu_yaw_jump_warn_threshold", 0.25);
  cmd_vel_timeout_sec_ = this->declare_parameter<double>("cmd_vel_timeout_sec", 0.5);
  enable_legacy_odom_ = this->declare_parameter<bool>("enable_legacy_odom", false);
  encoder_poll_rate_hz_ = this->declare_parameter<int>("encoder_poll_rate_hz", 30);
  const int legacy_monitoring_rate_hz = this->declare_parameter<int>("monitoring_rate_hz", 0);
  if (imu_timeout_sec_ <= 0.0)
  {
    imu_timeout_sec_ = 0.0;
  }
  if (imu_yaw_max_rate_ <= 0.0)
  {
    imu_yaw_max_rate_ = 2.0;
  }
  if (imu_yaw_filter_tau_sec_ < 0.0)
  {
    imu_yaw_filter_tau_sec_ = 0.0;
  }
  if (imu_yaw_jump_warn_threshold_ <= 0.0)
  {
    imu_yaw_jump_warn_threshold_ = 0.25;
  }
  if (cmd_vel_timeout_sec_ < 0.0)
  {
    cmd_vel_timeout_sec_ = 0.0;
  }

  if (legacy_monitoring_rate_hz > 0)
  {
    encoder_poll_rate_hz_ = legacy_monitoring_rate_hz;
    RCLCPP_WARN(
        this->get_logger(),
        "Parameter monitoring_rate_hz is deprecated; use encoder_poll_rate_hz instead");
  }
  if (encoder_poll_rate_hz_ <= 0)
  {
    encoder_poll_rate_hz_ = 30;
  }

  if (use_imu_data_orientation_)
  {
    imu_data_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu/data", 10, std::bind(&stellaN5_node::imu_data_callback, this, std::placeholders::_1));
  }
  ahrs_yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>("imu/yaw", 1, std::bind(&stellaN5_node::ahrs_yaw_data_callback, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(1), std::bind(&stellaN5_node::command_velocity_callback, this, std::placeholders::_1));
  wheel_encoder_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("wheel/encoders", qos);
  if (enable_legacy_odom_)
  {
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
    odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this, rclcpp::QoS(1));
  }

  // Drain the latest completed query promptly, independently of the serial
  // polling phase. Empty callbacks do not publish or fabricate sensor samples.
  Serial_timer = this->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&stellaN5_node::serial_callback, this));

  RCLCPP_INFO(
      this->get_logger(),
      "Encoder polling: %d Hz; wheel topic: /wheel/encoders; legacy odom: %s",
      encoder_poll_rate_hz_, enable_legacy_odom_ ? "enabled" : "disabled");
  serial_running_.store(true);
  serial_thread_ = std::thread(&stellaN5_node::serial_loop, this);
}

stellaN5_node::~stellaN5_node()
{
  serial_running_.store(false);
  command_changed_.notify_all();
  if (serial_thread_.joinable()) {serial_thread_.join();}
  if (RUN)
  {
    try
    {
      motor_serial::Deadline deadline{std::chrono::milliseconds(serial_transaction_timeout_ms_)};
      dual_m_command(dual_m_command_select::m_lav, 0.0, 0.0);
    }
    catch (const std::exception & e)
    {
      RCLCPP_WARN(this->get_logger(), "Final motor stop failed: %s", e.what());
    }
  }
  MW_Serial_DisConnect();
}

void stellaN5_node::imu_data_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (!use_imu_data_orientation_)
  {
    return;
  }

  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for stamped /imu/data orientation");
    return;
  }

  const double yaw = yaw_from_quaternion(msg->orientation);
  if (!std::isfinite(yaw))
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Ignoring invalid /imu/data orientation");
    return;
  }

  const rclcpp::Time stamp(msg->header.stamp);
  if (!use_imu_yaw_filter_)
  {
    ahrs_yaw = yaw / convertor_d2r;
    latest_imu_stamp_ = stamp;
    imu_data_received_ = true;
    return;
  }

  if (!filtered_yaw_initialized_)
  {
    filtered_yaw_rad_ = yaw;
    latest_filtered_yaw_stamp_ = stamp;
    filtered_yaw_initialized_ = true;
  }
  else
  {
    double dt = (stamp - latest_filtered_yaw_stamp_).seconds();
    if (dt <= 0.0 || dt > 1.0)
    {
      dt = 1.0 / 50.0;
    }

    const double raw_delta = angle_diff(yaw, filtered_yaw_rad_);
    if (std::abs(raw_delta) > imu_yaw_jump_warn_threshold_)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Limiting IMU yaw step %.3f rad", raw_delta);
    }

    const double max_step = imu_yaw_max_rate_ * dt;
    const double limited_delta = std::max(-max_step, std::min(max_step, raw_delta));
    const double alpha = imu_yaw_filter_tau_sec_ > 0.0
        ? std::min(1.0, dt / (imu_yaw_filter_tau_sec_ + dt))
        : 1.0;

    filtered_yaw_rad_ += alpha * limited_delta;
    latest_filtered_yaw_stamp_ = stamp;
  }

  ahrs_yaw = filtered_yaw_rad_ / convertor_d2r;
  latest_imu_stamp_ = stamp;
  imu_data_received_ = true;
}

void stellaN5_node::ahrs_yaw_data_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  if (!std::isfinite(msg->data))
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Ignoring invalid /imu/yaw value");
    return;
  }

  if (!use_imu_data_orientation_ || !imu_data_received_)
  {
    ahrs_yaw = msg->data;
  }

  latest_yaw_receive_time_ = this->now();
  yaw_received_ = true;
}

void stellaN5_node::command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->angular.z)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring non-finite /cmd_vel");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    pending_linear_velocity_ = msg->linear.x;
    pending_angular_velocity_ = msg->angular.z;
    last_cmd_vel_time_ = std::chrono::steady_clock::now();
    cmd_vel_received_ = true;
    ++command_sequence_;
  }
  command_changed_.notify_one();
}

bool stellaN5_node::apply_pending_command()
{
  std::uint64_t sequence;
  double linear;
  double angular;
  double age;
  bool timed_out;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!cmd_vel_received_) {return true;}
    sequence = command_sequence_;
    age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_cmd_vel_time_).count();
    timed_out = cmd_vel_timeout_sec_ > 0.0 && age > cmd_vel_timeout_sec_;
    if (sequence == applied_command_sequence_ && (!timed_out || cmd_vel_watchdog_stopped_)) {
      return true;
    }
    linear = timed_out ? 0.0 : pending_linear_velocity_;
    angular = timed_out ? 0.0 : pending_angular_velocity_;
  }
  try {
    motor_serial::Deadline deadline{std::chrono::milliseconds(serial_transaction_timeout_ms_)};
    dual_m_command(dual_m_command_select::m_lav, linear, angular);
    applied_command_sequence_ = sequence;
    cmd_vel_watchdog_stopped_ = timed_out;
    if (timed_out) {
      RCLCPP_WARN(get_logger(), "/cmd_vel timeout after %.3f s; motors stopped", age);
    }
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
      "Motor command failed (stop=%s): %s", timed_out ? "true" : "false", e.what());
    return false;  // Retry on the next worker iteration, including stop commands.
  }
}

void stellaN5_node::serial_loop()
{
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(1.0 / encoder_poll_rate_hz_));
  auto next_poll = std::chrono::steady_clock::now();
  while (rclcpp::ok() && serial_running_.load() && RUN) {
    const bool command_ok = apply_pending_command();
    auto now = std::chrono::steady_clock::now();
    if (now >= next_poll && command_ok) {
      const auto start = this->now();
      try {
        {
          motor_serial::Deadline deadline{std::chrono::milliseconds(serial_transaction_timeout_ms_)};
          if (Motor_MonitoringCommand(channel_1, _position) < 0) {
            throw std::runtime_error("Left encoder query failed");
          }
        }
        if (!apply_pending_command()) {  // Check the stop deadline between serial queries.
          throw std::runtime_error("Pending motor command failed");
        }
        {
          motor_serial::Deadline deadline{std::chrono::milliseconds(serial_transaction_timeout_ms_)};
          if (Motor_MonitoringCommand(channel_2, _position) < 0) {
            throw std::runtime_error("Right encoder query failed");
          }
        }
        const auto end = this->now();
        std::lock_guard<std::mutex> lock(encoder_mutex_);
        sampled_left_encoder_ = static_cast<std::int32_t>(MyMotorCommandReadValue.position[channel_1]);
        sampled_right_encoder_ = static_cast<std::int32_t>(MyMotorCommandReadValue.position[channel_2]);
        sampled_encoder_stamp_ = rclcpp::Time(
          start.nanoseconds() + (end.nanoseconds() - start.nanoseconds()) / 2,
          get_clock()->get_clock_type());
        encoder_sample_pending_ = true;
      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Encoder query failed; no sample published: %s", e.what());
        // Discard a partial/late response before issuing the next request.
        try {_serial.flushInput();} catch (const std::exception &) {}
      }
      now = std::chrono::steady_clock::now();
      next_poll += period;
      if (next_poll <= now) {next_poll = now;}
    }
    std::unique_lock<std::mutex> lock(command_mutex_);
    auto wake = std::min(next_poll, std::chrono::steady_clock::now() + std::chrono::milliseconds(10));
    command_changed_.wait_until(lock, wake, [this] {
      return !serial_running_.load() || command_sequence_ != applied_command_sequence_;
    });
    if (!command_ok) {
      // A failing pending command must not create an unbounded retry loop.
      command_changed_.wait_for(lock, std::chrono::milliseconds(10));
    }
  }
}

void stellaN5_node::serial_callback()
{
  std::int32_t left_raw;
  std::int32_t right_raw;
  rclcpp::Time stamp(0, 0, get_clock()->get_clock_type());
  {
    std::lock_guard<std::mutex> lock(encoder_mutex_);
    if (!encoder_sample_pending_) {return;}
    left_raw = sampled_left_encoder_;
    right_raw = sampled_right_encoder_;
    stamp = sampled_encoder_stamp_;
    encoder_sample_pending_ = false;
  }
  // DDS calls run only in the executor. The serial worker and stop watchdog
  // continue even when a remote reader or the network send queue stalls.
  publish_wheel_encoder_state(stamp, left_raw, right_raw);
  if (RUN)
  {
    enable_legacy_odom_ = this->get_parameter("enable_legacy_odom").as_bool();
    if (enable_legacy_odom_)
    {
      if (!previous_legacy_odom_enabled_)
      {
        if (!odom_pub_)
        {
          odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "odom", rclcpp::QoS(rclcpp::KeepLast(1)));
        }
        if (!odom_broadcaster)
        {
          odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this, rclcpp::QoS(1));
        }
        left_encoder_prev = left_raw;
        right_encoder_prev = right_raw;
        previous_legacy_odom_enabled_ = true;
        RCLCPP_WARN(
            this->get_logger(),
            "Legacy odom enabled. Disable wheel_odometry to avoid duplicate /odom and TF publishers");
      }
      update_odometry(left_raw, right_raw);
    }
    else
    {
      if (previous_legacy_odom_enabled_)
      {
        RCLCPP_WARN(this->get_logger(), "Legacy odom disabled");
      }
      previous_legacy_odom_enabled_ = false;
      odom_pub_.reset();
      odom_broadcaster.reset();
    }
  }
}

void stellaN5_node::publish_wheel_encoder_state(
  const rclcpp::Time & stamp, std::int32_t left_raw, std::int32_t right_raw)
{
  const double counts_per_wheel_revolution =
      Differential_MobileRobot.gear_ratio * MyMotorConfiguration.encoder_ppr[channel_1];
  if (counts_per_wheel_revolution <= 0.0)
  {
    RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Invalid encoder configuration: gear_ratio=%.3f encoder_ppr=%ld",
        Differential_MobileRobot.gear_ratio,
        MyMotorConfiguration.encoder_ppr[channel_1]);
    return;
  }

  const double radians_per_count = 2.0 * M_PI / counts_per_wheel_revolution;
  double left_delta = 0.0;
  double right_delta = 0.0;
  if (wheel_encoder_initialized_)
  {
    left_delta = unwrap_signed_32bit_delta(
        left_raw, left_encoder_prev_for_wheel_state_) * radians_per_count;
    right_delta = unwrap_signed_32bit_delta(
        right_raw, right_encoder_prev_for_wheel_state_) * radians_per_count;
    left_wheel_position_ += left_delta;
    right_wheel_position_ += right_delta;
  }

  sensor_msgs::msg::JointState wheel_state;
  wheel_state.header.stamp = stamp;
  wheel_state.name = {"left_wheel", "right_wheel"};
  wheel_state.position = {left_wheel_position_, right_wheel_position_};
  wheel_state.velocity = {0.0, 0.0};

  if (wheel_encoder_initialized_)
  {
    const double dt = (stamp - last_wheel_encoder_stamp_).seconds();
    if (dt > 0.0)
    {
      wheel_state.velocity[0] =
          left_delta / dt;
      wheel_state.velocity[1] =
          right_delta / dt;
    }
  }

  left_encoder_prev_for_wheel_state_ = left_raw;
  right_encoder_prev_for_wheel_state_ = right_raw;
  last_wheel_encoder_stamp_ = stamp;
  wheel_encoder_initialized_ = true;
  wheel_encoder_pub_->publish(wheel_state);
}

bool stellaN5_node::update_odometry(std::int32_t left_raw, std::int32_t right_raw)
{
  const rclcpp::Time now = this->now();

  if (use_imu_data_orientation_)
  {
    if (!imu_data_received_)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for /imu/data before publishing odom");
      return false;
    }

    const double imu_age = (now - latest_imu_stamp_).seconds();
    if (imu_timeout_sec_ > 0.0 && imu_age > imu_timeout_sec_)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "/imu/data is stale (age %.3f sec); skipping odom update", imu_age);
      return false;
    }
  }
  else
  {
    if (!yaw_received_)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for /imu/yaw before publishing odom");
      return false;
    }

    const double yaw_age = (now - latest_yaw_receive_time_).seconds();
    if (imu_timeout_sec_ > 0.0 && yaw_age > imu_timeout_sec_)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "/imu/yaw callback is stale (age %.3f sec); skipping odom update", yaw_age);
      return false;
    }
  }


  delta_left = Limit_i((left_raw - left_encoder_prev), 0, 15000) * pulse2meter();
  delta_right = Limit_i((right_raw - right_encoder_prev), 0, 15000) * pulse2meter();
  
  //로봇 기구학 적용및 센서퓨전
  delta_s  = (delta_right + delta_left) / 2.0 ;
  delta_th = (ahrs_yaw * convertor_d2r); 

/*
  th 값은 AHRS YAW 값을 참조하여 아래의 식은 주석으로 처리한다.
  delta_th = (delta_right - delta_left) / Differential_MobileRobot.axle_length;
*/

  delta_x  = delta_s * cos(delta_th);
  delta_y  = delta_s * sin(delta_th);
  
  x += delta_x;
  y += delta_y;

  nav_msgs::msg::Odometry odom;

  tf2::Quaternion Quaternion;
  Quaternion.setRPY(0, 0, delta_th);

  odom.pose.pose.orientation.x = Quaternion.x();
  odom.pose.pose.orientation.y = Quaternion.y();
  odom.pose.pose.orientation.z = Quaternion.z();
  odom.pose.pose.orientation.w = Quaternion.w();

  geometry_msgs::msg::TransformStamped t;
  
  t.header.stamp = now;
  t.header.frame_id = "odom";
  t.child_frame_id = "base_footprint";

  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = 0.0;

  t.transform.rotation.x = Quaternion.x();
  t.transform.rotation.y = Quaternion.y();
  t.transform.rotation.z = Quaternion.z();
  t.transform.rotation.w = Quaternion.w();

  odom_broadcaster->sendTransform(t);

  odom.header.frame_id = "odom";

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;

  odom.child_frame_id = "base_footprint";
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    const bool stale = cmd_vel_timeout_sec_ > 0.0 && (!cmd_vel_received_ ||
      std::chrono::duration<double>(std::chrono::steady_clock::now() - last_cmd_vel_time_).count() > cmd_vel_timeout_sec_);
    odom.twist.twist.linear.x = stale ? 0.0 : pending_linear_velocity_;
    odom.twist.twist.angular.z = stale ? 0.0 : pending_angular_velocity_;
  }

  odom.header.stamp = now;
  odom_pub_->publish(odom);

  left_encoder_prev = left_raw;
  right_encoder_prev = right_raw;
  return true;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  char motor_port[] = "/dev/MW";
  int result = 0;
  try {
    // Bound vendor startup too: a missing controller must exit, not spin forever.
    motor_serial::Deadline startup_deadline{std::chrono::seconds(15)};
    if (!MW_Serial_Connect(motor_port, 115200) || !Robot_Setting(::N5)) {
      throw std::runtime_error("Motor controller connection/setup failed");
    }
    RUN = true;
    Robot_Fault_Checking_RESET();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("stella_md_node"), "Motor startup failed: %s", e.what());
    MW_Serial_DisConnect();
    rclcpp::shutdown();
    return 1;
  }
  try {
    rclcpp::spin(std::make_shared<stellaN5_node>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("stella_md_node"), "Motor node failed: %s", e.what());
    result = 1;
  }

  rclcpp::shutdown();
  return result;
}
