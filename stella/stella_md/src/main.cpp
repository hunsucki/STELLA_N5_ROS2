#include "main.hpp"
#include "MW_value.hpp"
#include "MW_serial.hpp"
#include "stella.hpp"
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

stellaN5_node::stellaN5_node() : Node("stella_md_node")
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  use_imu_data_orientation_ = this->declare_parameter<bool>("use_imu_data_orientation", false);
  imu_timeout_sec_ = this->declare_parameter<double>("imu_timeout_sec", 0.0);
  use_imu_yaw_filter_ = this->declare_parameter<bool>("use_imu_yaw_filter", false);
  imu_yaw_max_rate_ = this->declare_parameter<double>("imu_yaw_max_rate", 2.0);
  imu_yaw_filter_tau_sec_ = this->declare_parameter<double>("imu_yaw_filter_tau_sec", 0.0);
  imu_yaw_jump_warn_threshold_ = this->declare_parameter<double>("imu_yaw_jump_warn_threshold", 0.25);
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

  if (use_imu_data_orientation_)
  {
    imu_data_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu/data", 10, std::bind(&stellaN5_node::imu_data_callback, this, std::placeholders::_1));
  }
  ahrs_yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>("imu/yaw", 1, std::bind(&stellaN5_node::ahrs_yaw_data_callback, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&stellaN5_node::command_velocity_callback, this, std::placeholders::_1));
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
  odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  int monitoring_rate_hz = this->declare_parameter<int>("monitoring_rate_hz", 10);
  if (monitoring_rate_hz <= 0)
  {
    monitoring_rate_hz = 10;
  }

  auto serial_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / monitoring_rate_hz));
  Serial_timer = this->create_wall_timer(serial_period, std::bind(&stellaN5_node::serial_callback, this));
}

stellaN5_node::~stellaN5_node()
{
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

void stellaN5_node::command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg)
{
  if(RUN)
  {
    goal_linear_velocity_ = cmd_vel_msg->linear.x;
    goal_angular_velocity_ = cmd_vel_msg->angular.z ;

    try
    {
      dual_m_command(dual_m_command_select::m_lav, goal_linear_velocity_, goal_angular_velocity_);
    }
    catch (const std::exception & e)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Motor command serial write failed: %s", e.what());
    }
  }
}

void stellaN5_node::serial_callback()
{
  if(RUN)
  {
    try
    {
      Motor_MonitoringCommand(channel_1, _position);
      Motor_MonitoringCommand(channel_2, _position);
    }
    catch (const std::exception & e)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Motor monitoring serial read failed: %s", e.what());
      return;
    }

    update_odometry();
  }
}

bool stellaN5_node::update_odometry()
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


  delta_left = Limit_i((MyMotorCommandReadValue.position[channel_1] - left_encoder_prev), 0, 15000) * pulse2meter();
  delta_right = Limit_i((MyMotorCommandReadValue.position[channel_2] - right_encoder_prev), 0, 15000) * pulse2meter();
  
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
  odom.twist.twist.linear.x = goal_linear_velocity_;
  odom.twist.twist.angular.z = goal_angular_velocity_;

  odom.header.stamp = now;
  odom_pub_->publish(odom);

  left_encoder_prev = MyMotorCommandReadValue.position[channel_1];
  right_encoder_prev = MyMotorCommandReadValue.position[channel_2];
  return true;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  MW_Serial_Connect("/dev/MW", 115200);

  if(Robot_Setting(::N5)) RUN = true;
  Robot_Fault_Checking_RESET();
  
  rclcpp::spin(std::make_shared<stellaN5_node>());

  rclcpp::shutdown();
  return 0;
}
