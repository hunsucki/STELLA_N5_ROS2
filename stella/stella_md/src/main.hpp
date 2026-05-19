#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class stellaN5_node : public rclcpp::Node
{
public:
    stellaN5_node();
    ~stellaN5_node();

private:

    // ROS timer
    rclcpp::TimerBase::SharedPtr Serial_timer;
    
    // ROS topic publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // ROS topic subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ahrs_yaw_sub_;
    
    rclcpp::Time time_now;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
    
    double goal_linear_velocity_ = 0.0;
    double goal_angular_velocity_ = 0.0;
    double imu_timeout_sec_ = 0.0;
    double imu_yaw_max_rate_ = 2.0;
    double imu_yaw_filter_tau_sec_ = 0.0;
    double imu_yaw_jump_warn_threshold_ = 0.25;
    bool use_imu_data_orientation_ = false;
    bool use_imu_yaw_filter_ = false;
    bool imu_data_received_ = false;
    bool yaw_received_ = false;
    bool filtered_yaw_initialized_ = false;
    rclcpp::Time latest_imu_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time latest_yaw_receive_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time latest_filtered_yaw_stamp_{0, 0, RCL_ROS_TIME};
    
    int left_encoder_prev=0,right_encoder_prev=0;
    
    double ahrs_yaw=0.0, filtered_yaw_rad_=0.0, delta_th=0.0,delta_s=0.0,delta_x=0.0,delta_y=0.0,x=0.0,y=0.0,th=0.0,delta_left = 0,delta_right = 0;

    void imu_data_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void ahrs_yaw_data_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg);
    void serial_callback();
    bool update_odometry();
};
