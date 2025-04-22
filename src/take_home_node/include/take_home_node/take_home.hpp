#pragma once

// Here we include message types which we can subscribe to or publish
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <raptor_dbw_msgs/msg/steering_extended_report.hpp>
#include <novatel_oem7_msgs/msg/rawimu.hpp>   

#include <deque>

class TakeHome : public rclcpp::Node {
 public:
  TakeHome(const rclcpp::NodeOptions& options);

  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr);
  void wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr);
  void steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr);
  void imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr);
  void curv_callback(std_msgs::msg::Float32::ConstSharedPtr);
  void compute_and_publish_slip();

 private:

  // Subscribers and Publishers  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            odometry_subscriber;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr wheel_speed_sub;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>::SharedPtr steering_sub;
  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr     imu_sub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr             curv_sub;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr metric_publisher;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_rr_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_rl_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_fr_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_fl_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr imu_jitter_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lap_time_pub;

  rclcpp::TimerBase::SharedPtr timer_;

  double vx{0.0}, vy{0.0}, omega{0.0}, delta_rad{0.0};
  raptor_dbw_msgs::msg::WheelSpeedReport last_wheel_speeds;
  bool have_wheel_speed{false}, have_steering{false};

  std::deque<rclcpp::Time> imu_times;

  double last_curv{0.0};
  rclcpp::Time lap_start_time;
  bool have_lap_start{false};

  static constexpr double wr = 1.523;
  static constexpr double wf = 1.638;
  static constexpr double lf = 1.7238;
  static constexpr double steering_ratio = 15.0;
};
