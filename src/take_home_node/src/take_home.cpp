#include "take_home_node/take_home.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <raptor_dbw_msgs/msg/steering_extended_report.hpp>
#include <novatel_oem7_msgs/msg/rawimu.hpp>
#include <cmath>

TakeHome::TakeHome(const rclcpp::NodeOptions& options)
    : Node("take_home_metrics", options) {

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // Look at the hpp file to define all class variables, including subscribers
    // A subscriber will "listen" to a topic and whenever a message is published to it, the subscriber
    // will pass it onto the attached callback (`TakeHome::odometry_callback` in this instance)
    odometry_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
      "vehicle/uva_odometry", qos_profile,
      std::bind(&TakeHome::odometry_callback, this, std::placeholders::_1));

    metric_publisher = this->create_publisher<std_msgs::msg::Float32>("metrics_output", qos_profile);

    slip_rr_pub = this->create_publisher<std_msgs::msg::Float32>("slip/long/rr", qos_profile);
    slip_rl_pub = this->create_publisher<std_msgs::msg::Float32>("slip/long/rl", qos_profile);
    slip_fr_pub = this->create_publisher<std_msgs::msg::Float32>("slip/long/fr", qos_profile);
    slip_fl_pub = this->create_publisher<std_msgs::msg::Float32>("slip/long/fl", qos_profile);

    wheel_speed_sub = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
      "/raptor_dbw_interface/wheel_speed_report", qos_profile,
      std::bind(&TakeHome::wheel_speed_callback, this, std::placeholders::_1));

    steering_sub = this->create_subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>(
      "/raptor_dbw_interface/steering_extended_report", qos_profile,
      std::bind(&TakeHome::steering_callback, this, std::placeholders::_1));

    imu_sub = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
      "/novatel_top/rawimu", qos_profile,
      std::bind(&TakeHome::imu_callback, this, std::placeholders::_1));

    imu_jitter_pub = this->create_publisher<std_msgs::msg::Float32>("imu_top/jitter", qos_profile);

    curv_sub = this->create_subscription<std_msgs::msg::Float32>(
      "curvilinear_distance", qos_profile,
      std::bind(&TakeHome::curv_callback, this, std::placeholders::_1));

    lap_time_pub = this->create_publisher<std_msgs::msg::Float32>("lap_time", qos_profile);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&TakeHome::compute_and_publish_slip, this));
}
// 
/**
 * Whenever a message is published to the topic "vehicle/uva_odometry" the subscriber passes the message onto this callback
 * To see what is in each message look for the corresponding .msg file in this repository
 * For instance, when running `ros2 bag info` on the given bag, we see the wheel speed report has message type of raptor_dbw_msgs/msgs/WheelSpeedReport
 * and from the corresponding WheelSpeedReport.msg file we see that we can do msg->front_left for the front left speed for instance.
 * For the built in ROS2 messages, we can find the documentation online: e.g. https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
 */

void TakeHome::odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
  float position_x = odom_msg->pose.pose.position.x;
  float position_y = odom_msg->pose.pose.position.y;
  float position_z = odom_msg->pose.pose.position.z;

  // Do stuff with this callback! or more, idc
  std_msgs::msg::Float32 metric_msg;
  metric_msg.data = (position_x + position_y + position_z) / (position_x + position_z);
  metric_publisher->publish(metric_msg);

  vx = odom_msg->twist.twist.linear.x;
  vy = odom_msg->twist.twist.linear.y;
  omega = odom_msg->twist.twist.angular.z;
}

void TakeHome::wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr msg) {
  last_wheel_speeds = *msg;
  have_wheel_speed = true;
}

void TakeHome::steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr msg) {
  delta_rad = (msg->primary_steering_angle_fbk / steering_ratio) * M_PI / 180.0;
  have_steering = true;
}

void TakeHome::compute_and_publish_slip() {
  if (!have_wheel_speed || !have_steering) return;

  auto to_mps = [](float kmph){ return kmph / 3.6f; };

  double v_rr = to_mps(last_wheel_speeds.rear_right);
  double v_rl = to_mps(last_wheel_speeds.rear_left);
  double v_fr = to_mps(last_wheel_speeds.front_right);
  double v_fl = to_mps(last_wheel_speeds.front_left);

  double vx_rr = vx - 0.5 * omega * wr;
  double vx_rl = vx + 0.5 * omega * wr;
  double k_rr  = (v_rr - vx_rr) / vx_rr;
  double k_rl  = (v_rl - vx_rl) / vx_rl;

  double vx_fr = vx - 0.5 * omega * wf;
  double vx_fl = vx + 0.5 * omega * wf;
  double vy_f  = vy + omega * lf;

  double vx_fr_d = std::cos(delta_rad) * vx_fr - std::sin(delta_rad) * vy_f;
  double vx_fl_d = std::cos(delta_rad) * vx_fl - std::sin(delta_rad) * vy_f;
  double k_fr = (v_fr - vx_fr_d) / vx_fr_d;
  double k_fl = (v_fl - vx_fl_d) / vx_fl_d;

  std_msgs::msg::Float32 out;
  out.data = k_rr; slip_rr_pub->publish(out);
  out.data = k_rl; slip_rl_pub->publish(out);
  out.data = k_fr; slip_fr_pub->publish(out);
  out.data = k_fl; slip_fl_pub->publish(out);
}

void TakeHome::imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg)
{
  rclcpp::Time now(msg->header.stamp); // Current IMU timestamp

  imu_times.push_back(now); // Add to sliding window

  // Remove any timestamps older than 1 second
  while (!imu_times.empty() && (now - imu_times.front()).seconds() > 1.0){
    imu_times.pop_front();
  }

  if (imu_times.size() < 2) {
    return;
  }

  double sum = 0.0;
  double sum_sq = 0.0;

  // Compute the delta t between consecutive timestamps
  for (size_t i = 1; i < imu_times.size(); ++i) {
    double dt = (imu_times[i] - imu_times[i-1]).seconds();
    sum += dt;
    sum_sq += dt * dt;
  }

  // Compute mean of the time deltas
  const size_t n = imu_times.size() - 1;
  double mean = sum / n;

  // Compute variance (jitter)
  std_msgs::msg::Float32 out;
  out.data = (sum_sq - n * mean * mean) / (n - 1);
  imu_jitter_pub->publish(out);
}

// Store time of current lap
double current_lap_time{0.0};

void TakeHome::curv_callback(std_msgs::msg::Float32::ConstSharedPtr msg)
{
  constexpr double DROP_THRESHOLD = 3000.0; // Detects when the curvilinear distance has reset
  double dist = msg->data;
  rclcpp::Time now = this->now(); // Current time 

  // For first message recieved
  if (!have_lap_start) {
    have_lap_start = true;
    lap_start_time = now;
    last_curv = dist;
    current_lap_time = 0.0;              
  }

  // Note new lap has started if distance drops significantly
  if (dist < last_curv - DROP_THRESHOLD) {
    rclcpp::Duration lap = now - lap_start_time;
    current_lap_time = lap.seconds();    
    lap_start_time  = now;
  }

  std_msgs::msg::Float32 out;
  out.data = current_lap_time;
  lap_time_pub->publish(out);

  last_curv = dist;
}


RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome)
