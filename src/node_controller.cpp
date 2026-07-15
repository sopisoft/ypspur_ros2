#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <yp_spur_ros2/node_controller.h>

namespace yp_spur_ros2
{

NodeController::NodeController(rclcpp::Node &node, CoordinatorClient &client, const Config &config, bool enable_timers)
    : node_(node), client_(client), config_(config)
{
  if (config_.device.empty() || config_.parameter_file.empty()) {
    throw std::runtime_error("device and parameter_file parameters are required");
  }

  config_.parameter_file = resolve_parameter_file_path(config_.parameter_file);
  if (config_.baudrate <= 0) {
    throw std::runtime_error("baudrate parameter must be positive");
  }

  YpscRos2Config c_config{};
  c_config.device = config_.device.c_str();
  c_config.parameter_file = config_.parameter_file.c_str();
  c_config.baudrate = config_.baudrate;

  std::string error;
  if (!client_.start(c_config, error)) {
    throw std::runtime_error(error.empty() ? "failed to start coordinator" : error);
  }

  odom_pub_ = node_.create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  cmd_vel_sub_ = node_.create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(10), [this](const geometry_msgs::msg::Twist &msg) { handle_cmd_vel(msg); });

  if (config_.publish_tf) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_.shared_from_this());
  }

  if (enable_timers) {
    const auto publish_period = publish_period_from_rate(config_.odom_publish_rate);
    const auto timeout_period = std::chrono::milliseconds(20);
    odom_timer_ = node_.create_wall_timer(publish_period, [this]() { publish_odometry(); });
    timeout_timer_ = node_.create_wall_timer(timeout_period, [this]() { handle_timeout(); });
  }
}

NodeController::~NodeController()
{
  stop();
}

void NodeController::stop()
{
  if (stopped_) {
    return;
  }
  stopped_ = true;
  client_.stop();
}

void NodeController::handle_cmd_vel(const geometry_msgs::msg::Twist &msg)
{
  const auto clamped =
    clamp_velocity(msg.linear.x, msg.angular.z, config_.max_linear_velocity, config_.max_angular_velocity);

  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    last_command_time_ = std::chrono::steady_clock::now();
    timed_out_ = false;
  }

  send_velocity(clamped.linear, clamped.angular);
}

void NodeController::handle_timeout()
{
  const auto now = std::chrono::steady_clock::now();
  bool should_stop = false;

  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (timed_out_) {
      return;
    }
    should_stop = is_command_timed_out(last_command_time_,
                                       std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                         std::chrono::duration<double>(config_.cmd_vel_timeout)),
                                       now);
    if (should_stop) {
      timed_out_ = true;
    }
  }

  if (should_stop) {
    send_velocity(0.0, 0.0);
  }
}

void NodeController::publish_odometry()
{
  YpscRos2Odometry odom{};
  if (!client_.get_odometry(odom) || !odom.valid) {
    return;
  }

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = node_.now();
  msg.header.frame_id = config_.odom_frame;
  msg.child_frame_id = config_.base_frame;
  msg.pose.pose.position.x = odom.x;
  msg.pose.pose.position.y = odom.y;
  msg.pose.pose.position.z = 0.0;
  msg.pose.pose.orientation = yaw_to_quaternion(odom.yaw);
  msg.twist.twist.linear.x = odom.linear_velocity;
  msg.twist.twist.angular.z = odom.angular_velocity;
  odom_pub_->publish(msg);

  if (!config_.publish_tf || !tf_broadcaster_) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  transform.header = msg.header;
  transform.child_frame_id = config_.base_frame;
  transform.transform.translation.x = odom.x;
  transform.transform.translation.y = odom.y;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation = msg.pose.pose.orientation;
  tf_broadcaster_->sendTransform(transform);
}

void NodeController::send_velocity(double linear, double angular)
{
  if (!client_.is_connected()) {
    return;
  }

  std::string error;
  if (!client_.set_velocity(linear, angular, error)) {
    RCLCPP_WARN(node_.get_logger(), "failed to forward velocity command: %s", error.c_str());
  }
}

} // namespace yp_spur_ros2
