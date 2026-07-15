#ifndef YP_SPUR_ROS2_NODE_CONTROLLER_H
#define YP_SPUR_ROS2_NODE_CONTROLLER_H

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <yp_spur_ros2/coordinator_client.h>
#include <yp_spur_ros2/node_util.h>

namespace yp_spur_ros2
{

class NodeController
{
public:
  struct Config {
    std::string device;
    std::string parameter_file;
    int baudrate = 115200;
    std::string odom_frame = "odom";
    std::string base_frame = "base_link";
    double cmd_vel_timeout = 0.5;
    double odom_publish_rate = 30.0;
    double max_linear_velocity = 1.0;
    double max_angular_velocity = 2.0;
    bool publish_tf = true;
  };

  NodeController(rclcpp::Node &node, CoordinatorClient &client, const Config &config, bool enable_timers = true);
  ~NodeController();

  void handle_cmd_vel(const geometry_msgs::msg::Twist &msg);
  void handle_timeout();
  void publish_odometry();
  void stop();

private:
  void send_velocity(double linear, double angular);

  rclcpp::Node &node_;
  CoordinatorClient &client_;
  Config config_;
  std::mutex command_mutex_;
  std::chrono::steady_clock::time_point last_command_time_ = std::chrono::steady_clock::now();
  bool timed_out_ = false;
  bool stopped_ = false;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

} // namespace yp_spur_ros2

#endif // YP_SPUR_ROS2_NODE_CONTROLLER_H
