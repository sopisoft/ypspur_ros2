#ifndef YP_SPUR_ROS2_NODE_UTIL_H
#define YP_SPUR_ROS2_NODE_UTIL_H

#include <chrono>
#include <string>

#include <geometry_msgs/msg/quaternion.hpp>

namespace yp_spur_ros2
{

struct VelocityCommand {
  double linear;
  double angular;
};

geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw);
VelocityCommand clamp_velocity(double linear, double angular, double max_linear_velocity, double max_angular_velocity);
std::chrono::nanoseconds publish_period_from_rate(double odom_publish_rate_hz);
bool is_command_timed_out(std::chrono::steady_clock::time_point last_command_time,
                          std::chrono::steady_clock::duration timeout, std::chrono::steady_clock::time_point now);
std::string resolve_parameter_file_path(const std::string &parameter_file);

} // namespace yp_spur_ros2

#endif // YP_SPUR_ROS2_NODE_UTIL_H
