#include "yp_spur_ros2/node_util.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>

namespace yp_spur_ros2
{

namespace
{

std::filesystem::path expand_home_directory(const std::string &path)
{
  if (path.empty() || path[0] != '~') {
    return std::filesystem::path(path);
  }

  const char *home = std::getenv("HOME");
  if (!home || home[0] == '\0') {
    return std::filesystem::path(path);
  }

  if (path.size() == 1) {
    return std::filesystem::path(home);
  }

  if (path[1] != '/') {
    return std::filesystem::path(path);
  }

  return std::filesystem::path(home) / path.substr(2);
}

} // namespace

geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  const double half_yaw = yaw * 0.5;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half_yaw);
  q.w = std::cos(half_yaw);
  return q;
}

VelocityCommand clamp_velocity(double linear, double angular, double max_linear_velocity, double max_angular_velocity)
{
  VelocityCommand command;
  command.linear = std::clamp(linear, -std::abs(max_linear_velocity), std::abs(max_linear_velocity));
  command.angular = std::clamp(angular, -std::abs(max_angular_velocity), std::abs(max_angular_velocity));
  return command;
}

std::chrono::nanoseconds publish_period_from_rate(double odom_publish_rate_hz)
{
  const double clamped_rate = std::max(odom_publish_rate_hz, 0.1);
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / clamped_rate));
}

bool is_command_timed_out(std::chrono::steady_clock::time_point last_command_time,
                          std::chrono::steady_clock::duration timeout, std::chrono::steady_clock::time_point now)
{
  return (now - last_command_time) > timeout;
}

std::string resolve_parameter_file_path(const std::string &parameter_file)
{
  if (parameter_file.empty() || parameter_file == "embedded") {
    return parameter_file;
  }

  const auto expanded_path = expand_home_directory(parameter_file);
  if (std::filesystem::exists(expanded_path)) {
    return expanded_path.string();
  }

  return expanded_path.string();
}

} // namespace yp_spur_ros2
