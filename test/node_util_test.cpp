#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>

#include <yp_spur_ros2/node_util.h>

int main()
{
  const auto zero = yp_spur_ros2::yaw_to_quaternion(0.0);
  if (std::fabs(zero.z) > 1e-9 || std::fabs(zero.w - 1.0) > 1e-9) {
    std::cerr << "yaw zero conversion failed\n";
    return 1;
  }

  const auto quarter_turn = yp_spur_ros2::yaw_to_quaternion(3.14159265358979323846 / 2.0);
  if (std::fabs(quarter_turn.z - 0.7071067811865475) > 1e-6 || std::fabs(quarter_turn.w - 0.7071067811865476) > 1e-6) {
    std::cerr << "quarter turn conversion failed\n";
    return 1;
  }

  const auto clamped = yp_spur_ros2::clamp_velocity(2.0, -5.0, 1.25, 3.0);
  if (std::fabs(clamped.linear - 1.25) > 1e-9 || std::fabs(clamped.angular + 3.0) > 1e-9) {
    std::cerr << "velocity clamp failed\n";
    return 1;
  }

  const auto period = yp_spur_ros2::publish_period_from_rate(20.0);
  if (period != std::chrono::nanoseconds(50000000)) {
    std::cerr << "publish period calculation failed\n";
    return 1;
  }

  const auto floor_period = yp_spur_ros2::publish_period_from_rate(0.0);
  if (floor_period != std::chrono::nanoseconds(10000000000LL)) {
    std::cerr << "publish period floor failed\n";
    return 1;
  }

  const auto now = std::chrono::steady_clock::now();
  if (yp_spur_ros2::is_command_timed_out(
        now, std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::milliseconds(500)), now)) {
    std::cerr << "timeout should not trigger immediately\n";
    return 1;
  }
  if (!yp_spur_ros2::is_command_timed_out(
        now - std::chrono::seconds(2),
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::milliseconds(500)), now)) {
    std::cerr << "timeout should trigger after a long gap\n";
    return 1;
  }
  if (yp_spur_ros2::is_command_timed_out(
        now - std::chrono::milliseconds(500),
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::milliseconds(500)), now)) {
    std::cerr << "timeout should not trigger on the boundary\n";
    return 1;
  }

  const auto negative_limit = yp_spur_ros2::clamp_velocity(-3.0, 4.0, -1.5, -2.25);
  if (std::fabs(negative_limit.linear + 1.5) > 1e-9 || std::fabs(negative_limit.angular - 2.25) > 1e-9) {
    std::cerr << "negative limits should be normalized\n";
    return 1;
  }

  const char *home_env = std::getenv("HOME");
  if (!home_env) {
    std::cerr << "HOME must be set for parameter path resolution test\n";
    return 1;
  }
  const std::string home = home_env;
  const std::string home_param = home + "/.yp_spur_ros2_test.param";
  {
    FILE *file = std::fopen(home_param.c_str(), "w");
    if (!file) {
      std::cerr << "failed to create test parameter file in home directory\n";
      return 1;
    }
    std::fputs("VERSION 4\n", file);
    std::fclose(file);
  }
  if (yp_spur_ros2::resolve_parameter_file_path("~/.yp_spur_ros2_test.param") != home_param) {
    std::cerr << "home directory should be expanded for parameter files\n";
    std::remove(home_param.c_str());
    return 1;
  }
  std::remove(home_param.c_str());

  return 0;
}
