#include <chrono>
#include <cmath>
#include <future>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <yp_spur_ros2/node_controller.h>

namespace
{

class FakeCoordinatorClient final : public yp_spur_ros2::CoordinatorClient
{
public:
  bool start(const YpscRos2Config &config, std::string &error) override
  {
    started = true;
    connected = true;
    start_device = config.device;
    start_parameter_file = config.parameter_file;
    error.clear();
    return true;
  }

  bool set_velocity(double linear, double angular, std::string &error) override
  {
    last_linear = linear;
    last_angular = angular;
    ++velocity_calls;
    error.clear();
    return connected;
  }

  bool get_odometry(YpscRos2Odometry &odom) override
  {
    odom = snapshot;
    return snapshot.valid != 0;
  }

  bool stop() override
  {
    stopped = true;
    connected = false;
    return true;
  }

  bool is_connected() const override
  {
    return connected;
  }

  bool started = false;
  bool connected = false;
  bool stopped = false;
  int velocity_calls = 0;
  double last_linear = 0.0;
  double last_angular = 0.0;
  std::string start_device;
  std::string start_parameter_file;
  YpscRos2Odometry snapshot{};
};

bool wait_for_odom(rclcpp::executors::SingleThreadedExecutor &executor, const std::function<bool()> &ready,
                   std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (ready()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return ready();
}

} // namespace

// The test uses ROS 2 node construction APIs that can throw during setup.
// NOLINTNEXTLINE(bugprone-exception-escape)
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto controller_node = std::make_shared<rclcpp::Node>("yp_spur_controller_test");
  auto listener_node = std::make_shared<rclcpp::Node>("yp_spur_listener_test");
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(controller_node);
  executor->add_node(listener_node);

  FakeCoordinatorClient client;

  yp_spur_ros2::NodeController::Config invalid_config;
  bool rejected_missing_params = false;
  try {
    yp_spur_ros2::NodeController invalid_controller(*controller_node, client, invalid_config, false);
    (void)invalid_controller;
  } catch (const std::exception &) {
    rejected_missing_params = true;
  }
  if (!rejected_missing_params) {
    std::cerr << "controller should reject missing parameters\n";
    return 1;
  }

  yp_spur_ros2::NodeController::Config config;
  config.device = "/dev/ttyUSB0";
  config.parameter_file = "/tmp/robot.param";
  config.baudrate = 115200;
  config.cmd_vel_timeout = 0.05;
  config.publish_tf = false;
  config.max_linear_velocity = 1.0;
  config.max_angular_velocity = 2.0;
  config.odom_frame = "odom";
  config.base_frame = "base_link";

  auto controller = std::make_unique<yp_spur_ros2::NodeController>(*controller_node, client, config, false);
  if (!client.started || !client.connected) {
    std::cerr << "controller should start the coordinator client\n";
    return 1;
  }
  if (client.start_device != config.device || client.start_parameter_file != config.parameter_file) {
    std::cerr << "controller should forward startup parameters\n";
    return 1;
  }

  geometry_msgs::msg::Twist twist;
  twist.linear.x = 4.0;
  twist.angular.z = -7.0;
  controller->handle_cmd_vel(twist);
  if (client.velocity_calls != 1) {
    std::cerr << "cmd_vel should reach the coordinator once\n";
    return 1;
  }
  if (std::fabs(client.last_linear - 1.0) > 1e-9 || std::fabs(client.last_angular + 2.0) > 1e-9) {
    std::cerr << "cmd_vel should be clamped before forwarding\n";
    return 1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  controller->handle_timeout();
  if (client.velocity_calls != 2 || std::fabs(client.last_linear) > 1e-9 || std::fabs(client.last_angular) > 1e-9) {
    std::cerr << "timeout should publish a zero command\n";
    return 1;
  }

  std::promise<nav_msgs::msg::Odometry> odom_promise;
  auto odom_future = odom_promise.get_future();
  auto odom_sub = listener_node->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, [&odom_promise](const nav_msgs::msg::Odometry &msg) {
      try {
        odom_promise.set_value(msg);
      } catch (const std::future_error &) {
        return;
      }
    });
  (void)odom_sub;

  client.snapshot.valid = 1;
  client.snapshot.sequence = 42;
  client.snapshot.receive_time_ns = 123456789;
  client.snapshot.x = 1.25;
  client.snapshot.y = -0.75;
  client.snapshot.yaw = 1.0;
  client.snapshot.linear_velocity = 0.4;
  client.snapshot.angular_velocity = -0.2;

  controller->publish_odometry();

  if (wait_for_odom(
        *executor,
        [&odom_future]() { return odom_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready; },
        std::chrono::seconds(1)) == false) {
    std::cerr << "odometry should be published on the ROS 2 topic\n";
    return 1;
  }
  nav_msgs::msg::Odometry odom_msg = odom_future.get();

  if (odom_msg.header.frame_id != config.odom_frame || odom_msg.child_frame_id != config.base_frame) {
    std::cerr << "odometry should use configured frame names\n";
    return 1;
  }
  if (std::fabs(odom_msg.pose.pose.position.x - client.snapshot.x) > 1e-9 ||
      std::fabs(odom_msg.pose.pose.position.y - client.snapshot.y) > 1e-9 ||
      std::fabs(odom_msg.twist.twist.linear.x - client.snapshot.linear_velocity) > 1e-9 ||
      std::fabs(odom_msg.twist.twist.angular.z - client.snapshot.angular_velocity) > 1e-9) {
    std::cerr << "odometry values should match the coordinator snapshot\n";
    return 1;
  }

  controller.reset();
  executor->remove_node(listener_node);
  executor->remove_node(controller_node);
  rclcpp::shutdown();
  return 0;
}
