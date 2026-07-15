#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <yp_spur_ros2/node_controller.h>

namespace
{

yp_spur_ros2::NodeController::Config read_config(rclcpp::Node &node)
{
  node.declare_parameter<std::string>("device", "");
  node.declare_parameter<std::string>("parameter_file", "");
  node.declare_parameter<int64_t>("baudrate", 115200);
  node.declare_parameter<double>("cmd_vel_timeout", 0.5);
  node.declare_parameter<double>("odom_publish_rate", 30.0);
  node.declare_parameter<double>("max_linear_velocity", 1.0);
  node.declare_parameter<double>("max_angular_velocity", 2.0);
  node.declare_parameter<bool>("publish_tf", true);
  node.declare_parameter<std::string>("odom_frame", "odom");
  node.declare_parameter<std::string>("base_frame", "base_link");

  yp_spur_ros2::NodeController::Config config;
  config.device = node.get_parameter("device").as_string();
  config.parameter_file = node.get_parameter("parameter_file").as_string();
  config.baudrate = static_cast<int>(node.get_parameter("baudrate").as_int());
  config.cmd_vel_timeout = node.get_parameter("cmd_vel_timeout").as_double();
  config.odom_publish_rate = node.get_parameter("odom_publish_rate").as_double();
  config.max_linear_velocity = std::abs(node.get_parameter("max_linear_velocity").as_double());
  config.max_angular_velocity = std::abs(node.get_parameter("max_angular_velocity").as_double());
  config.publish_tf = node.get_parameter("publish_tf").as_bool();
  config.odom_frame = node.get_parameter("odom_frame").as_string();
  config.base_frame = node.get_parameter("base_frame").as_string();
  return config;
}

} // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<rclcpp::Node>("yp_spur_node");
    yp_spur_ros2::CApiCoordinatorClient client;
    auto config = read_config(*node);
    auto controller = std::make_shared<yp_spur_ros2::NodeController>(*node, client, config);
    rclcpp::spin(node);
    controller.reset();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("yp_spur_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
