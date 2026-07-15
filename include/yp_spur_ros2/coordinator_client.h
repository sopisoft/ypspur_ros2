#ifndef YP_SPUR_ROS2_COORDINATOR_CLIENT_H
#define YP_SPUR_ROS2_COORDINATOR_CLIENT_H

#include <string>

#include <yp_spur_ros2/coordinator_adapter.h>

namespace yp_spur_ros2
{

class CoordinatorClient
{
public:
  virtual ~CoordinatorClient() = default;

  virtual bool start(const YpscRos2Config &config, std::string &error) = 0;
  virtual bool set_velocity(double linear, double angular, std::string &error) = 0;
  virtual bool get_odometry(YpscRos2Odometry &odom) = 0;
  virtual bool stop() = 0;
  virtual bool is_connected() const = 0;
};

class CApiCoordinatorClient final : public CoordinatorClient
{
public:
  bool start(const YpscRos2Config &config, std::string &error) override;
  bool set_velocity(double linear, double angular, std::string &error) override;
  bool get_odometry(YpscRos2Odometry &odom) override;
  bool stop() override;
  bool is_connected() const override;
};

} // namespace yp_spur_ros2

#endif // YP_SPUR_ROS2_COORDINATOR_CLIENT_H
