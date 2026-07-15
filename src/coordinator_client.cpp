#include <string>

#include <yp_spur_ros2/coordinator_client.h>

namespace yp_spur_ros2
{

bool CApiCoordinatorClient::start(const YpscRos2Config &config, std::string &error)
{
  if (ypsc_ros2_start(&config) != 0) {
    error = ypsc_ros2_last_error();
    return false;
  }
  return true;
}

bool CApiCoordinatorClient::set_velocity(double linear, double angular, std::string &error)
{
  if (ypsc_ros2_set_velocity(linear, angular) != 0) {
    error = ypsc_ros2_last_error();
    return false;
  }
  return true;
}

bool CApiCoordinatorClient::get_odometry(YpscRos2Odometry &odom)
{
  return ypsc_ros2_get_odometry(&odom) == 0;
}

bool CApiCoordinatorClient::stop()
{
  return ypsc_ros2_stop() == 0;
}

bool CApiCoordinatorClient::is_connected() const
{
  return ypsc_ros2_is_connected() != 0;
}

} // namespace yp_spur_ros2
