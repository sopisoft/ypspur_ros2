#ifndef YP_SPUR_ROS2_COORDINATOR_ADAPTER_H
#define YP_SPUR_ROS2_COORDINATOR_ADAPTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

typedef struct {
  const char *device;
  const char *parameter_file;
  int baudrate;
} YpscRos2Config;

typedef struct {
  uint64_t sequence;
  uint64_t receive_time_ns;
  double x;
  double y;
  double yaw;
  double linear_velocity;
  double angular_velocity;
  int valid;
} YpscRos2Odometry;

int ypsc_ros2_start(const YpscRos2Config *config);
int ypsc_ros2_set_velocity(double linear, double angular);
int ypsc_ros2_get_odometry(YpscRos2Odometry *odom);
int ypsc_ros2_stop(void);
int ypsc_ros2_is_connected(void);
const char *ypsc_ros2_last_error(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // YP_SPUR_ROS2_COORDINATOR_ADAPTER_H
