#ifndef YP_SPUR_ROS2_COORDINATOR_ADAPTER_INTERNAL_H
#define YP_SPUR_ROS2_COORDINATOR_ADAPTER_INTERNAL_H

#include <pthread.h>
#include <stddef.h>
#include <stdint.h>

#include <ypspur/command.h>
#include <ypspur/odometry.h>

#include <yp_spur_ros2/coordinator_adapter.h>

typedef struct {
  pthread_t control_thread;
  pthread_t odometry_thread;
  int control_thread_started;
  int odometry_thread_started;
  int started;
  int connected;
  pthread_mutex_t state_mutex;
  pthread_mutex_t odometry_mutex;
  YpscRos2Odometry odometry;
  char last_error[256];
} YpscRos2Runtime;

extern SpurUserParams g_spur;
extern YpscRos2Runtime g_runtime;

void ypsc_set_odometry_hook(OdometryHook fn);
void init_spur_command(void);
void set_last_error(const char *fmt, ...);
void reset_odometry_snapshot(void);
void copy_odometry_snapshot(const OdometryPtr odm);
void install_odometry_hook(void);
void *control_thread_main(void *arg);
void *odometry_thread_main(void *arg);
int initialize_param_state(const YpscRos2Config *config);
int load_parameter_file(const YpscRos2Config *config, char *resolved_filename, size_t resolved_len);
int configure_device(const YpscRos2Config *config);
int start_threads(void);
void stop_motors(void);
void join_threads(void);

#endif // YP_SPUR_ROS2_COORDINATOR_ADAPTER_INTERNAL_H
