#include <pthread.h>
#include <stdio.h>
#include <string.h>

#include <ypspur/command.h>
#include <ypspur/control.h>
#include <ypspur/odometry.h>
#include <ypspur/param.h>
#include <ypspur/serial.h>
#include <ypspur/utility.h>
#include <ypspur/ypspur-coordinator.h>

#include "coordinator_adapter_internal.h"

int ypsc_ros2_start(const YpscRos2Config *config)
{
  if (!config || !config->device || !config->parameter_file) {
    set_last_error("device and parameter_file are required");
    return -1;
  }

  pthread_mutex_lock(&g_runtime.state_mutex);
  if (g_runtime.started) {
    pthread_mutex_unlock(&g_runtime.state_mutex);
    set_last_error("coordinator is already running");
    return -1;
  }
  g_runtime.started = 1;
  g_runtime.connected = 0;
  pthread_mutex_unlock(&g_runtime.state_mutex);

  hook_pre_global();
  ypsc_clear_stop_request();
  reset_odometry_snapshot();

  initialize_param_state(config);
  init_coordinate_systems();
  init_odometry();
  init_spur_command();
  install_odometry_hook();

  if (!configure_device(config)) {
    serial_close();
    ypsc_request_stop();
    join_threads();
    pthread_mutex_lock(&g_runtime.state_mutex);
    g_runtime.started = 0;
    pthread_mutex_unlock(&g_runtime.state_mutex);
    return -1;
  }

  if (!start_threads()) {
    serial_close();
    ypsc_request_stop();
    join_threads();
    pthread_mutex_lock(&g_runtime.state_mutex);
    g_runtime.started = 0;
    pthread_mutex_unlock(&g_runtime.state_mutex);
    return -1;
  }

  pthread_mutex_lock(&g_runtime.state_mutex);
  g_runtime.connected = 1;
  pthread_mutex_unlock(&g_runtime.state_mutex);

  return 0;
}

int ypsc_ros2_set_velocity(double linear, double angular)
{
  SpurUserParamsPtr spur;

  pthread_mutex_lock(&g_runtime.state_mutex);
  if (!g_runtime.connected) {
    pthread_mutex_unlock(&g_runtime.state_mutex);
    set_last_error("coordinator is not connected");
    return -1;
  }
  pthread_mutex_unlock(&g_runtime.state_mutex);

  spur = get_spur_user_param_ptr();
  pthread_mutex_lock(&spur->mutex);
  {
    double data[2];
    data[0] = linear;
    data[1] = angular;
    vel_com(data, spur);
  }
  pthread_mutex_unlock(&spur->mutex);

  return 0;
}

int ypsc_ros2_get_odometry(YpscRos2Odometry *odom)
{
  if (!odom) {
    set_last_error("odometry output pointer is null");
    return -1;
  }

  pthread_mutex_lock(&g_runtime.odometry_mutex);
  *odom = g_runtime.odometry;
  pthread_mutex_unlock(&g_runtime.odometry_mutex);

  return odom->valid ? 0 : -1;
}

int ypsc_ros2_stop(void)
{
  SpurUserParamsPtr spur = get_spur_user_param_ptr();

  pthread_mutex_lock(&g_runtime.state_mutex);
  if (!g_runtime.started) {
    pthread_mutex_unlock(&g_runtime.state_mutex);
    return 0;
  }
  g_runtime.connected = 0;
  pthread_mutex_unlock(&g_runtime.state_mutex);

  pthread_mutex_lock(&spur->mutex);
  spur->run_mode = RUN_STOP;
  spur->vref = 0;
  spur->wref = 0;
  pthread_mutex_unlock(&spur->mutex);

  {
    double control_cycle = p(YP_PARAM_CONTROL_CYCLE, 0);
    if (control_cycle <= 0) {
      control_cycle = 0.02;
    }
    yp_usleep((int)(control_cycle * 2000000.0));
  }

  ypsc_request_stop();
  join_threads();
  stop_motors();
  serial_close();
  ypsc_clear_stop_request();

  pthread_mutex_lock(&g_runtime.state_mutex);
  g_runtime.started = 0;
  pthread_mutex_unlock(&g_runtime.state_mutex);

  return 0;
}

int ypsc_ros2_is_connected(void)
{
  int connected;

  pthread_mutex_lock(&g_runtime.state_mutex);
  connected = g_runtime.connected;
  pthread_mutex_unlock(&g_runtime.state_mutex);

  return connected;
}

const char *ypsc_ros2_last_error(void)
{
  return g_runtime.last_error;
}
