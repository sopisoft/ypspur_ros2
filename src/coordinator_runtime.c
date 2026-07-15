#include <math.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ypspur/control.h>
#include <ypspur/odometry.h>
#include <ypspur/param.h>
#include <ypspur/utility.h>
#include <ypspur/ypspur-coordinator.h>

#include "coordinator_adapter_internal.h"

SpurUserParams g_spur;
YpscRos2Runtime g_runtime = {
  .state_mutex = PTHREAD_MUTEX_INITIALIZER,
  .odometry_mutex = PTHREAD_MUTEX_INITIALIZER,
};

SpurUserParamsPtr get_spur_user_param_ptr()
{
  return &g_spur;
}

void init_spur_command(void)
{
  memset(&g_spur, 0, sizeof(g_spur));
  g_spur.run_mode = RUN_STOP;
  g_spur.before_run_mode = -10000;
  pthread_mutex_init(&g_spur.mutex, NULL);
}

void ypsc_set_odometry_hook(OdometryHook fn)
{
  set_odometry_hook(fn);
}

void set_last_error(const char *fmt, ...)
{
  va_list ap;

  pthread_mutex_lock(&g_runtime.state_mutex);
  va_start(ap, fmt);
  vsnprintf(g_runtime.last_error, sizeof(g_runtime.last_error), fmt, ap);
  va_end(ap);
  pthread_mutex_unlock(&g_runtime.state_mutex);
}

void reset_odometry_snapshot(void)
{
  pthread_mutex_lock(&g_runtime.odometry_mutex);
  memset(&g_runtime.odometry, 0, sizeof(g_runtime.odometry));
  pthread_mutex_unlock(&g_runtime.odometry_mutex);
}

void copy_odometry_snapshot(const OdometryPtr odm)
{
  if (!odm)
    return;

  pthread_mutex_lock(&g_runtime.odometry_mutex);
  g_runtime.odometry.sequence++;
  g_runtime.odometry.receive_time_ns = (uint64_t)(odm->time * 1000000000.0);
  g_runtime.odometry.x = odm->x;
  g_runtime.odometry.y = odm->y;
  g_runtime.odometry.yaw = odm->theta;
  g_runtime.odometry.linear_velocity = odm->v;
  g_runtime.odometry.angular_velocity = odm->w;
  g_runtime.odometry.valid = 1;
  pthread_mutex_unlock(&g_runtime.odometry_mutex);
}

static void odometry_hook(const OdometryPtr odm, const ErrorStatePtr err)
{
  (void)err;
  copy_odometry_snapshot(odm);
}

void install_odometry_hook(void)
{
  ypsc_set_odometry_hook(odometry_hook);
}

void *control_thread_main(void *arg)
{
  (void)arg;
  control_loop();
  return NULL;
}

void *odometry_thread_main(void *arg)
{
  (void)arg;
  odometry_receive_loop();
  return NULL;
}
