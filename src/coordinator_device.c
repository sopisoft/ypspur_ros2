#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ypspur/adinput.h>
#include <ypspur/command.h>
#include <ypspur/control.h>
#include <ypspur/param.h>
#include <ypspur/ping.h>
#include <ypspur/serial.h>
#include <ypspur/ypprotocol.h>

#include "coordinator_adapter_internal.h"

int initialize_param_state(const YpscRos2Config *config)
{
  ParametersPtr param = get_param_ptr();

  memset(param, 0, sizeof(*param));
  strncpy(param->device_name, config->device, sizeof(param->device_name) - 1);
  strncpy(param->parameter_filename, config->parameter_file, sizeof(param->parameter_filename) - 1);
  param->option = OPTION_NO_IPC | OPTION_WITHOUT_SSM | OPTION_PARAM_FILE;
  param->output_lv = OUTPUT_LV_INFO;
  param->msq_key = YPSPUR_MSQ_KEY;
  param->speed = config->baudrate;
  param->ssm_id = 0;
  return 1;
}

static int load_embedded_parameter_file(char *resolved_filename, size_t resolved_len)
{
  char param_buffer[2048];
  FILE *temp_paramfile = tmpfile();

  if (!temp_paramfile) {
    set_last_error("failed to create temporary file for embedded parameters");
    return 0;
  }

  if (!get_embedded_param(param_buffer)) {
    fclose(temp_paramfile);
    set_last_error("failed to read embedded parameters from device");
    return 0;
  }

  fputs(param_buffer, temp_paramfile);
  rewind(temp_paramfile);

  if (!set_paramptr(temp_paramfile)) {
    fclose(temp_paramfile);
    set_last_error("failed to load embedded parameter file");
    return 0;
  }

  snprintf(resolved_filename, resolved_len, "embedded");
  fclose(temp_paramfile);
  return 1;
}

int load_parameter_file(const YpscRos2Config *config, char *resolved_filename, size_t resolved_len)
{
  if (strcmp(config->parameter_file, "embedded") == 0) {
    return load_embedded_parameter_file(resolved_filename, resolved_len);
  }

  if (!set_param((char *)config->parameter_file, resolved_filename)) {
    set_last_error("failed to load parameter file: %s", config->parameter_file);
    return 0;
  }

  return 1;
}

int configure_device(const YpscRos2Config *config)
{
  Ver_t version;
  Param_t driver_param;
  char paramfile[512];
  int i;

  if (!serial_connect((char *)config->device)) {
    set_last_error("failed to connect to device: %s", config->device);
    return 0;
  }

  for (i = 0; i < 3; i++) {
    int current, age;
    int device_current, device_age;

    if (get_version(&version) < 0)
      continue;
    if (strstr(version.protocol, "YPP:") != version.protocol)
      continue;

    sscanf(YP_PROTOCOL_NAME, "YPP:%d:%d", &current, &age);
    sscanf(version.protocol, "YPP:%d:%d", &device_current, &device_age);

    get_param_ptr()->device_version = device_current;
    get_param_ptr()->device_version_age = device_age;

    if (device_current - device_age > current || device_current < current - age)
      continue;
    break;
  }

  if (i == 3) {
    set_last_error("device protocol is not compatible with coordinator protocol");
    return 0;
  }

  if (get_parameter(&driver_param) < 0) {
    set_last_error("failed to read driver parameters from device");
    return 0;
  }

  if (!load_parameter_file(config, paramfile, sizeof(paramfile))) {
    return 0;
  }

  if (strlen(driver_param.pwm_resolution) > 0) {
    int pwm = atoi(driver_param.pwm_resolution);
    for (i = 0; i < YP_PARAM_MAX_MOTOR_NUM; i++) {
      *pp(YP_PARAM_PWM_MAX, i) = pwm;
    }
  } else if (p(YP_PARAM_PWM_MAX, 0) == 0) {
    set_last_error("failed to read PWM resolution from device and parameter file");
    return 0;
  }

  if (get_param_ptr()->speed <= 0) {
    set_last_error("baudrate must be specified");
    return 0;
  }

  if (set_baudrate(get_param_ptr()->speed) == 0) {
    set_last_error("failed to set baudrate to %d", get_param_ptr()->speed);
    return 0;
  }

  if (get_param_ptr()->admask) {
    set_admask(get_param_ptr()->admask);
  }

  if (apply_robot_params() < 1) {
    set_last_error("failed to apply robot parameters");
    return 0;
  }

  {
    SpurUserParamsPtr spur = get_spur_user_param_ptr();
    spur->v = p(YP_PARAM_MAX_VEL, 0);
    spur->w = p(YP_PARAM_MAX_W, 0);
    spur->dv = p(YP_PARAM_MAX_ACC_V, 0);
    spur->dw = p(YP_PARAM_MAX_ACC_W, 0);
    spur->run_mode = RUN_STOP;
    for (i = 0; i < YP_PARAM_MAX_MOTOR_NUM; i++) {
      spur->wheel_mode[i] = MOTOR_CONTROL_VEL;
      spur->wheel_mode_prev[i] = -1;
    }
  }

  return 1;
}

int start_threads(void)
{
  if (pthread_create(&g_runtime.control_thread, NULL, control_thread_main, NULL) != 0) {
    set_last_error("failed to start control thread");
    return 0;
  }
  g_runtime.control_thread_started = 1;

  if (pthread_create(&g_runtime.odometry_thread, NULL, odometry_thread_main, NULL) != 0) {
    set_last_error("failed to start odometry thread");
    return 0;
  }
  g_runtime.odometry_thread_started = 1;
  return 1;
}

void stop_motors(void)
{
  int i;
  ParametersPtr param = get_param_ptr();

  for (i = 0; i < YP_PARAM_MAX_MOTOR_NUM; i++) {
    if (param->motor_enable[i]) {
      parameter_set(PARAM_servo, i, SERVO_LEVEL_STOP);
    }
  }
}

void join_threads(void)
{
  if (g_runtime.control_thread_started) {
    pthread_join(g_runtime.control_thread, NULL);
    g_runtime.control_thread_started = 0;
  }
  if (g_runtime.odometry_thread_started) {
    pthread_join(g_runtime.odometry_thread, NULL);
    g_runtime.odometry_thread_started = 0;
  }
}
