/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/loggers/file_logger.c
 *  @brief File logger for Linux based autopilots
 */

#include "file_logger.h"

#include <stdio.h>
#include "std.h"

#include "subsystems/imu.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "state.h"

// #include "subsystems/electrical.h"
#include "modules/computer_vision/video_capture.h"
// optical flow module and textons module:
#include "modules/ctrl/optical_flow_landing.h"
#include "modules/computer_vision/textons.h"

#include "modules/computer_vision/video_thread.h"
// reading the pressuremeter:
#include "subsystems/abi.h"
#ifndef LOGGER_BARO_ID
#define LOGGER_BARO_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(LOGGER_BARO_ID)
float logger_pressure;
static abi_event baro_ev; ///< The baro ABI event
/// Callback function of the ground altitude
static void logger_baro_cb(uint8_t sender_id __attribute__((unused)), float pressure);
// Reading from "sensors":
static void logger_baro_cb(uint8_t sender_id, float pressure)
{
  logger_pressure = pressure;
}

// reading the pressuremeter:
//#include "subsystems/abi.h"
#ifndef LOGGER_SONAR_ID
#define LOGGER_SONAR_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(LOGGER_SONAR_ID)
float logger_sonar;
static abi_event sonar_ev; ///< The sonar ABI event
/// Callback function of the ground altitude
static void logger_sonar_cb(uint8_t sender_id __attribute__((unused)), float height);
// Reading from "sensors":
static void logger_sonar_cb(uint8_t sender_id, float height)
{
  logger_sonar = height;
}

// reading OF info:
#ifndef LOGGER_OF_ID
#define LOGGER_OF_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(LOGGER_OF_ID)
uint32_t OF_stamp;
int16_t OF_flow_x;
int16_t OF_flow_y;
int16_t OF_flow_der_x;
int16_t OF_flow_der_y;
float OF_quality;
float OF_size_divergence;
static abi_event OF_ev; ///< The sonar ABI event
static void logger_optical_flow_cb(uint8_t sender_id __attribute__((unused)),uint32_t stamp, int16_t flow_x,
                                   int16_t flow_y, int16_t flow_der_x, int16_t flow_der_y, float quality, float size_divergence);

static void logger_optical_flow_cb(uint8_t sender_id, uint32_t stamp, int16_t flow_x,
                                   int16_t flow_y, int16_t flow_der_x, int16_t flow_der_y, float quality, float size_divergence)
{
  OF_stamp = stamp;
  OF_flow_x = flow_x;
  OF_flow_y = flow_y;
  OF_flow_der_x = flow_der_x;
  OF_flow_der_y = flow_der_y;
  OF_quality = quality;
  OF_size_divergence = size_divergence;
}

// reading RPMs:
#ifndef LOGGER_RPM_ID
#define LOGGER_RPM_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(LOGGER_RPM_ID)
uint16_t RPM[8]; // max an octocopter
uint8_t RPM_num_act;
static abi_event RPM_ev; ///< The sonar ABI event
static void logger_rpm_cb(uint8_t sender_id, uint16_t * rpm, uint8_t num_act);

static void logger_rpm_cb(uint8_t sender_id, uint16_t * rpm, uint8_t num_act)
{
  RPM_num_act = num_act;
  for(int i = 0; i < num_act; i++) {
      RPM[i] = rpm[i];
  }
}


// timing the video snapshots:
struct timeval stop, start;
//float time_stamp = 0;
float prev_ss_time = 0;
int take_shot = 0;
int shots = 0;

/** Set the default File logger path to the USB drive */
#ifndef FILE_LOGGER_PATH
#define FILE_LOGGER_PATH /data/video/usb
#endif

/** The file pointer */
static FILE *file_logger = NULL;

/** Start the file logger and open a new file */
void file_logger_start(void)
{

  uint32_t counter = 0;
  char filename[512];
  shots = 0;

  // Check for available files
  sprintf(filename, "%s/%05d.csv", STRINGIFY(FILE_LOGGER_PATH), counter);
  printf("Logging to file name = %s\n", filename);
  while ((file_logger = fopen(filename, "r"))) {
    fclose(file_logger);

    counter++;
    sprintf(filename, "%s/%05d.csv", STRINGIFY(FILE_LOGGER_PATH), counter);
  }

  // start timer:
  gettimeofday(&start, 0);

  file_logger = fopen(filename, "w");

  /*char str[n_textons * 4];
  char add_str[4];
  for(int i = 0; i < n_textons-1; i++)
  {
      sprintf(add_str, "t%2d,", i);
  }
  sprintf(add_str, "t%2d", n_textons-1);
  strcat(str, add_str);
  printf("\n\n\nTexton entries: %s\n\n\n", str);
  */

  if (file_logger != NULL) {
    fprintf(
          file_logger,
          "time,accel_x,accel_y,accel_z,gyro_p,gyro_q,gyro_r,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,att_phi,att_theta,att_psi,rate_p,rate_q,rate_r,OF_time,flow_x, flow_y,flow_der_x, flow_der_y, div_size, div,noise, fps,cmd_thrust,cmd_roll,cmd_pitch,cmd_yaw,rpm1_abi,rpm2_abi,rpm3_abi,rpm4_abi,num_act"
        );
  }
  else {
      printf("Could not open log!\n");
  }

  // Subscribe to the altitude above ground level ABI messages
  AbiBindMsgBARO_ABS(LOGGER_BARO_ID, &baro_ev, logger_baro_cb);
  AbiBindMsgAGL(LOGGER_SONAR_ID, &sonar_ev, logger_sonar_cb);
  AbiSendMsgOPTICAL_FLOW(LOGGER_OF_ID, &OF_ev, logger_optical_flow_cb);
  AbiBindMsgRPM(LOGGER_RPM_ID, &RPM_ev, logger_rpm_cb);
}

/** Stop the logger an nicely close the file */
void file_logger_stop(void)
{
  if (file_logger != NULL) {
    fclose(file_logger);
    file_logger = NULL;
  }
}

/** Log the values to a csv file */
void file_logger_periodic(void)
{
  if (file_logger == NULL) {
    return;
  }
  static uint32_t counter;
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();

  //timing
/*  gettimeofday(&stop, 0);
  double curr_time = (double)(stop.tv_sec + stop.tv_usec / 1000000.0);
  double time_stamp = curr_time - (double)(start.tv_sec + start.tv_usec / 1000000.0);
  if((time_stamp - prev_ss_time) > 0.2) // for 5hz
  {
    video_capture_shoot();
    prev_ss_time = time_stamp;
    take_shot = shots;
    shots +=1;
  }
  else
  {
    take_shot = -1;
  }*/

  struct FloatEulers *eulers = stateGetNedToBodyEulers_f();
  struct NedCoor_f *velocities = stateGetSpeedNed_f();
  struct NedCoor_f *position = stateGetPositionNed_f();
  struct FloatRates *rates = stateGetBodyRates_f();

  float GT_divergence = 0.0f;
  if(position->z >= 1E-3) {
      GT_divergence = velocities->z / position->z;
  }
  float noise = 0.0f; // what was this?

  fprintf(file_logger, "%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%d,%f,%f,%f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
          counter,
/*
	  imu.accel_unscaled.x,
	  imu.accel_unscaled.y,
	  imu.accel_unscaled.z,
	  imu.gyro_unscaled.p,
	  imu.gyro_unscaled.q,
	  imu.gyro_unscaled.r,
*/
          imu.accel.x, // m/s^2 scaled with 10
          imu.accel.y,
          imu.accel.z,
	  imu.gyro.p, // rad/s scaled with 12
	  imu.gyro.q,
	  imu.gyro.r,
          position->x,
	  position->y,
	  position->z,
	  velocities->x,
	  velocities->y,
	  velocities->z,
	  eulers->phi,
	  eulers->theta,
	  eulers->psi,
	  rates->p,
	  rates->q,
	  rates->r,
	  OF_stamp,
	  OF_flow_x,
	  OF_flow_y,
	  OF_flow_der_x,
	  OF_flow_der_y,
	  OF_quality,
	  OF_size_divergence,
	  GT_divergence,
	  noise,
	  vid->fps,
          stabilization_cmd[COMMAND_THRUST],
          stabilization_cmd[COMMAND_ROLL],
          stabilization_cmd[COMMAND_PITCH],
          stabilization_cmd[COMMAND_YAW],
	  RPM[0],
	  RPM[1],
	  RPM[2],
	  RPM[3],
	  RPM_num_act
         );

  counter++;
}
