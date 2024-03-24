/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/computer_vision/video_capture.c
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>

#include "modules/computer_vision/video_capture.h"
#include "modules/computer_vision/cv.h"

#include "lib/encoding/jpeg.h"

// Note: this define is set automatically when the video_exif module is included,
// and exposes functions to write data in the image exif headers.
#if JPEG_WITH_EXIF_HEADER
#include "lib/exif/exif_module.h"
#endif

#ifndef VIDEO_CAPTURE_PATH
#define VIDEO_CAPTURE_PATH /data/video/images
#endif

#ifndef VIDEO_CAPTURE_JPEG_QUALITY
#define VIDEO_CAPTURE_JPEG_QUALITY 99
#endif

#ifndef VIDEO_CAPTURE_FPS
#define VIDEO_CAPTURE_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(VIDEO_CAPTURE_FPS)








#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "mcu_periph/sys_time.h"
#include "state.h"
#include "generated/airframe.h"
#ifdef COMMAND_THRUST
#include "firmwares/rotorcraft/stabilization.h"
#else
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "firmwares/fixedwing/stabilization/stabilization_adaptive.h"
#endif

#include "generated/modules.h"


/** The file pointer */
static FILE *logger_file = NULL;


/** Logging functions */

/** Write CSV header
 * Write column names at the top of the CSV file. Make sure that the columns
 * match those in logger_file_write_row! Don't forget the \n at the end of the
 * line.
 * @param file Log file pointer
 */
static void custom_logger_file_write_header(FILE *file) {
  #ifdef VIDEO_CONCURRENT_LOG
  fprintf(file, "time,");
  fprintf(file, "pos_x,pos_y,pos_z,");
  fprintf(file, "vel_x,vel_y,vel_z,");
  fprintf(file, "att_phi,att_theta,att_psi,");
  fprintf(file, "rate_p,rate_q,rate_r,");
  fprintf(file, "rmat11,ramat12,ramt13,rmat21,ramat22,ramt23,rmat31,ramat32,ramt33\n");
#ifdef BOARD_BEBOP
  //fprintf(file, "rpm_obs_1,rpm_obs_2,rpm_obs_3,rpm_obs_4,");
  //fprintf(file, "rpm_ref_1,rpm_ref_2,rpm_ref_3,rpm_ref_4,");
#endif
#ifdef INS_EXT_POSE_H
  //ins_ext_pos_log_header(file);
#endif
#ifdef COMMAND_THRUST
  //fprintf(file, "cmd_thrust,cmd_roll,cmd_pitch,cmd_yaw\n");
#else
  //fprintf(file, "h_ctl_aileron_setpoint,h_ctl_elevator_setpoint\n");
#endif

  #endif
}


static char csv_dir[256];
static char date_time[80];
static char filename[512];

/** Write CSV row
 * Write values at this timestamp to log file. Make sure that the printf's match
 * the column headers of logger_file_write_header! Don't forget the \n at the
 * end of the line.
 * @param file Log file pointer
 */
static void custom_logger_file_write_row(FILE *file, uint32_t image_timestamp) {
  #ifdef VIDEO_CONCURRENT_LOG
  struct NedCoor_f *pos = stateGetPositionNed_f();
  struct NedCoor_f *vel = stateGetSpeedNed_f();
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  struct FloatRates *rates = stateGetBodyRates_f();

  fprintf(file, "%d,", image_timestamp);
  fprintf(file, "%f,%f,%f,", pos->x, pos->y, pos->z);
  fprintf(file, "%f,%f,%f,", vel->x, vel->y, vel->z);
  fprintf(file, "%f,%f,%f,", att->phi, att->theta, att->psi);
  fprintf(file, "%f,%f,%f,", rates->p, rates->q, rates->r);

  struct FloatRMat *ned_to_body_rmat = stateGetNedToBodyRMat_f();
  fprintf(file, "%f,%f,%f,%f,%f,%f,%f,%f,%f\n", ned_to_body_rmat->m[0],ned_to_body_rmat->m[1],ned_to_body_rmat->m[2],ned_to_body_rmat->m[3],ned_to_body_rmat->m[4],ned_to_body_rmat->m[5],ned_to_body_rmat->m[6],ned_to_body_rmat->m[7],ned_to_body_rmat->m[8]);


#ifdef BOARD_BEBOP
  //fprintf(file, "%d,%d,%d,%d,",actuators_bebop.rpm_obs[0],actuators_bebop.rpm_obs[1],actuators_bebop.rpm_obs[2],actuators_bebop.rpm_obs[3]);
  //fprintf(file, "%d,%d,%d,%d,",actuators_bebop.rpm_ref[0],actuators_bebop.rpm_ref[1],actuators_bebop.rpm_ref[2],actuators_bebop.rpm_ref[3]);
#endif
#ifdef INS_EXT_POSE_H
  //ins_ext_pos_log_data(file);
#endif
#ifdef COMMAND_THRUST
  //fprintf(file, "%d,%d,%d,%d\n",
      //stabilization_cmd[COMMAND_THRUST], stabilization_cmd[COMMAND_ROLL],
      //stabilization_cmd[COMMAND_PITCH], stabilization_cmd[COMMAND_YAW]);
#else
  //fprintf(file, "%d,%d\n", h_ctl_aileron_setpoint, h_ctl_elevator_setpoint);
#endif

  fflush(file);
  #endif
}


/** Start the file logger and open a new file */
void custom_logger_file_start()
{
  #ifdef VIDEO_CONCURRENT_LOG
  // Create output folder if necessary
  if (access(csv_dir, F_OK)) {
    char save_dir_cmd[266]; // write 10b + [0:256]
    sprintf(save_dir_cmd, "mkdir -p %s", csv_dir);
    if (system(save_dir_cmd) != 0) {
      printf("[video_capture] Could not create images directory %s.\n", csv_dir);
      return;
    }
  }

  uint32_t counter = 0;

  // Check for available files
  sprintf(filename, "%s/%s.csv", csv_dir, date_time);
  while ((logger_file = fopen(filename, "r"))) {
    fclose(logger_file);

    sprintf(filename, "%s/%s_%05d.csv", csv_dir, date_time, counter);
    counter++;
  }

  logger_file = fopen(filename, "w");
  if(!logger_file) {
    printf("[logger_file] ERROR opening log file %s!\n", filename);
    return;
  }

  printf("[logger_file] Start logging to %s...\n", filename);

  custom_logger_file_write_header(logger_file);
  #endif
}

/** Stop the logger an nicely close the file */
void custom_logger_file_stop(void)
{
  #ifdef VIDEO_CONCURRENT_LOG
  if (logger_file != NULL) {
    fclose(logger_file);
    logger_file = NULL;
  }
  #endif
}

/** Log the values to a csv file    */
void custom_logger_file_periodic(uint32_t image_timestamp)
{
  #ifdef VIDEO_CONCURRENT_LOG
  if (logger_file == NULL) {
    printf("[logger_file] ERROR opening log file %s!\n", filename);
    return;
  }
  custom_logger_file_write_row(logger_file, image_timestamp);

  #endif
}


















// Module settings
bool video_capture_take_shot = false; // Capture single images
bool video_capture_record_video = false; // Capture video
bool stopped_capture = false;
int video_capture_index = 0;

// Save directory
static char save_dir[256];

// Forward function declarations
struct image_t *video_capture_func(struct image_t *img, uint8_t camera_id);
void video_capture_save(struct image_t *img);


void video_capture_init(void)
{
  // Create images directory with timestamp
  struct timeval tv;
  struct tm *tm;
  gettimeofday(&tv, NULL);
  tm = localtime(&tv.tv_sec);

  sprintf(save_dir, "%s/%04d%02d%02d-%02d%02d%02d", STRINGIFY(VIDEO_CAPTURE_PATH),
      tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
      tm->tm_hour, tm->tm_min, tm->tm_sec);

  #ifdef VIDEO_CONCURRENT_LOG
  sprintf(date_time, "%04d%02d%02d-%02d%02d%02d", tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
      tm->tm_hour, tm->tm_min, tm->tm_sec);

  sprintf(csv_dir, "%s/%04d%02d%02d-%02d%02d%02d", STRINGIFY(VIDEO_CAPTURE_PATH),
      tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
      tm->tm_hour, tm->tm_min, tm->tm_sec);
  #endif
  // Folder creation delayed until capture starts, see video_capture_save.
  // This prevents empty folders if nothing is actually recorded.

  // Add function to computer vision pipeline
  cv_add_to_device(&VIDEO_CAPTURE_CAMERA, video_capture_func, VIDEO_CAPTURE_FPS, 0);
}


struct image_t *video_capture_func(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  // If take_shot bool is set, save the image
  if (video_capture_take_shot || video_capture_record_video) {
    video_capture_save(img);
    video_capture_take_shot = false;
  }

  #ifdef VIDEO_CONCURRENT_LOG
  if (stopped_capture) {
    custom_logger_file_stop();
  }
  #endif

  // No modification to image
  return NULL;
}


void video_capture_shoot(void)
{
  // Set take_shot bool to true
  video_capture_take_shot = true;
}

void video_capture_start_capture(void) {
  video_capture_record_video = true;
}

void video_capture_stop_capture(void) {
  video_capture_record_video = false;
  stopped_capture = true;
}

void video_capture_save(struct image_t *img)
{
  static bool created_logger_file = false;
  // Create output folder if necessary
  if (access(save_dir, F_OK)) {

    char save_dir_cmd[266]; // write 10b + [0:256]
    sprintf(save_dir_cmd, "mkdir -p %s", save_dir);
    if (system(save_dir_cmd) != 0) {
      printf("[video_capture] Could not create images directory %s.\n", save_dir);
      return;
    }
  }
  if (!created_logger_file) {
    #ifdef VIDEO_CONCURRENT_LOG
    custom_logger_file_start();
    created_logger_file = true;
    #endif
  }

  // Declare storage for image location
  char save_name[266]; // write 10b + [0-256]

  // Generate image filename from image timestamp
  sprintf(save_name, "%s/%u.jpg", save_dir, img->pprz_ts);
  printf("[video_capture] Saving image to %s.\n", save_name);

  #ifdef VIDEO_CONCURRENT_LOG
  custom_logger_file_periodic(img->pprz_ts);
  #endif

  // Create jpg image from raw frame
  struct image_t img_jpeg;
  image_create(&img_jpeg, img->w, img->h, IMAGE_JPEG);
  jpeg_encode_image(img, &img_jpeg, VIDEO_CAPTURE_JPEG_QUALITY, true);

#if JPEG_WITH_EXIF_HEADER
  write_exif_jpeg(save_name, img_jpeg.buf, img_jpeg.buf_size, img_jpeg.w, img_jpeg.h);
#else
  // Open file
  FILE *fp = fopen(save_name, "w");
  if (fp == NULL) {
    printf("[video_capture] Could not write shot %s.\n", save_name);
    return;
  }

  // Save it to the file and close it
  fwrite(img_jpeg.buf, sizeof(uint8_t), img_jpeg.buf_size, fp);
  fclose(fp);
#endif

  // Free image
  image_free(&img_jpeg);
}
