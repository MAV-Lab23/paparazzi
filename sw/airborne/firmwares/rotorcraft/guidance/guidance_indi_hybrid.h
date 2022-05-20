/*
 * Copyright (C) 2015 Ewoud Smeur <ewoud.smeur@gmail.com>
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
 */

/**
 * @file firmwares/rotorcraft/guidance/guidance_indi_hybrid.h
 *
 * A guidance mode based on Incremental Nonlinear Dynamic Inversion
 * Come to ICRA2016 to learn more!
 *
 */

#ifndef GUIDANCE_INDI_HYBRID_H
#define GUIDANCE_INDI_HYBRID_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "filters/high_pass_filter.h"

extern void guidance_indi_enter(void);
extern void guidance_indi_run(float *heading_sp);
extern void stabilization_attitude_set_setpoint_rp_quat_f(struct FloatEulers* indi_rp_cmd, bool in_flight, int32_t heading);
extern void guidance_indi_init(void);
extern void guidance_indi_propagate_filters(void);

struct guidance_indi_hybrid_params {
  float pos_gain;
  float pos_gainz;
  float speed_gain;
  float speed_gainz;
  float heading_bank_gain;
};

extern float hybrid_pitch_limit;

extern struct guidance_indi_hybrid_params gih_params;
extern float guidance_indi_specific_force_gain;
extern float guidance_indi_max_airspeed;
extern float nav_max_speed;
extern bool take_heading_control;
extern float guidance_indi_max_bank;

extern float lift_pitch_eff;
extern float pitch_priority_factor;
extern float roll_priority_factor;
extern float thrust_priority_factor;
extern float pusher_priority_factor;
extern float fwd_sideslip_g;
extern float min_accel;
extern bool pusher_slowdown;
extern bool div_push;
extern float Wv_z;
extern bool theta_slowdown;
extern bool thrust_slowdown;

extern float pitch_pref_deg;
extern int chirp_number;
extern float phi_ref_c;
extern float theta_ref_c;
extern float psi_ref_c;
extern struct NedCoor_f pos_ref_c;
extern struct FloatVect3 speed_ref_c;
extern struct FloatVect3 acc_body_ref_c;
extern struct FloatVect3 acc_body_c;

extern float proj_wp_dist;
extern bool roll_zero;
#endif /* GUIDANCE_INDI_HYBRID_H */
