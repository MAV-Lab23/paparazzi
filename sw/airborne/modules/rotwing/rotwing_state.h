/*
 * Copyright (C) 2023 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/rotwing/rotwing_state.h"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * This module keeps track of the current state of a rotating wing drone and desired state set by the RC or flightplan. Paramters are being scheduled in each change of a current state and desired state. Functions are defined in this module to call the actual state and desired state and set a desired state.
 */

#ifndef ROTWING_STATE_H
#define ROTWING_STATE_H

#include "std.h"

/** Rotwing States
 */
#define ROTWING_STATE_HOVER               0 // Wing is skewed in 0 degrees (quad)
#define ROTWING_STATE_LOW_ANGLE_SKEWING   1 // Wing is skewing in between 0 (quad) and 55 degrees (half skew)
#define ROTWING_STATE_HALF_SKEW           2 // Wing skew angle is fixed at 55 degrees
#define ROTWING_STATE_HIGH_ANGLE_SKEWING  3 // Wing is skewing in between 55 (half skew) and 90 degrees (fixed_wing)
#define ROTWING_STATE_FW                  4 // Wing is skewed at 90 degrees (fixed wing), hover motors have full authority
#define ROTWING_STATE_FW_HOV_MOT_IDLE     5 // Wing is skewed at 90 degrees (fixed wing), hover motors are forced to idle
#define ROTWING_STATE_FW_HOV_MOT_OFF      6 // Wing is skewed at 90 degrees (fixed wubg), hover motors are switched off

struct RotwingState {
  uint8_t current_state;
  uint8_t desired_state;
};

extern struct RotwingState rotwing_state;

extern void init_rotwing_state(void);
extern void periodic_rotwing_state(void);

#endif  // ROTWING_STATE_H
