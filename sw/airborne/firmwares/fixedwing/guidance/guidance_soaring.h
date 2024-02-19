/*
 * Copyright (C) 2010 ENAC
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
 *  @file firmwares/fixedwing/guidance/guidance_soaring.h
 *  TGL based control for soaring with fixed wing vehicles, based on thesis by Tom Suys.
 *
 */

#include <math.h>
#include "paparazzi.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "firmwares/fixedwing/guidance/guidance_common.h"

// PID copied from https://github.com/pms67/PID/blob/master/PID.h

typedef struct {
    // Controller gains
	float Kp;
	float Ki;
	float Kd;

	// Derivative low-pass filter time constant
	float tau;

	// Output limits
	float output_min;
	float output_max;
	
	// Integrator limits
	float integrator_min;
	float integrator_max;

	// Sample time (in seconds)
	float t_step;

	// Controller "memory"
	float integrator;
	float prev_error;			/* Required for integrator */
	float differentiator;
	float prev_measurement;		/* Required for differentiator */

	// Controller output
	float out;
} PIDController;

typedef struct {
    float x;
    float z;
} Point;  // For creating the plane struct

typedef struct {
    // Ax + Bz + C = 0, extruded along y-axis
    float A;
    float B;
    float C;
} Plane;

// Functions

extern void guidance_soaring_init(void);
extern void guidance_soaring_loop(void);

// Settings

extern float heading_plane_setpoint;
extern float TGL_p1_x;
extern float TGL_p1_z;
extern float TGL_p2_x;
extern float TGL_p2_z;