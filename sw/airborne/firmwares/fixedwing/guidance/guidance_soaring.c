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
 *  @file firmwares/fixedwing/guidance/guidance_soaring.c
 *  TGL based control for soaring with fixed wing vehicles, based on thesis by Tom Suys.
 *
 */

#include "firmwares/fixedwing/guidance/guidance_soaring.h"
#include "state.h"
#include "firmwares/fixedwing/nav.h"
#include "generated/airframe.h"
#include "autopilot.h"

#include <stdlib.h>
#include <math.h>

//? Setup

// General
struct EnuCoor_f* current_pos;

// For yaw control
PIDController yaw_controller;
float y_setpoint;
float rudder_output;

// For pitch control
PIDController pitch_controller;
float pitch_output;
float tgl_error;
// Create TGL plane
Point p1;
Point p2;
Plane tgl_plane;

// Settings
float heading_plane_setpoint;
float TGL_p1_x;
float TGL_p1_z;
float TGL_p2_x;
float TGL_p2_z;
// For setpoints, X is actually Y

//? Helper functions

float calcTGLDistance(float x, float z, Plane tgl_plane) {
    // Calculates perpendicular distance from point to TGL plane, with correct sign
    float tgl_distance = fabs(tgl_plane.A * x + tgl_plane.B * z + tgl_plane.C) / ((tgl_plane.A * tgl_plane.A) + (tgl_plane.B * tgl_plane.B));
    tgl_distance = (tgl_plane.A * x + tgl_plane.B * z + tgl_plane.C) > 0 ? -tgl_distance : tgl_distance; //! Sign may be reversed
    return tgl_distance;
}

Plane createPlane(Point p1, Point p2) {
    Plane plane;
    plane.A = p2.z - p1.z;
    plane.B = p1.x - p2.x;
    plane.C = -plane.A * p1.x - plane.B * p1.z;
    return plane;
}

void pidControllerReset(PIDController *pid) {
	// Initialize controller variables
    pid->Kp = 0.0f;
    pid->Ki = 0.0f;
    pid->Kd = 0.0f;
    pid->tau = 0.0f;
    pid->output_min = -INFINITY;
    pid->output_max = INFINITY;
    pid->integrator_min = -INFINITY;
    pid->integrator_max = INFINITY;
	pid->integrator = 0.0f;
	pid->prev_error  = 0.0f;
	pid->differentiator  = 0.0f;
	pid->prev_measurement = 0.0f;
	pid->out = 0.0f;
}

float pidControllerUpdate(PIDController *pid, float setpoint, float measurement) {
    float error = setpoint - measurement;

    float proportional = pid->Kp * error;

    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->t_step * (error + pid->prev_error);
	// Anti-wind-up via integrator clamping
    if (pid->integrator > pid->integrator_max) {
        pid->integrator = pid->integrator_max;
    } else if (pid->integrator < pid->integrator_min) {
        pid->integrator = pid->integrator_min;
    }

	// Derivative band-limited differentiator
    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prev_measurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * pid->tau - pid->t_step) * pid->differentiator)
                        / (2.0f * pid->tau + pid->t_step);

	// Compute output and apply limits
    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->output_max) {
        pid->out = pid->output_max;
    } else if (pid->out < pid->output_min) {
        pid->out = pid->output_min;
    }

    pid->prev_error       = error;
    pid->prev_measurement = measurement;

    return pid->out;
}

void guidance_soaring_init(void) {


    // Init PI controller for yaw
    pidControllerReset(&yaw_controller);
    yaw_controller.Kp = 0.01;
    yaw_controller.Ki = 0.001;
    yaw_controller.t_step = 0.01;
    yaw_controller.output_max = 10 / 180 * M_PI; // [rad]
    yaw_controller.output_min = -yaw_controller.output_max;

    // Init PI controller for pitch
    pidControllerReset(&pitch_controller);
    pitch_controller.Kp = -0.01;
    pitch_controller.Ki = -0.001;
    pitch_controller.t_step = 0.01;
    pitch_controller.output_max = 30 / 180 * M_PI;  // [rad]
    pitch_controller.output_min = -pitch_controller.output_max;
}

void guidance_soaring_loop(void) {
    // Update settings
    y_setpoint = heading_plane_setpoint;
    p1.x = TGL_p1_x;
    p1.z = TGL_p1_z;
    p2.x = TGL_p2_x;
    p2.z = TGL_p2_z;
    tgl_plane = createPlane(p1, p2);

    // Get current position
    current_pos = stateGetPositionEnu_f();

    // Yaw control
    rudder_output = pidControllerUpdate(&yaw_controller, y_setpoint, current_pos->x);  // Changed current_pos->y to x to account for coord sys
    h_ctl_rudder_setpoint = rudder_output;

    // Roll control
    h_ctl_roll_setpoint = 0;

    // Pitch control

    // TGL plane defined by a line and extruded along the y-axis
    // Pitch down when behind the plane, pitch up when in front of the plane
    tgl_error = calcTGLDistance(current_pos->y, current_pos->z, tgl_plane);  // Changed x to y
    pitch_output = pidControllerUpdate(&pitch_controller, 0, tgl_error);
    h_ctl_pitch_setpoint = pitch_output;
}