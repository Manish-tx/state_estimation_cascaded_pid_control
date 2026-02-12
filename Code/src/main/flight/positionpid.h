// cascaded_pos_vel_control.h
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
extern float g_desPitchDeg;
extern float g_desRollDeg;
extern float g_desThrust;
extern float pid_x_outAcc;

 extern float pid_y_outAcc;
 extern float pid_z_outAcc;

// Initialize all PID parameters and states
void cascaded_pid_init(void);

// Reset integrators (use when switching modes or arming)
void cascaded_pid_reset(void);

// Main cascaded PID update
// Inputs: desired X,Y,Z positions (int32), dt (seconds)
// Output: acceleration commands ax, ay, az
void cascaded_pid_update(float dt);

// Convert acceleration commands to desired pitch, roll, thrust
void accelToAttitude(float ax, float ay, float az);

#ifdef __cplusplus
}
#endif