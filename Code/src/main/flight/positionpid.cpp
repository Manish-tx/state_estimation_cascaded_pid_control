// cascaded_pos_vel_control.cpp

#include "positionpid.h"
#include <math.h>
#include <common/maths.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "rctoposition.h"
#include "ekf.h"

#include "../API/Debugging.h"
#include "../API/API-Utils.h"
// External EKF values (declare from your system)

 float pid_x_outAcc;
 float pid_y_outAcc;
 float pid_z_outAcc;

float g_desPitchDeg = 0.0f;
float g_desRollDeg  = 0.0f;
float g_desThrust   = 1.0f;
// ---- Utility macros ----
#define CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

typedef struct {
    float kp, ki, kd;
    float out_min, out_max;
    float integrator_min, integrator_max;
} pid_params_f;

typedef struct {
    float integrator;
    float prev_measurement;
    float prev_error;
} pid_state_f;

// PID instances for each axis
static pid_params_f pid_pos_x, pid_pos_y, pid_pos_z;
static pid_params_f pid_vel_x, pid_vel_y, pid_vel_z;

static pid_state_f state_pos_x, state_pos_y, state_pos_z;
static pid_state_f state_vel_x, state_vel_y, state_vel_z;

// Limits
static const float POS_TO_VEL_LIMIT = 100.0f;
static const float VEL_TO_ACC_LIMIT = 200.0f;

static const float ACC_XY_MAX = 3.0f;
static const float ACC_Z_MAX  = 9.0f;
 float posTargetZ_m;
// ---------------------------------------------------------------------
// Internal function: initialize a PID parameter structure
// ---------------------------------------------------------------------
static void init_pid(pid_params_f *p,
                     float kp, float ki, float kd,
                     float minOut, float maxOut,
                     float minI, float maxI)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->out_min = minOut;
    p->out_max = maxOut;
    p->integrator_min = minI;
    p->integrator_max = maxI;
}

// ---------------------------------------------------------------------
// Reset all PID states
// ---------------------------------------------------------------------
void cascaded_pid_reset(void)
{
    state_pos_x = {0};
    state_pos_y = {0};
    state_pos_z = {0};

    state_vel_x = {0};
    state_vel_y = {0};
    state_vel_z = {0};
}

// ---------------------------------------------------------------------
// Initialize cascaded PID parameters
// ---------------------------------------------------------------------
void cascaded_pid_init(void)
{
    init_pid(&pid_pos_x, 0.12f, 0.01f, 0.0f, -100, 100, -200, 200);
    init_pid(&pid_pos_y, 0.12f, 0.0f, 0.0f, -100, 100, -200, 200);
    init_pid(&pid_pos_z, 4.0f, 0.0f, 0.1f, -100, 100, -1000, 1000);

    init_pid(&pid_vel_x, 0.1f, 0.0f, 1.0f, -200, 200, -500, 500);
    init_pid(&pid_vel_y, 0.1f, 0.0f, 1.0f, -200, 200, -500, 500);
    init_pid(&pid_vel_z, 3.0f, 0.0f, 0.05f, -200, 200, -1000, 1000);

    cascaded_pid_reset();
}

// ---------------------------------------------------------------------
// Internal PID step (derivative on measurement)
// ---------------------------------------------------------------------
static float pid_step(pid_params_f *p, pid_state_f *s,
                      float error, float measurement, float dt)
{
    if (dt <= 0) return 0;

    float P = p->kp * error;

    // Integrator with clamp
    s->integrator += (p->ki * error * dt);
    s->integrator = CLAMP(s->integrator, p->integrator_min, p->integrator_max);

    float I = s->integrator;

    // Derivative on measurement
    float dMeas = (measurement - s->prev_measurement) / dt;
    float D = -p->kd * dMeas;

    s->prev_measurement = measurement;
    s->prev_error = error;

    float out = P + I + D;
    if (out < p->out_max && out > p->out_min) {
        s->integrator += p->ki * error * dt;
    }
    return CLAMP(out, p->out_min, p->out_max);
}

// ---------------------------------------------------------------------
// Cascaded Position → Velocity → Acceleration PID
// ---------------------------------------------------------------------
void cascaded_pid_update(float dt)
{
    // --- Read current states ---
    float posZ = ekfOf_pz;      // Only Z position is used
    float velZ = ekfOf_vz;

    float velX = 0;
    float velY = 0;

    // if(abs(posZ - posTargetZ_m/100) < 0.35)
    // {
    //     // velX = ekfOf_vx;
    //     // velY = ekfOf_vy;
    //     pid_pos_z.kp = 5;
    //     pid_pos_z.ki = 0.02;
    //     pid_vel_z.kp = 2;
    // } 
    // else{
    //     pid_pos_z.kp = 20;
    //     pid_vel_z.kp = 5;
    // }

    // -------------------------------------------------------------
    //      X & Y AXIS: NO POSITION PID
    // -------------------------------------------------------------
    // Desired velocities are always ZERO (to hold position)
    float desVelX = 0.0f;
    float desVelY = 0.0f;

    // Velocity errors for XY
    float err_vel_x = desVelX - velX;
    float err_vel_y = desVelY - velY;

    // Run ONLY the velocity PID in X and Y
    float accX = pid_step(&pid_vel_x, &state_vel_x, err_vel_x, velX, dt);
    float accY = pid_step(&pid_vel_y, &state_vel_y, err_vel_y, velY, dt);

    // Limit XY acceleration
    accX = CLAMP(accX, -ACC_XY_MAX, ACC_XY_MAX);
    accY = CLAMP(accY, -ACC_XY_MAX, ACC_XY_MAX);

    // -------------------------------------------------------------
    //      Z AXIS: FULL CASCADED POSITION → VELOCITY PID
    // -------------------------------------------------------------
    float desZ = posTargetZ_m/100;  // your global target

    float err_pos_z = desZ - posZ;

    // Outer loop (position → desired velocity)
    float desVelZ = pid_step(&pid_pos_z, &state_pos_z, err_pos_z, posZ, dt);
    desVelZ = CLAMP(desVelZ, -POS_TO_VEL_LIMIT, POS_TO_VEL_LIMIT);

    // Inner loop (velocity → acceleration)
    float err_vel_z = desVelZ - velZ;
    float accZ = pid_step(&pid_vel_z, &state_vel_z, err_vel_z, velZ, dt);

    // Clamp Z acceleration
    accZ = CLAMP(accZ, -ACC_Z_MAX, ACC_Z_MAX);

    // Output vector
    pid_x_outAcc = accX;
    pid_y_outAcc = accY;
    pid_z_outAcc = accZ;

    static uint32_t lastTime1 = 0;
    uint32_t now1 = micros();

    // Update every 30 ms (33 Hz)
    if (now1 - lastTime1 >= 30000) {
        lastTime1 = now1;
        // Monitor_Print("posZ:",posZ);
        // Monitor_Print("velZ:",velZ);
        // Monitor_Print("INAccZ:",accZ);
    }
}

// ---------------------------------------------------------------------
// Convert (ax, ay, az) → (pitch, roll, thrust)
// Small-angle hover approx — replace with your drone dynamics
// ---------------------------------------------------------------------
void accelToAttitude(float ax, float ay, float az)
{
    const float g = 9.80665f;

    float pitchCmdDeg;
    float rollCmdDeg;
    float thrustCmd;


    // Convert acceleration to tilt angles
    float pitch = atan2f(ax, g);     // pitch forward/back
    float roll  = -atan2f(ay, g);    // roll left/right

    // Convert rad → deg
    pitchCmdDeg = pitch * 180.0f / 3.1415;
    rollCmdDeg  = roll  * 180.0f / 3.1415;

    // Thrust: compensate gravity
    float thrustNorm = (g + az)/g;   // 1.0 = hover
    thrustNorm = constrainf(thrustNorm*1500, 1000, 2000);

    thrustCmd = thrustNorm;
    g_desPitchDeg = pitchCmdDeg;
    g_desRollDeg  = rollCmdDeg;
    g_desThrust   = thrustNorm;
}