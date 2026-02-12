/*
 * mahony_flight_controller.h
 * Global Variable Declarations and Configuration
 */

#ifndef MAHONY_FLIGHT_CONTROLLER_H
#define MAHONY_FLIGHT_CONTROLLER_H

#include <cmath>
#include <stdint.h>

// Define M_PI if the environment doesn't provide it
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ==================================================================================
// SECTION 1: GLOBAL CONFIGURATION CONSTANTS
// ==================================================================================

// IMU Configuration (ICM-20948)
const float MAHONY_ACCEL_SCALE = 4096.0f; 
const float MAHONY_GYRO_SCALE  = 16.4f;

// Optical Flow Configuration (PAW3903)
const float MAHONY_FLOW_GYRO_COMP_FACTOR = 0.003f;
const float MAHONY_FLOW_PIXEL_SCALER     = 0.0015f; 

// Filter Tuning Constants
const float MAHONY_G = 9.80665f;
const float MAHONY_ACC_GATE_LOW = 0.70f;
const float MAHONY_ACC_GATE_HIGH = 1.45f;
const float MAHONY_ACC_WEIGHT_FALLBACK = 0.15f;

// const float MAHONY_Kp = 1.0f;
// const float MAHONY_Ki = 0.02f;
const float MAHONY_Kp = 1.2f;
const float MAHONY_Ki = 0.005f;
const float MAHONY_INT_LIMIT = 0.4f;
// Note: Calculations using M_PI must happen in runtime or be constexpr. 
// For C-style simplicity, we define the base value here.
const float MAHONY_SPIN_RATE_LIMIT_DEG = 80.0f; 
const float MAHONY_MAX_INNOVATION = 3.0f;

// ==================================================================================
// SECTION 2: GLOBAL STATE VARIABLES (Extern Declarations)
// ==================================================================================

// --- Filter State (Quaternion) ---
extern float mahony_q0;
extern float mahony_q1;
extern float mahony_q2;
extern float mahony_q3;

// --- Filter State (Integral Error) ---
extern float mahony_e_int_x;
extern float mahony_e_int_y;
extern float mahony_e_int_z;

// --- Sensor Output State ---
extern float mahony_ax_mps2, mahony_ay_mps2, mahony_az_mps2;
extern float mahony_gx_rad,  mahony_gy_rad,  mahony_gz_rad;

// --- Final Attitude State ---
extern float mahony_roll;
extern float mahony_pitch;
extern float mahony_yaw;

// --- Optical Flow State ---
extern float mahony_vel_x_world;
extern float mahony_vel_y_world;

// ==================================================================================
// SECTION 3: FUNCTION PROTOTYPES
// ==================================================================================

// Helper
float mahony_invSqrt(float x);

// Sensor Bridge
void mahony_process_imu(float ax_raw, float ay_raw, float az_raw, 
                        float gx_raw, float gy_raw, float gz_raw);
float mahony_process_range(float range_mm);
void mahony_process_flow(float flow_x, float flow_y, float altitude_m, float dt);

// Filter Logic
void mahony_update_filter(float dt);
void mahony_compute_euler();

#endif // MAHONY_FLIGHT_CONTROLLER_H