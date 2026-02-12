#ifndef MAHONY_YAW_H
#define MAHONY_YAW_H

#include <cmath>
#include <stdint.h>
#include <algorithm> // Added for std::max safety

// Define M_PI if missing
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ==================================================================================
// SECTION 1: CALIBRATED CONFIGURATION
// ==================================================================================

// // --- Gyroscope Settings ---
const float MAHONY_YAW_GYRO_SCALE = 16.4f; // LSB/(deg/s)
const float MAHONY_YAW_GYRO_BIAS_Z = 0.71f; // Stationary bias to subtract

// --- Magnetometer Settings ---
// Hard Iron Offsets (Centering)
const float MAHONY_YAW_MAG_OFFSET_X = 175.0f;
const float MAHONY_YAW_MAG_OFFSET_Y = -422.0f;
const float MAHONY_YAW_MAG_OFFSET_Z = -45.0f;

// Soft Iron Scales (Oval correction)
const float MAHONY_YAW_MAG_SCALE_X = 1.018f;
const float MAHONY_YAW_MAG_SCALE_Y = 0.983f;
const float MAHONY_YAW_MAG_SCALE_Z = 1.000f;

// --- Filter Tuning Constants ---
const float MAHONY_YAW_MAG_GATE_LOW = 0.05f; 
const float MAHONY_YAW_MAG_GATE_HIGH = 2.00f;
const float MAHONY_YAW_MAG_WEIGHT_FALLBACK = 0.10f;

const float MAHONY_YAW_Kp_YAW = 5.0f;
const float MAHONY_YAW_Ki_YAW = 0.03f;
const float MAHONY_YAW_INT_LIMIT_YAW = 0.4f;
const float MAHONY_YAW_MAX_MAG_INNOVATION = 3.0f;

// Note: M_PI calculation done here for compile-time constant
const float MAHONY_YAW_SPIN_RATE_DISABLE_INTEGRAL = 80.0f * (M_PI / 180.0f); 

// // ==================================================================================
// // SECTION 2: GLOBAL STATE DECLARATIONS (Extern)
// // ==================================================================================

// // --- Diagnostics ---
extern float mahony_debug_magWeight;
extern float mahony_debug_magError;

// --- Processed Sensor Data ---
// extern float mahony_gx_rad, mahony_gy_rad, mahony_gz_rad;
// extern float mahony_mx_cal, mahony_my_cal, mahony_mz_cal;

// --- Output ---
extern float mahony_yaw_out;

// --- Filter State (Quaternion) ---
extern float mahony_yaw_q0, mahony_yaw_q1, mahony_yaw_q2, mahony_yaw_q3;

// --- Filter State (Integral Error) ---
// extern float mahony_e_int_x, mahony_e_int_y, mahony_e_int_z;

// ==================================================================================
// SECTION 3: FUNCTION PROTOTYPES
// ==================================================================================

// Math Helpers
float mahony_yaw_invSqrt1(float x);
void mahony_yaw_quat_rotate(float vx, float vy, float vz, float& outx, float& outy, float& outz);
void mahony_yaw_quat_rotate_conj(float vx, float vy, float vz, float& outx, float& outy, float& outz);

// Sensor Processing
void mahony_yaw_process_gyro(float gx_raw, float gy_raw, float gz_raw);
void mahony_yaw_process_mag(float mx_raw, float my_raw, float mz_raw);
// Filter Logic
void mahony_yaw_set_yaw(float yaw_rad);
void mahony_yaw_update_filter(float dt);
void mahony_yaw_compute();

#endif // MAHONY_YAW_H