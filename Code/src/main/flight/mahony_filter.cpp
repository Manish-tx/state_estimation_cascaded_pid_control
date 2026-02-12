/*
 * mahony_flight_controller.cpp
 * Implementation of procedural logic and Variable Definition
 */

#include "mahony_filter.h"
#include "../API/Debugging.h"

// ==================================================================================
// VARIABLE DEFINITIONS (Allocating memory for externs)
// ==================================================================================

// --- Filter State (Quaternion) - Initialized to identity ---
float mahony_q0 = 1.0f;
float mahony_q1 = 0.0f;
float mahony_q2 = 0.0f;
float mahony_q3 = 0.0f;

// --- Filter State (Integral Error) ---
float mahony_e_int_x = -0.1f;
float mahony_e_int_y = -0.01f;
float mahony_e_int_z = -0.01f;

// --- Sensor Output State ---
float mahony_ax_mps2 = 0.0f, mahony_ay_mps2 = 0.0f, mahony_az_mps2 = 0.0f;
float mahony_gx_rad = 0.0f,  mahony_gy_rad = 0.0f,  mahony_gz_rad = 0.0f;

// --- Final Attitude State ---
float mahony_roll  = 0.0f;
float mahony_pitch = 0.0f;
float mahony_yaw   = 0.0f;

// --- Optical Flow State ---
float mahony_vel_x_world = 0.0f;
float mahony_vel_y_world = 0.0f;

// ==================================================================================
// HELPER FUNCTIONS
// ==================================================================================

float mahony_invSqrt(float x) {
    return 1.0f / sqrtf(x);
}

// ==================================================================================
// SENSOR BRIDGE LOGIC
// ==================================================================================

void mahony_process_imu(float ax_raw, float ay_raw, float az_raw, 
                        float gx_raw, float gy_raw, float gz_raw) {
    
    // Accel: LSB -> g -> m/s^2 (Store directly in global state)
    mahony_ax_mps2 = (ax_raw / MAHONY_ACCEL_SCALE) * 9.80665f;
    mahony_ay_mps2 = (ay_raw / MAHONY_ACCEL_SCALE) * 9.80665f;
    mahony_az_mps2 = (az_raw / MAHONY_ACCEL_SCALE) * 9.80665f;

    // Gyro: LSB -> deg/s -> rad/s (Store directly in global state)
    mahony_gx_rad = (gx_raw / MAHONY_GYRO_SCALE) * (M_PI / 180.0f);
    mahony_gy_rad = (gy_raw / MAHONY_GYRO_SCALE) * (M_PI / 180.0f);
    mahony_gz_rad = (gz_raw / MAHONY_GYRO_SCALE) * (M_PI / 180.0f);
}

float mahony_process_range(float range_mm) {
    float distance_m = range_mm / 1000.0f;

    // Reject invalid ranges
    if (distance_m > 4.0f || distance_m < 0.02f) return -1.0f; 

    // TILT CORRECTION (Uses global roll/pitch)
    float tilt_angle = sqrt(mahony_roll*mahony_roll + mahony_pitch*mahony_pitch);
    return distance_m * cos(tilt_angle);
}

void mahony_process_flow(float flow_x, float flow_y, float altitude_m, float dt) {
    float flow_rate_x = flow_x / dt;
    float flow_rate_y = flow_y / dt;

    // Gyro Compensation (Uses global gyro data)
    float comp_flow_x = flow_rate_x - (mahony_gy_rad / MAHONY_FLOW_GYRO_COMP_FACTOR);
    float comp_flow_y = flow_rate_y - (mahony_gx_rad / MAHONY_FLOW_GYRO_COMP_FACTOR);

    float safe_alt = (altitude_m < 0.1f) ? 0.1f : altitude_m;

    // Store in global velocity state
    mahony_vel_x_world = comp_flow_x * safe_alt * MAHONY_FLOW_PIXEL_SCALER;
    mahony_vel_y_world = comp_flow_y * safe_alt * MAHONY_FLOW_PIXEL_SCALER;
}

// ==================================================================================
// MAHONY FILTER LOGIC
// ==================================================================================

// Updates global quaternion state based on global sensor state
void mahony_update_filter(float dt) {
    // 1. Sanity Check
    if (dt <= 0.0f || dt > 0.005f) return;

    // Local aliases for readability
    float ax = mahony_ax_mps2;
    float ay = mahony_ay_mps2;
    float az = mahony_az_mps2;
    float gx = mahony_gx_rad;
    float gy = mahony_gy_rad;
    float gz = mahony_gz_rad;

    // 2. Analyze Accelerometer Health
    float acc_mag = sqrtf(ax*ax + ay*ay + az*az);
    if (acc_mag < 1e-9f) return;

    float ax_norm = ax / acc_mag;
    float ay_norm = ay / acc_mag;
    float az_norm = az / acc_mag;

    float acc_mag_norm = acc_mag / MAHONY_G;

    // 3. Gating Logic
    float accWeight;
    if (acc_mag_norm >= MAHONY_ACC_GATE_LOW && acc_mag_norm <= MAHONY_ACC_GATE_HIGH) {
        accWeight = 1.0f;
    } else {
        accWeight = MAHONY_ACC_WEIGHT_FALLBACK;
    }

    // 4. Prediction Step (Rotate Earth-Z by Quaternion)
    float vx = 2.0f * (mahony_q1 * mahony_q3 - mahony_q0 * mahony_q2);
    float vy = 2.0f * (mahony_q0 * mahony_q1 + mahony_q2 * mahony_q3);
    float vz = mahony_q0 * mahony_q0 - mahony_q1 * mahony_q1 - mahony_q2 * mahony_q2 + mahony_q3 * mahony_q3;

    // 5. Error Calculation
    float ex = ay_norm * vz - az_norm * vy;
    float ey = az_norm * vx - ax_norm * vz;
    float ez = ax_norm * vy - ay_norm * vx;
    
    float e_mag = sqrtf(ex*ex + ey*ey + ez*ez);

    // 6. Outlier Rejection
    if (e_mag * accWeight > MAHONY_MAX_INNOVATION) {
        accWeight = 0.0f;
    }

    // 7. Calculate Proportional Correction
    float omega_Px = ex * MAHONY_Kp;
    float omega_Py = ey * MAHONY_Kp;
    float omega_Pz = ez * MAHONY_Kp;

    // 8. Calculate Integral Correction
    float spin_rate = sqrtf(gx*gx + gy*gy + gz*gz);
    float spin_limit_rad = MAHONY_SPIN_RATE_LIMIT_DEG * (M_PI / 180.0f);

    if (spin_rate < spin_limit_rad) {
        mahony_e_int_x += ex * MAHONY_Ki * dt;
        mahony_e_int_y += ey * MAHONY_Ki * dt;
        mahony_e_int_z += ez * MAHONY_Ki * dt;
    }

    // Anti-Windup
    float e_int_norm = sqrtf(mahony_e_int_x*mahony_e_int_x + mahony_e_int_y*mahony_e_int_y + mahony_e_int_z*mahony_e_int_z);
    if (e_int_norm > MAHONY_INT_LIMIT) {
        float scale = MAHONY_INT_LIMIT / e_int_norm;
        mahony_e_int_x *= scale;
        mahony_e_int_y *= scale;
        mahony_e_int_z *= scale;
    }

    // 9. Apply Corrections
    float correction_x = omega_Px * accWeight + mahony_e_int_x * accWeight;
    float correction_y = omega_Py * accWeight + mahony_e_int_y * accWeight;
    float correction_z = omega_Pz * accWeight + mahony_e_int_z * accWeight;

    // Apply to Gyro
    float gx_corr = gx + correction_x;
    float gy_corr = gy + correction_y;
    float gz_corr = gz + correction_z;

    // 10. CRITICAL: Yaw Lock (Decouple Yaw)
    gz_corr = gz; 

    // 11. Integrate Quaternion (Directly update globals)
    float qa = mahony_q0, qb = mahony_q1, qc = mahony_q2;
    mahony_q0 += 0.5f * (-qb * gx_corr - qc * gy_corr - mahony_q3 * gz_corr) * dt;
    mahony_q1 += 0.5f * (qa * gx_corr + qc * gz_corr - mahony_q3 * gy_corr) * dt;
    mahony_q2 += 0.5f * (qa * gy_corr - qb * gz_corr + mahony_q3 * gx_corr) * dt;
    mahony_q3 += 0.5f * (qa * gz_corr + qb * gy_corr - qc * gx_corr) * dt;

    // 12. Normalize Quaternion
    float recipNorm = mahony_invSqrt(mahony_q0*mahony_q0 + mahony_q1*mahony_q1 + mahony_q2*mahony_q2 + mahony_q3*mahony_q3);
    mahony_q0 *= recipNorm;
    mahony_q1 *= recipNorm;
    mahony_q2 *= recipNorm;
    mahony_q3 *= recipNorm;
}

void mahony_compute_euler() {
    // Roll
    float sinr_cosp = 2.0f * (mahony_q0 * mahony_q1 + mahony_q2 * mahony_q3);
    float cosr_cosp = 1.0f - 2.0f * (mahony_q1 * mahony_q1 + mahony_q2 * mahony_q2);
    mahony_roll = atan2f(sinr_cosp, cosr_cosp);

    // Pitch
    float sinp = 2.0f * (mahony_q0 * mahony_q2 - mahony_q3 * mahony_q1);
    if (fabs(sinp) >= 1.0f)
        mahony_pitch = copysign(M_PI / 2.0f, sinp);
    else
        mahony_pitch = asinf(sinp);

    // Yaw
    float siny_cosp = 2.0f * (mahony_q0 * mahony_q3 + mahony_q1 * mahony_q2);
    float cosy_cosp = 1.0f - 2.0f * (mahony_q2 * mahony_q2 + mahony_q3 * mahony_q3);
    mahony_yaw = atan2f(siny_cosp, cosy_cosp);
}