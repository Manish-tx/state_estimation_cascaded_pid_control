#include "mahony_yaw.h"
#include "../API/API-Utils.h"
#include "../API/Debugging.h"
// #include <algorithm> // For std::max
// #include <iostream>  // For console output

// // ==================================================================================
// // SECTION 1: GLOBAL VARIABLE DEFINITIONS
// // ==================================================================================

// Filter State
float mahony_yaw_q0 = 1.0f;
float mahony_yaw_q1 = 0.0f;
float mahony_yaw_q2 = 0.0f;
float mahony_yaw_q3 = 0.0f;

// Integral Error
float mahony_yaw_e_int_x = 0.0f;
float mahony_yaw_e_int_y = 0.0f;
float mahony_yaw_e_int_z = 0.0f;

// Diagnostics
float mahony_debug_magWeight = 0.0f;
float mahony_debug_magError = 0.0f;

// Processed Sensor Data
float mahony_yaw_gx_rad = 0.0f;
float mahony_yaw_gy_rad = 0.0f;
float mahony_yaw_gz_rad = 0.0f;

float mahony_yaw_mx_cal = 0.0f;
float mahony_yaw_my_cal = 0.0f;
float mahony_yaw_mz_cal = 0.0f;

// Output
float mahony_yaw_out = 0.0f;

// // ==================================================================================
// // SECTION 2: MATH HELPERS IMPLEMENTATION
// // ==================================================================================

float mahony_yaw_invSqrt1(float x) {
    if (x <= 0.0f) return 0.0f;
    return 1.0f / sqrtf(x);
}

// Rotate Body -> Earth (q * v * q*)
void mahony_yaw_quat_rotate(float vx, float vy, float vz, 
                        float& outx, float& outy, float& outz) {
    float q0 = mahony_yaw_q0, q1 = mahony_yaw_q1, q2 = mahony_yaw_q2, q3 = mahony_yaw_q3;
    
    float rx = q1 * q1 + q0 * q0 - q2 * q2 - q3 * q3;
    float ry = 2 * (q1 * q2 - q0 * q3);
    float rz = 2 * (q1 * q3 + q0 * q2);
    
    float ux = 2 * (q1 * q2 + q0 * q3);
    float uy = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
    float uz = 2 * (q2 * q3 - q0 * q1);
    
    float fx = 2 * (q1 * q3 - q0 * q2);
    float fy = 2 * (q2 * q3 + q0 * q1);
    float fz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    
    outx = rx*vx + ry*vy + rz*vz;
    outy = ux*vx + uy*vy + uz*vz;
    outz = fx*vx + fy*vy + fz*vz;
}

// Rotate Earth -> Body (q* * v * q)
void mahony_yaw_quat_rotate_conj(float vx, float vy, float vz, 
                             float& outx, float& outy, float& outz) {
    float q0 = mahony_yaw_q0, q1 = mahony_yaw_q1, q2 = mahony_yaw_q2, q3 = mahony_yaw_q3;
    
    float rx = q1 * q1 + q0 * q0 - q2 * q2 - q3 * q3;
    float ry = 2 * (q1 * q2 + q0 * q3);
    float rz = 2 * (q1 * q3 - q0 * q2);
    
    float ux = 2 * (q1 * q2 - q0 * q3);
    float uy = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
    float uz = 2 * (q2 * q3 + q0 * q1);
    
    float fx = 2 * (q1 * q3 + q0 * q2);
    float fy = 2 * (q2 * q3 - q0 * q1);
    float fz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    
    outx = rx*vx + ux*vy + fx*vz;
    outy = ry*vx + uy*vy + fy*vz;
    outz = rz*vx + uz*vy + fz*vz;
}

// ==================================================================================
// SECTION 3: SENSOR PROCESSING IMPLEMENTATION
// ==================================================================================

void mahony_yaw_process_gyro(float gx_raw, float gy_raw, float gz_raw) {

    mahony_yaw_gx_rad = (gx_raw / MAHONY_YAW_GYRO_SCALE) * (M_PI / 180.0f);
    mahony_yaw_gy_rad = (gy_raw / MAHONY_YAW_GYRO_SCALE) * (M_PI / 180.0f);
    
    // Convert to Rad/s and handle bias
    mahony_yaw_gz_rad = -1.0f * ((gz_raw - MAHONY_YAW_GYRO_BIAS_Z) / MAHONY_YAW_GYRO_SCALE) * (M_PI / 180.0f);
}

void mahony_yaw_process_mag(float mx_raw, float my_raw, float mz_raw) {
    // 1. Apply Hard Iron Offsets
    float mx = mx_raw - MAHONY_YAW_MAG_OFFSET_X;
    float my = my_raw - MAHONY_YAW_MAG_OFFSET_Y;
    float mz = mz_raw - MAHONY_YAW_MAG_OFFSET_Z;

    // 2. Apply Soft Iron Scaling
    mahony_yaw_mx_cal = mx * MAHONY_YAW_MAG_SCALE_X;
    mahony_yaw_my_cal = my * MAHONY_YAW_MAG_SCALE_Y;
    mahony_yaw_mz_cal = mz * MAHONY_YAW_MAG_SCALE_Z;
}

// ==================================================================================
// SECTION 4: FILTER CORE LOGIC IMPLEMENTATION
// ==================================================================================

void mahony_yaw_set_yaw(float yaw_rad) {
    mahony_yaw_q0 = cosf(yaw_rad / 2.0f);
    mahony_yaw_q1 = 0.0f;
    mahony_yaw_q2 = 0.0f;
    mahony_yaw_q3 = sinf(yaw_rad / 2.0f);
    
    mahony_yaw_e_int_x = 0.0f; 
    mahony_yaw_e_int_y = 0.0f; 
    mahony_yaw_e_int_z = 0.0f;
}

void mahony_yaw_update_filter(float dt) {
    // 1. Check DT
    if (dt <= 0.0f || dt > 0.5f) return;

    // Local aliases
    float mx = mahony_yaw_mx_cal;
    float my = mahony_yaw_my_cal;
    float mz = mahony_yaw_mz_cal;
    float gx = mahony_yaw_gx_rad;
    float gy = mahony_yaw_gy_rad;
    float gz = mahony_yaw_gz_rad;

    // 2. Normalize Magnetometer (Body Frame)
    float m_sq = mx*mx + my*my + mz*mz;
    // Using 1e-9f safety floor for division
    float m_inv = 1.0f / std::max(sqrtf(m_sq), 1e-9f);
    float mx_n = mx * m_inv;
    float my_n = my * m_inv;
    float mz_n = mz * m_inv;

    // 3. Rotate to Earth Frame (Body -> Earth)
    float mex, mey, mez;
    mahony_yaw_quat_rotate(mx_n, my_n, mz_n, mex, mey, mez);

    // 4. Extract Horizontal Component
    float mh_x = mex;
    float mh_y = mey;

    float mh_mag = sqrtf(mh_x*mh_x + mh_y*mh_y);
    float mh_inv = 1.0f / std::max(mh_mag, 1e-9f);
    float mh_x_n = mh_x * mh_inv;
    float mh_y_n = mh_y * mh_inv;

    // 5. Gating & Weight
    float magWeight = 0.0f;
    if (mh_mag >= MAHONY_YAW_MAG_GATE_LOW && mh_mag <= MAHONY_YAW_MAG_GATE_HIGH) {
        magWeight = 1.0f;
    } else {
        magWeight = MAHONY_YAW_MAG_WEIGHT_FALLBACK;
    }

    // 6. Compute Innovation (Error in Earth Frame)
    float e_earth_z = -mh_y_n; 

    // 7. Reject Large Spikes
    float e_mag = fabsf(e_earth_z);
    if (e_mag * magWeight > MAHONY_YAW_MAX_MAG_INNOVATION) {
        magWeight = 0.0f;
    }

    mahony_debug_magWeight = magWeight;
    mahony_debug_magError = e_mag;

    // 8. Rotate Error Back to Body Frame
    float ex_body, ey_body, ez_body;
    float e_earth_x = 0.0f, e_earth_y = 0.0f;
    mahony_yaw_quat_rotate_conj(e_earth_x, e_earth_y, e_earth_z, ex_body, ey_body, ez_body);

    // 9. Correction Terms
    float w_Pz = ez_body * MAHONY_YAW_Kp_YAW;

    // 10. Integrator (Yaw)
    float spin_rate = sqrtf(gx*gx + gy*gy + gz*gz);
    if (spin_rate < MAHONY_YAW_SPIN_RATE_DISABLE_INTEGRAL) {
        mahony_yaw_e_int_x += ex_body * MAHONY_YAW_Ki_YAW * dt;
        mahony_yaw_e_int_y += ey_body * MAHONY_YAW_Ki_YAW * dt;
        mahony_yaw_e_int_z += ez_body * MAHONY_YAW_Ki_YAW * dt;

        // Anti-Windup
        float e_int_norm = sqrtf(mahony_yaw_e_int_x*mahony_yaw_e_int_x + mahony_yaw_e_int_y*mahony_yaw_e_int_y + mahony_yaw_e_int_z*mahony_yaw_e_int_z);
        if (e_int_norm > MAHONY_YAW_INT_LIMIT_YAW) {
            float scale = MAHONY_YAW_INT_LIMIT_YAW / e_int_norm;
            mahony_yaw_e_int_x *= scale;
            mahony_yaw_e_int_y *= scale;
            mahony_yaw_e_int_z *= scale;
        }
    }

    // 11. Combine Corrections (Only Z affected)
    float corr_z = w_Pz * magWeight + mahony_yaw_e_int_z * magWeight;

    float gx_final = gx; 
    float gy_final = gy;
    float gz_final = gz + corr_z; 

    // 12. Integrate Quaternion
    float qa = mahony_yaw_q0, qb = mahony_yaw_q1, qc = mahony_yaw_q2;
    mahony_yaw_q0 += 0.5f * (-qb * gx_final - qc * gy_final - mahony_yaw_q3 * gz_final) * dt;
    mahony_yaw_q1 += 0.5f * (qa * gx_final + qc * gz_final - mahony_yaw_q3 * gy_final) * dt;
    mahony_yaw_q2 += 0.5f * (qa * gy_final - qb * gz_final + mahony_yaw_q3 * gx_final) * dt;
    mahony_yaw_q3 += 0.5f * (qa * gz_final + qb * gy_final - qc * gx_final) * dt;

    // Normalize
    float recipNorm = mahony_yaw_invSqrt1(mahony_yaw_q0*mahony_yaw_q0 + mahony_yaw_q1*mahony_yaw_q1 + mahony_yaw_q2*mahony_yaw_q2 + mahony_yaw_q3*mahony_yaw_q3);
    mahony_yaw_q0 *= recipNorm; 
    mahony_yaw_q1 *= recipNorm; 
    mahony_yaw_q2 *= recipNorm; 
    mahony_yaw_q3 *= recipNorm;
}

void mahony_yaw_compute() {
    // Yaw Calculation
    float siny_cosp = 2.0f * (mahony_yaw_q0 * mahony_yaw_q3 + mahony_yaw_q1 * mahony_yaw_q2);
    float cosy_cosp = 1.0f - 2.0f * (mahony_yaw_q2 * mahony_yaw_q2 + mahony_yaw_q3 * mahony_yaw_q3);
    mahony_yaw_out = atan2f(siny_cosp, cosy_cosp);
}