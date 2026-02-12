#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <float.h> // for FLT_EPSILON
#include "flight/ekf.h"

// ==========================================================================
// USER CONFIGURATION (TUNE THESE FIRST)
// ==========================================================================

// 1. DIVERGENCE CONTROL
#define ENABLE_BIAS_ESTIMATION  0 

// 2. FLOW SENSOR ORIENTATION (BODY FRAME ALIGNMENT)
static const bool SWAP_FLOW_XY    = false; 
static const bool INVERT_FLOW_X   = false; 
static const bool INVERT_FLOW_Y   = false;

// 3. SENSOR SPECS
static const float ACCEL_SCALE_FACTOR = 1; 

static const float H_TOF_MAX          = 4.0f; 
static const float H_TOF_MARGIN       = 0.5f;

// Minimum height to run flow math
#define MIN_PZ            0.20f // 20cm

// ==========================================================================
// GLOBAL OUTPUTS
// ==========================================================================
float ekfOf_px = 0.0f, ekfOf_py = 0.0f, ekfOf_pz = 0.0f;
float ekfOf_vx = 0.0f, ekfOf_vy = 0.0f, ekfOf_vz = 0.0f;
float ekfOf_bax = 0.0f, ekfOf_bay = 0.0f, ekfOf_baz = 0.0f;
float ekfOf_b_baro = 0.0f;
float ekfOf_scale = 1.0f;

// ==========================================================================
// CONSTANTS & NOISE PARAMETERS
// ==========================================================================
#define GRAVITY         9.80665f
#define STATE_DIM       11
#define EPS_SMALL       1e-9f

const float GYRO_SCALE_FACTOR = (1.0f/16.4f) * (3.14159265f / 180.0f);

// Optical Flow: PAW3903 @ 3200 CPI (Bright Mode)
// static const float FLOW_SCALE_RAD_PER_COUNT = 0.003307f; 
const float FLOW_SCALE_RAD_PER_COUNT = 0.006614f;

// Units
 const float TOF_SCALE_MM_TO_M  = 0.001f;
 const float BARO_SCALE_CM_TO_M = 0.01f;

// Process Noise
 const float q_pos_base   = 0.01f;     // Position uncertainty
 const float q_vel_base   = 10.0f;     // Velocity jerk
 const float q_ba_base    = 1.0e-6f;   // Bias random walk
 const float q_bbaro_base = 1.0e-6f;   
 const float q_ofscale    = 1.0e-8f;   // Scale drift

// Measurement Noise
const float R_tof_base   = 0.005f;     // High trust in Lidar
const float R_baro_base  = 35.0f;     
const float R_of_base    = 0.3f;     // Flow noise

// Gating Thresholds
const float OF_GATE_SIGMA   = 3.0f;   
const float TOF_GATE_SIGMA  = 10.0f;
const float BARO_GATE_SIGMA = 6.0f;

// Flow Quality
const float MIN_QUALITY_SCALE = 0.1f;
const float MAX_QUALITY_SCALE = 1.0f;

// ==========================================================================
// INTERNAL STATE
// ==========================================================================
bool initialized = false;
float x[STATE_DIM]; 
float P[STATE_DIM][STATE_DIM];

// ==========================================================================
// HELPER FUNCTIONS
// ==========================================================================

static void mat_mul_NxN(float C[STATE_DIM][STATE_DIM], float A[STATE_DIM][STATE_DIM], float B[STATE_DIM][STATE_DIM]) {
    float temp[STATE_DIM][STATE_DIM];
    memset(temp, 0, sizeof(temp));
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            for (int k = 0; k < STATE_DIM; k++) {
                temp[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    memcpy(C, temp, sizeof(temp));
}

// 1D Kalman Update (Standard Linear)
static void update_1D(float *H, float z, float R, float gate_sigma) {
    if (isnan(z)) return;

    float Hx = 0.0f;
    for (int i = 0; i < STATE_DIM; i++) Hx += H[i] * x[i];
    float y = z - Hx;

    float PHt[STATE_DIM];
    for (int i = 0; i < STATE_DIM; i++) {
        PHt[i] = 0.0f;
        for (int k = 0; k < STATE_DIM; k++) PHt[i] += P[i][k] * H[k];
    }

    float S = 0.0f;
    for (int k = 0; k < STATE_DIM; k++) S += H[k] * PHt[k];
    S += R;

    if (S <= EPS_SMALL) return;
    if (fabsf(y) > gate_sigma * sqrtf(S)) return;

    float K[STATE_DIM];
    float invS = 1.0f / S;
    for (int i = 0; i < STATE_DIM; i++) K[i] = PHt[i] * invS;

    for (int i = 0; i < STATE_DIM; i++) x[i] += K[i] * y;

    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            P[i][j] -= K[i] * PHt[j];
        }
    }
}

// 2D Kalman Update (MODIFIED: Accepts Residual 'y' directly to handle Non-Linearity)
static void update_2D(float H[2][STATE_DIM], float y[2], float R[2], float gate_sigma) {
    // NOTE: 'y' is passed in directly (Measured - Predicted)
    // We do NOT calculate y = z - Hx here because Hx is zero for optical flow functions.

    float PHt[STATE_DIM][2];
    for (int i = 0; i < STATE_DIM; i++) {
        for (int m = 0; m < 2; m++) {
            PHt[i][m] = 0.0f;
            for (int k = 0; k < STATE_DIM; k++) PHt[i][m] += P[i][k] * H[m][k];
        }
    }

    float S[2][2];
    for (int m = 0; m < 2; m++) {
        for (int n = 0; n < 2; n++) {
            float val = 0.0f;
            for (int k = 0; k < STATE_DIM; k++) val += H[m][k] * PHt[k][n];
            S[m][n] = val;
        }
        S[m][m] += R[m];
    }

    float det = S[0][0]*S[1][1] - S[0][1]*S[1][0];
    if (det < EPS_SMALL) return;
    
    float invDet = 1.0f / det;
    float S_inv[2][2];
    S_inv[0][0] =  S[1][1] * invDet;
    S_inv[0][1] = -S[0][1] * invDet;
    S_inv[1][0] = -S[1][0] * invDet;
    S_inv[1][1] =  S[0][0] * invDet;

    float temp_vec[2];
    temp_vec[0] = S_inv[0][0]*y[0] + S_inv[0][1]*y[1];
    temp_vec[1] = S_inv[1][0]*y[0] + S_inv[1][1]*y[1];
    float d2 = y[0]*temp_vec[0] + y[1]*temp_vec[1];
    
    if (sqrtf(d2) > gate_sigma) return;

    float K[STATE_DIM][2];
    for (int i = 0; i < STATE_DIM; i++) {
        K[i][0] = PHt[i][0]*S_inv[0][0] + PHt[i][1]*S_inv[1][0];
        K[i][1] = PHt[i][0]*S_inv[0][1] + PHt[i][1]*S_inv[1][1];
    }

    for (int i = 0; i < STATE_DIM; i++) {
        x[i] += K[i][0]*y[0] + K[i][1]*y[1];
    }

    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            float khp = K[i][0]*PHt[j][0] + K[i][1]*PHt[j][1];
            P[i][j] -= khp;
        }
    }
}

// ==========================================================================
// PUBLIC FUNCTIONS
// ==========================================================================

void ekfOf_Reset(void) {
    initialized = true;
    memset(x, 0, sizeof(x));
    x[10] = 1.0f; // of_scale init

    memset(P, 0, sizeof(P));
    P[0][0] = 1.0f; P[1][1] = 1.0f; P[2][2] = 0.25f; // pos
    P[3][3] = 0.5f; P[4][4] = 0.5f; P[5][5] = 0.5f;  // vel
    
    if (ENABLE_BIAS_ESTIMATION) {
        P[6][6] = 0.05f; P[7][7] = 0.05f; P[8][8] = 0.05f;
    } else {
        P[6][6] = 0.0f;  P[7][7] = 0.0f;  P[8][8] = 0.0f;
    }

    P[9][9] = 0.25f;   // baro bias
    P[10][10] = 0.01f; // scale
}

void ekfOf_Update(float ax, float ay, float az,
                  float wx, float wy, float wz,
                  float delta_x, float delta_y, float quality,
                  float pz_tof, float pz_baro,
                  float Qw, float Qx, float Qy, float Qz,
                  bool tof_valid,
                  bool of_valid,
                  float dt)
{
    if (!initialized) ekfOf_Reset();
    if (dt <= 0.0f) dt = 0.002f; 

    // --- 1. SENSOR SCALING ---
    ax *= ACCEL_SCALE_FACTOR;
    ay *= ACCEL_SCALE_FACTOR;
    az *= ACCEL_SCALE_FACTOR;

    wx *= GYRO_SCALE_FACTOR;
    wy *= GYRO_SCALE_FACTOR;
    wz *= GYRO_SCALE_FACTOR;

    if (!isnan(pz_tof)) pz_tof *= TOF_SCALE_MM_TO_M;
    if (!isnan(pz_baro)) pz_baro *= BARO_SCALE_CM_TO_M;

    // --- 2. PREDICTION STEP ---
    float px = x[0], py = x[1], pz = x[2];
    float vx = x[3], vy = x[4], vz = x[5];
    float bax = x[6], bay = x[7], baz = x[8];

    // Rotation Matrix Rwb (Body to World)
    float Rwb[3][3];
    float qw2 = Qw*Qw, qx2 = Qx*Qx, qy2 = Qy*Qy, qz2 = Qz*Qz;
    Rwb[0][0] = qw2 + qx2 - qy2 - qz2;   Rwb[0][1] = 2.0f*(Qx*Qy - Qw*Qz); Rwb[0][2] = 2.0f*(Qx*Qz + Qw*Qy);
    Rwb[1][0] = 2.0f*(Qx*Qy + Qw*Qz);   Rwb[1][1] = qw2 - qx2 + qy2 - qz2; Rwb[1][2] = 2.0f*(Qy*Qz - Qw*Qx);
    Rwb[2][0] = 2.0f*(Qx*Qz - Qw*Qy);   Rwb[2][1] = 2.0f*(Qy*Qz + Qw*Qx); Rwb[2][2] = qw2 - qx2 - qy2 + qz2;

    float a_body[3] = {ax, ay, az};
    float a_world[3] = {0, 0, 0};
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) a_world[i] += Rwb[i][j] * a_body[j];
    }
    
    // ENU Frame: Gravity is negative Z
    a_world[2] -= GRAVITY;

    // Effective Accel
    float ax_eff = a_world[0] - bax;
    float ay_eff = a_world[1] - bay;
    float az_eff = a_world[2] - baz;

    // Kinematic Prediction
    x[0] = px + vx*dt + 0.5f*ax_eff*dt*dt;
    x[1] = py + vy*dt + 0.5f*ay_eff*dt*dt;
    x[2] = pz + vz*dt + 0.5f*az_eff*dt*dt;

    x[3] = vx + ax_eff*dt;
    x[4] = vy + ay_eff*dt;
    x[5] = vz + az_eff*dt;

    // Jacobian F
    float F[STATE_DIM][STATE_DIM];
    memset(F, 0, sizeof(F));
    for(int i=0; i<STATE_DIM; i++) F[i][i] = 1.0f;

    F[0][3] = dt; F[0][6] = -0.5f*dt*dt;
    F[1][4] = dt; F[1][7] = -0.5f*dt*dt;
    F[2][5] = dt; F[2][8] = -0.5f*dt*dt;
    F[3][6] = -dt;
    F[4][7] = -dt;
    F[5][8] = -dt;

    // Process Noise
    float Qd[STATE_DIM];
    memset(Qd, 0, sizeof(Qd));
    Qd[0] = q_pos_base*dt; Qd[1] = q_pos_base*dt; Qd[2] = q_pos_base*dt;
    Qd[3] = q_vel_base*dt; Qd[4] = q_vel_base*dt; Qd[5] = q_vel_base*dt;
    
    if (ENABLE_BIAS_ESTIMATION) {
        Qd[6] = q_ba_base*dt; Qd[7] = q_ba_base*dt; Qd[8] = q_ba_base*dt;
    } else {
        Qd[6] = 0.0f; Qd[7] = 0.0f; Qd[8] = 0.0f;
    }

    Qd[9] = q_bbaro_base*dt;
    Qd[10] = q_ofscale*dt;

    // P = FPF' + Qd
    float FP[STATE_DIM][STATE_DIM];
    mat_mul_NxN(FP, F, P);

    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            float val = 0.0f;
            for (int k = 0; k < STATE_DIM; k++) {
                val += FP[i][k] * F[j][k];
            }
            if (i == j) val += Qd[i];
            P[i][j] = val;
        }
    }

    // Force Symmetry
    for(int i=0; i<STATE_DIM; i++) {
        for(int j=i+1; j<STATE_DIM; j++) {
            float avg = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = avg; P[j][i] = avg;
        }
    }

    // --- 3. MEASUREMENT UPDATES ---

    // A. TOF
    if (tof_valid) {
        float R_tof_eff = INFINITY;
        float pz_curr = x[2];
        if (pz_curr > 0.0f && pz_curr <= H_TOF_MAX) {
            R_tof_eff = R_tof_base;
        }

        if (!isinf(R_tof_eff)) {
            float H_tof[STATE_DIM];
            memset(H_tof, 0, sizeof(H_tof));
            H_tof[2] = 1.0f;
            update_1D(H_tof, pz_tof, R_tof_eff, TOF_GATE_SIGMA);
        }
    }

    // B. Baro
    if (!isnan(pz_baro)) {
        float H_baro[STATE_DIM];
        memset(H_baro, 0, sizeof(H_baro));
        H_baro[2] = 1.0f; H_baro[9] = 1.0f;
        update_1D(H_baro, pz_baro, R_baro_base, BARO_GATE_SIGMA);
    }

    // C. Optical Flow (NON-LINEAR UPDATE FIXED)
    if (of_valid && !isnan(delta_x) && !isnan(delta_y) && x[2] > MIN_PZ) {
        
        // --- 1. SENSOR MEASUREMENT PREP ---
        float raw_dx = (delta_x * FLOW_SCALE_RAD_PER_COUNT) / dt;
        float raw_dy = (delta_y * FLOW_SCALE_RAD_PER_COUNT) / dt;

        float flow_x, flow_y;

        if (SWAP_FLOW_XY) {
            flow_x = raw_dy;
            flow_y = raw_dx;
        } else {
            flow_x = raw_dx;
            flow_y = raw_dy;
        }

        if (INVERT_FLOW_X) flow_x = -flow_x;
        if (INVERT_FLOW_Y) flow_y = -flow_y;

        // Rotational Compensation (Measured Flow due to Rotation)
        float flow_rot[2] = { wy, wx }; 
        float z_meas[2] = { flow_x - flow_rot[0], flow_y - flow_rot[1] };

        // --- 2. NON-LINEAR PREDICTION (The Fix) ---
        // Get estimated World Velocity
        float v_world[3] = {x[3], x[4], x[5]};
        float v_body[3] = {0,0,0};
        
        // Transform World Velocity to Body Frame (Transpose of Rwb)
        for(int i=0; i<3; i++) {
            for(int j=0; j<3; j++) v_body[i] += Rwb[j][i] * v_world[j]; 
        }

        float pz_safe = x[2]; 
        float scale = x[10];

        // Explicitly calculate Expected Flow based on current state estimate
        // Expected Flow = -Scale * V_body / Height
        float flow_pred_x = (-scale * v_body[0]) / pz_safe;
        float flow_pred_y = (-scale * v_body[1]) / pz_safe;

        // Calculate Residual (Innovation) 'y' explicitly
        float y_residual[2];
        y_residual[0] = z_meas[0] - flow_pred_x;
        y_residual[1] = z_meas[1] - flow_pred_y;

        // --- 3. JACOBIAN CALCULATION ---
        float H_of[2][STATE_DIM];
        memset(H_of, 0, sizeof(H_of));

        // Partial wrt Velocity (World Frame via Rotation)
        for(int k=0; k<3; k++) {
            H_of[0][3+k] = (-scale / pz_safe) * Rwb[k][0]; 
            H_of[1][3+k] = (-scale / pz_safe) * Rwb[k][1]; 
        }

        // Partial wrt Position Z
        H_of[0][2] = scale * v_body[0] / (pz_safe * pz_safe);
        H_of[1][2] = scale * v_body[1] / (pz_safe * pz_safe);

        // Partial wrt Scale
        H_of[0][10] = -v_body[0] / pz_safe;
        H_of[1][10] = -v_body[1] / pz_safe;

        float qual = fminf(fmaxf(quality, 0.0f), 255.0f);
        float qual_factor = MIN_QUALITY_SCALE + (MAX_QUALITY_SCALE - MIN_QUALITY_SCALE) * (qual / 255.0f);
        float R_val = R_of_base / (qual_factor * qual_factor); 

        float R_of[2] = { R_val, R_val };
        
        // --- 4. CALL UPDATE (With Explicit Residual) ---
        update_2D(H_of, y_residual, R_of, OF_GATE_SIGMA);
    }

    // --- 4. EXPORT ---
    ekfOf_px = x[0]; ekfOf_py = x[1]; ekfOf_pz = x[2];
    ekfOf_vx = x[3]; ekfOf_vy = x[4]; ekfOf_vz = x[5];
    ekfOf_bax = x[6]; ekfOf_bay = x[7]; ekfOf_baz = x[8];
    ekfOf_b_baro = x[9];
    ekfOf_scale = x[10];
}