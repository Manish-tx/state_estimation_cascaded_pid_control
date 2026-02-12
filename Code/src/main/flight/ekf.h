#ifndef FLIGHT_EKF_OPTICAL_FLOW_H
#define FLIGHT_EKF_OPTICAL_FLOW_H

#include <stdint.h>
#include <stdbool.h>

// --------------------------------------------------------------------------
// Global Output Definitions (World Frame)
// --------------------------------------------------------------------------
extern float ekfOf_px, ekfOf_py, ekfOf_pz;       // Position (m)
extern float ekfOf_vx, ekfOf_vy, ekfOf_vz;       // Velocity (m/s)
extern float ekfOf_bax, ekfOf_bay, ekfOf_baz;    // Accel Bias (m/s^2)
extern float ekfOf_b_baro;                       // Barometer Bias (m)
extern float ekfOf_scale;                        // Optical Flow Scale factor

// --------------------------------------------------------------------------
// Function Prototypes
// --------------------------------------------------------------------------

/**
 * Reset the EKF state and covariance.
 * Call this before arming or when resetting the system.
 */
void ekfOf_Reset(void);

/**
 * Update the Extended Kalman Filter
 *
 * @param ax, ay, az      Raw accelerometer (m/s^2)
 * @param wx, wy, wz      Raw gyroscope (rad/s)
 * @param delta_x, y      Optical flow angular rates (rad/s) - NAN if invalid
 * @param quality         Optical flow quality (0..255)
 * @param pz_tof          TOF height (m) - NAN if invalid
 * @param pz_baro         Baro height (m) - NAN if invalid
 * @param Qw, Qx, Qy, Qz  Attitude Quaternion (Body -> World)
 * @param dt              Time step (s)
 */
void ekfOf_Update(float ax, float ay, float az,
                  float wx, float wy, float wz,
                  float delta_x, float delta_y, float quality,
                  float pz_tof, float pz_baro,
                  float Qw, float Qx, float Qy, float Qz,
                  bool tof_valid,
                  bool of_valid,
                  float dt);

#endif