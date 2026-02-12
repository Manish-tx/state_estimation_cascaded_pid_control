#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include <stdint.h>

// --------------------------------------------------------------------------
// Global Tuning Constants (Declarations)
// --------------------------------------------------------------------------

// Low pass filter cutoff for D-term (Hz)
extern const float DTERM_LPF_HZ;

// Outer Loop (Angle) Gains: [Roll, Pitch, Yaw]
extern const float ANGLE_P[3];
extern const float ANGLE_I[3];
extern const float ANGLE_D[3];
extern const float ANGLE_I_LIMIT[3];

// Inner Loop (Rate) Gains: [Roll, Pitch, Yaw]
extern const float RATE_P[3];
extern const float RATE_I[3];
extern const float RATE_D[3];
extern const float RATE_I_LIMIT[3];

// --------------------------------------------------------------------------
// Global Outputs (Accessible from anywhere)
// --------------------------------------------------------------------------

// Final Motor Mixer commands (arbitrary PID units, typically +/- 1000 range)
// Unique name as requested
extern int16_t pidCascadedAltitude_out[3]; 

// Debugging: Intermediate Desired Rates (Output of Outer Loop)
extern float pidCascadedAltitude_debugDesiredRate[3]; 

// --------------------------------------------------------------------------
// Function Prototypes
// --------------------------------------------------------------------------

/**
 * Reset the PID controller state (Zero out integrators and previous errors).
 * Call this when arming or resetting the flight controller.
 */
void pidCascadedAttitude_Reset(void);

/**
 * Update the Cascaded PID Controller
 * @param currentAngles   Current attitude [Roll, Pitch, Yaw] in Degrees
 * @param gyroRates       Current gyro readings [Roll, Pitch, Yaw] in Deg/s
 * @param targetAngles    Desired attitude (from RC sticks) in Degrees
 * @param targetYawRate   Desired Yaw Rate (from RC sticks) in Deg/s (Yaw skips outer loop)
 * @param dt              Time step in seconds
 */
void pidCascadedAttitude_Update(float currentAngles[3], 
                                float gyroRates[3], 
                                float targetAngles[3], 
                                float targetYawRate,
                                float dt);

#endif // FLIGHT_CONTROL_H