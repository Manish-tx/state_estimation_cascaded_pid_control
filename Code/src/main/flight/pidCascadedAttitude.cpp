#include <math.h>
#include <string.h> // for memset
#include "flight/pidCascadedAttitude.h"

// --------------------------------------------------------------------------
// Global Output Definitions
// --------------------------------------------------------------------------
int16_t pidCascadedAltitude_out[3] = {0, 0, 0};
float pidCascadedAltitude_debugDesiredRate[3] = {0.0f, 0.0f, 0.0f};

// --------------------------------------------------------------------------
// Internal State Variables (Static Globals)
// These persist between function calls but are not visible to other files
// --------------------------------------------------------------------------
static float prevAngleError[3] = {0.0f, 0.0f, 0.0f};
static float angleITerm[3]     = {0.0f, 0.0f, 0.0f};
static float prevGyroRate[3]   = {0.0f, 0.0f, 0.0f};
static float rateITerm[3]      = {0.0f, 0.0f, 0.0f};
static float dTermFilterState[3] = {0.0f, 0.0f, 0.0f};

// --------------------------------------------------------------------------
// Tuning Configuration (Definitions)
// --------------------------------------------------------------------------

// Constants
static const float MAX_CONTROL_RATE = 300.0f; // Limit desired rotation to 300 deg/s
static const float MAX_PID_OUTPUT   = 1000.0f; // Mixer limit

// Filter settings
const float DTERM_LPF_HZ = 100.0f;

// Outer Loop (Angle) Gains - P converts Error(deg) to Rate(deg/s)
// Format: { Roll, Pitch, Yaw }
// const float ANGLE_P[3]       = { 15.5f,  12.5f,  0.0f }; 
// const float ANGLE_I[3]       = { 0.0f,  0.0f,  0.0f }; 
// const float ANGLE_D[3]       = { 0.0f,  0.0f,  0.0f }; 
const float ANGLE_P[3]       = { 10.5f,  8.5f,  1.0f }; 
const float ANGLE_I[3]       = { 0.0f,  0.0f,  0.0f }; 
const float ANGLE_D[3]       = { 2.0f,  1.0f,  0.0f }; 
const float ANGLE_I_LIMIT[3] = { 0.0f,  0.0f,  0.0f }; 

// Inner Loop (Rate) Gains - P converts Error(deg/s) to MotorCommand
// Format: { Roll, Pitch, Yaw }
// const float RATE_P[3]       = { 1.45f, 2.45f, 0.60f }; 
// const float RATE_I[3]       = { 0.35f, 0.35f, 0.45f }; 
// const float RATE_D[3]       = { 0.02f, 0.02f, 0.00f }; 
const float RATE_P[3]       = { 0.45f, 0.45f, 0.0f }; 
const float RATE_I[3]       = { 0.0f, 0.0f, 0.0f }; 
const float RATE_D[3]       = { 0.0f, 0.0f, 0.00f }; 
const float RATE_I_LIMIT[3] = { 200.f, 200.f, 200.f }; 

// --------------------------------------------------------------------------
// Helper Functions (Internal)
// --------------------------------------------------------------------------

static float constrainf(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

// Simple PT1 (1st order) Low Pass Filter
static float applyPT1Filter(float input, float* state, float cutoff_hz, float dt) {
    if (cutoff_hz <= 0.0f) return input; // Filter disabled
    
    float rc = 1.0f / (2.0f * 3.14159265f * cutoff_hz);
    float alpha = dt / (rc + dt);
    
    *state = *state + alpha * (input - *state);
    return *state;
}

// --------------------------------------------------------------------------
// Public Function Implementations
// --------------------------------------------------------------------------

void pidCascadedAttitude_Reset(void) {
    // Explicitly zero out all internal states
    for(int i=0; i<3; i++) {
        prevAngleError[i] = 0.0f;
        angleITerm[i]     = 0.0f;
        prevGyroRate[i]   = 0.0f;
        rateITerm[i]      = 0.0f;
        dTermFilterState[i] = 0.0f;
    }
}

void pidCascadedAttitude_Update(float currentAngles[3], 
                                float gyroRates[3], 
                                float targetAngles[3], 
                                float targetYawRate,
                                float dt) 
{
    // Prevent divide by zero if dt is malformed
    if (dt <= 0.000001f) dt = 0.001f;

    // Loop through Roll (0), Pitch (1), Yaw (2)
    for (int axis = 0; axis < 3; axis++) {
        
        float desiredRate = 0.0f;

        // ----------------------------------------
        // OUTER LOOP: ATTITUDE (Angle -> Rate)
        // ----------------------------------------
        // We only run Outer Loop for Roll(0) and Pitch(1). 
        // Yaw(2) is typically Rate-controlled by stick directly.
        
        if (axis != 2) { 
            float errorAngle = targetAngles[axis] - currentAngles[axis];

            // P Term
            float P_out = ANGLE_P[axis] * errorAngle;

            // I Term
            angleITerm[axis] += ANGLE_I[axis] * errorAngle * dt;
            angleITerm[axis] = constrainf(angleITerm[axis], -ANGLE_I_LIMIT[axis], ANGLE_I_LIMIT[axis]);

            // D Term (Derivative of Angle)
            float deltaError = (errorAngle - prevAngleError[axis]) / dt;
            float D_out = ANGLE_D[axis] * deltaError;
            
            prevAngleError[axis] = errorAngle;

            desiredRate = P_out + angleITerm[axis] + D_out;
            
            // Clamp desired rate
            desiredRate = constrainf(desiredRate, -MAX_CONTROL_RATE, MAX_CONTROL_RATE);
        } else {
            // For Yaw, the "Outer Loop" output is just the stick input
            desiredRate = targetYawRate;
        }

        // Store to global variable for debugging
        pidCascadedAltitude_debugDesiredRate[axis] = desiredRate;

        // ----------------------------------------
        // INNER LOOP: RATE (Rate -> Motor Command)
        // ----------------------------------------
        
        float errorRate = desiredRate - gyroRates[axis];

        // P Term
        float P_inner = RATE_P[axis] * errorRate;

        // I Term
        rateITerm[axis] += RATE_I[axis] * errorRate * dt;
        rateITerm[axis] = constrainf(rateITerm[axis], -RATE_I_LIMIT[axis], RATE_I_LIMIT[axis]);

        // D Term (Derivative of Gyro Rate)
        // Measurement Derivative (Derivative on PV)
        float deltaGyro = (gyroRates[axis] - prevGyroRate[axis]) / dt;
        prevGyroRate[axis] = gyroRates[axis];

        // Apply Low Pass Filter to D-term input
        float dTermFiltered = applyPT1Filter(deltaGyro, &dTermFilterState[axis], DTERM_LPF_HZ, dt);

        float D_inner = -(RATE_D[axis] * dTermFiltered);

        // Summation
        float output = P_inner + rateITerm[axis] + D_inner;

        // Clamp to Mixer range
        output = constrainf(output, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);

        // Store to global output variable
        pidCascadedAltitude_out[axis] = (int16_t)output;
    }
}