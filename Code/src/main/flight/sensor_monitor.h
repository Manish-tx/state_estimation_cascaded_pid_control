#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// --- Tunable Parameters ---
#define WINDOW_SIZE 20          // Buffer size for variance calculation
#define VAR_THRESHOLD 5.0f      // "Some constant" (if r > this, sensor is noisy)
#define BASE_R 0.1f             // Default trust
#define R_INCREMENT 10.0f       // Penalty value
#define R_MAX_FAILURE 99999.0f  // "Very very big value"
#define MAX_DT_MS 20            // Max time between samples allowed
#define FAILURE_WINDOW_MS 1000  // Window to count errors
#define MAX_FAILURES_ALLOWED 5  // Max errors before permanent failure

// --- The Sensor Object ---
typedef struct {
    float data_buffer[WINDOW_SIZE];
    int buffer_index;
    
    float current_R;            // The "R" value (Noise Covariance)
    bool is_failed;             // Is the sensor dead?
    
    uint32_t last_timestamp;    // For checking "dt"
    int failure_count;          // Counter for errors
    uint32_t window_start_time; // Start of the error counting window
} SensorMonitor;

extern void sensor_init(SensorMonitor* s);
extern void sensor_update(SensorMonitor* s, float raw_value, uint32_t current_time_ms);