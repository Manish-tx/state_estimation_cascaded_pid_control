#include "sensor_monitor.h"

// --- Initialize the sensor struct (Like a constructor) ---
void sensor_init(SensorMonitor* s) {
    s->buffer_index = 0;
    s->current_R = BASE_R;
    s->is_failed = false;
    s->last_timestamp = 0;
    s->failure_count = 0;
    s->window_start_time = 0;
    
    // Clear buffer
    for(int i = 0; i < WINDOW_SIZE; i++) {
        s->data_buffer[i] = 0.0f;
    }
}

// --- Internal Helper: Handle Error Counting ---
void register_error(SensorMonitor* s, uint32_t now) {
    // Check if the old window has expired, if so, reset count
    if ((now - s->window_start_time) > FAILURE_WINDOW_MS) {
        s->failure_count = 0;
        s->window_start_time = now;
    }

    s->failure_count++;

    // "Multiple failures in small window -> SENSOR FAILED"
    if (s->failure_count > MAX_FAILURES_ALLOWED) {
        s->is_failed = true;
        s->current_R = R_MAX_FAILURE; // "Very very big value"
    }
}

// --- Internal Helper: Calculate Variance ---
float calculate_variance(SensorMonitor* s, float new_val) {
    // Add to circular buffer
    s->data_buffer[s->buffer_index] = new_val;
    s->buffer_index = (s->buffer_index + 1) % WINDOW_SIZE;

    // 1. Calculate Mean
    float mean = 0.0f;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        mean += s->data_buffer[i];
    }
    mean /= WINDOW_SIZE;

    // 2. Calculate Variance
    float variance = 0.0f;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        float diff = s->data_buffer[i] - mean;
        variance += (diff * diff);
    }
    return variance / WINDOW_SIZE;
}

// --- MAIN UPDATE FUNCTION ---
// Call this every time you read raw data
void sensor_update(SensorMonitor* s, float raw_value, uint32_t current_time_ms) {
    
    if (s->is_failed) return; // Do nothing if sensor is dead

    // CHECK 1: DT (Time difference) Check
    // "If 'dt' failures..."
    if ((current_time_ms - s->last_timestamp) > MAX_DT_MS) {
        register_error(s, current_time_ms);
    }
    s->last_timestamp = current_time_ms;

    // CHECK 2: Moving Variance Check
    // "Measure moving variance = r"
    float r = calculate_variance(s, raw_value);

    // "If r > constant, increment R"
    if (r > VAR_THRESHOLD) {
        s->current_R += R_INCREMENT;
        register_error(s, current_time_ms);
    } else {
        // Optional: Healing logic (slowly lower R back to normal)
        if (s->current_R > BASE_R) {
            s->current_R -= 0.1f;
        }
    }
}