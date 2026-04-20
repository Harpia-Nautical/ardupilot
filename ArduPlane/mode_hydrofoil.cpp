#include "mode.h"
#include "Plane.h"

// Initialize state on mode entry
bool ModeHydrofoil::_enter()
{
    // Validate rangefinder is present and configured correctly
    if (!validate_rangefinder()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Hydrofoil: No valid rangefinder");
        return false;
    }

    // Validate GPS is available for speed
    if (plane.gps.status() < AP_GPS_FixType::FIX_3D) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Hydrofoil: GPS required");
        return false;
    }

    // Initialize state machine
    current_state = State::IDLE;
    state_entry_time_ms = AP_HAL::millis();

    // Reset speed estimate
    speed_estimate_ms = 0.0f;
    last_gps_update_ms = 0;
    speed_integrated_since_gps = 0.0f;

    // Reset rangefinder filter
    rangefinder_buffer_idx = 0;
    filtered_altitude_cm = 0.0f;
    last_rangefinder_update_ms = 0;
    for (uint8_t i = 0; i < MEDIAN_FILTER_SIZE; i++) {
        rangefinder_buffer[i] = 0.0f;
    }

    // Reset PID states
    pitch_integrator = 0.0f;
    altitude_integrator = 0.0f;
    roll_integrator = 0.0f;
    last_pitch_error = 0.0f;
    last_altitude_error = 0.0f;
    last_roll_error = 0.0f;

    // Reset control outputs
    pitch_pid_out = 0.0f;
    altitude_pid_out = 0.0f;
    roll_pid_out = 0.0f;
    feedforward_front = 0.0f;
    feedforward_rear = 0.0f;

    // Reset RC modifiers
    altitude_offset_cm = 0.0f;
    roll_setpoint_deg = 0.0f;

    gcs().send_text(MAV_SEVERITY_INFO, "Hydrofoil mode active");
    return true;
}

// Cleanup on mode exit
void ModeHydrofoil::_exit()
{
    // Reset integrators to prevent windup affecting other modes
    pitch_integrator = 0.0f;
    altitude_integrator = 0.0f;
    roll_integrator = 0.0f;

    gcs().send_text(MAV_SEVERITY_INFO, "Hydrofoil mode exit");
}

// Pre-arm checks
bool ModeHydrofoil::_pre_arm_checks(size_t buflen, char *buffer) const
{
    // Check rangefinder
    if (!plane.rangefinder.has_data_orient(ROTATION_PITCH_270)) {
        hal.util->snprintf(buffer, buflen, "Hydrofoil: rangefinder not ready");
        return false;
    }

    // Check GPS
    if (plane.gps.status() < AP_GPS_FixType::FIX_3D) {
        hal.util->snprintf(buffer, buflen, "Hydrofoil: GPS fix required");
        return false;
    }

    // Check IMU calibration
    if (!plane.ahrs.healthy()) {
        hal.util->snprintf(buffer, buflen, "Hydrofoil: IMU not healthy");
        return false;
    }

    // Check feedforward constants are non-zero
    if (plane.g2.hydrofoil_K_front <= 0.0f || plane.g2.hydrofoil_K_rear <= 0.0f) {
        hal.util->snprintf(buffer, buflen, "Hydrofoil: feedforward K not configured");
        return false;
    }

    return true;
}

// Main update function called every loop
void ModeHydrofoil::update()
{
    // Update speed estimate from GPS + IMU fusion
    update_speed_estimate();

    // Update rangefinder filtering
    get_filtered_rangefinder_cm();

    // Run state machine
    update_state_machine();

    // Log data for analysis
    log_data();

    // Mix and output to servos
    mix_and_output_servos();
}

// Run function for controller resets
void ModeHydrofoil::run()
{
    // This mode doesn't use the standard TECS controller
    // Controllers are managed internally in update()
}

// ============================================================================
// STATE MACHINE
// ============================================================================

void ModeHydrofoil::update_state_machine()
{
    const uint32_t now_ms = AP_HAL::millis();
    const float throttle = plane.get_throttle_input(true);
    const float speed = speed_estimate_ms;
    const float altitude_cm = filtered_altitude_cm;

    State new_state = current_state;

    switch (current_state) {
        case State::IDLE:
            state_idle();
            // Transition to ACCELERATION_RUN when throttle applied
            if (throttle > plane.g.hydrofoil_throttle_min) {
                new_state = State::ACCELERATION_RUN;
                gcs().send_text(MAV_SEVERITY_INFO, "Hydrofoil: ACCELERATION_RUN");
            }
            break;

        case State::ACCELERATION_RUN:
            state_acceleration_run();
            // Transition to TRANSITION when speed threshold reached AND rangefinder shows liftoff
            if (speed >= plane.g.hydrofoil_min_foiling_speed &&
                altitude_cm > plane.g.hydrofoil_liftoff_detect_cm) {
                new_state = State::TRANSITION;
                gcs().send_text(MAV_SEVERITY_INFO, "Hydrofoil: TRANSITION");
            }
            // Fall back to IDLE if throttle cut
            if (throttle < plane.g.hydrofoil_throttle_min * 0.5f) {
                new_state = State::IDLE;
                gcs().send_text(MAV_SEVERITY_INFO, "Hydrofoil: IDLE");
            }
            break;

        case State::TRANSITION:
            state_transition();
            // Transition to FOILING when altitude stable near target
            if (altitude_cm > plane.g.hydrofoil_target_alt_cm * 0.8f &&
                altitude_cm < plane.g.hydrofoil_target_alt_cm * 1.2f &&
                now_ms - state_entry_time_ms > 2000) {  // At least 2 seconds in transition
                new_state = State::FOILING;
                gcs().send_text(MAV_SEVERITY_INFO, "Hydrofoil: FOILING");
            }
            // Fall back if speed too low
            if (speed < plane.g.hydrofoil_min_foiling_speed * 0.9f) {
                new_state = State::TOUCHDOWN;
                gcs().send_text(MAV_SEVERITY_INFO, "Hydrofoil: TOUCHDOWN (from transition)");
            }
            break;

        case State::FOILING:
            state_foiling();
            // Transition to TOUCHDOWN when speed drops
            if (speed < plane.g.hydrofoil_min_foiling_speed * 0.85f) {
                new_state = State::TOUCHDOWN;
                gcs().send_text(MAV_SEVERITY_INFO, "Hydrofoil: TOUCHDOWN");
            }
            // Emergency: rangefinder failure
            if (now_ms - last_rangefinder_update_ms > 1000) {
                new_state = State::TOUCHDOWN;
                gcs().send_text(MAV_SEVERITY_WARNING, "Hydrofoil: TOUCHDOWN (rangefinder fail)");
            }
            break;

        case State::TOUCHDOWN:
            state_touchdown();
            // Return to IDLE when hull on water and speed low
            if (altitude_cm < plane.g.hydrofoil_liftoff_detect_cm * 0.5f &&
                speed < plane.g.hydrofoil_min_foiling_speed * 0.3f) {
                new_state = State::IDLE;
                gcs().send_text(MAV_SEVERITY_INFO, "Hydrofoil: IDLE");
            }
            break;
    }

    // State change handling
    if (new_state != current_state) {
        current_state = new_state;
        state_entry_time_ms = now_ms;

        // Reset integrators on major state transitions
        if (new_state == State::IDLE || new_state == State::TOUCHDOWN) {
            altitude_integrator = 0.0f;
        }
    }
}

void ModeHydrofoil::state_idle()
{
    // Neutral control surfaces, no algorithms active
    feedforward_front = 0.0f;
    feedforward_rear = 0.0f;
    pitch_pid_out = 0.0f;
    altitude_pid_out = 0.0f;
    roll_pid_out = 0.0f;
}

void ModeHydrofoil::state_acceleration_run()
{
    // Feedforward with floor for front wings
    const float ff_base = get_feedforward_front(speed_estimate_ms);
    feedforward_front = MAX(plane.g.hydrofoil_front_floor, ff_base);

    // Rear wing: feedforward + pitch PID + proactive bias
    feedforward_rear = get_feedforward_rear(speed_estimate_ms);
    pitch_pid_out = pitch_controller();
    pitch_pid_out += plane.g.hydrofoil_rear_bias;

    // Roll PID holds 0° - critical to prevent strut dig-in
    roll_setpoint_deg = 0.0f;
    roll_pid_out = roll_controller();

    // Altitude loop inactive during acceleration run
    altitude_pid_out = 0.0f;
}

void ModeHydrofoil::state_transition()
{
    // Feedforward curves active
    feedforward_front = get_feedforward_front(speed_estimate_ms);
    feedforward_rear = get_feedforward_rear(speed_estimate_ms);

    // Proactive bias fading out linearly over first 2 seconds
    const uint32_t time_in_transition = AP_HAL::millis() - state_entry_time_ms;
    const float bias_fade = constrain_float(1.0f - (time_in_transition / 2000.0f), 0.0f, 1.0f);
    const float proactive_bias = plane.g.hydrofoil_rear_bias * bias_fade;

    // All PIDs active with aggressive gains
    pitch_pid_out = pitch_controller();
    altitude_pid_out = altitude_controller();
    roll_setpoint_deg = 0.0f;  // Still holding 0° in transition
    roll_pid_out = roll_controller();

    // Apply fading bias to rear
    feedforward_rear += proactive_bias;
}

void ModeHydrofoil::state_foiling()
{
    // Full control stack active
    feedforward_front = get_feedforward_front(speed_estimate_ms);
    feedforward_rear = get_feedforward_rear(speed_estimate_ms);

    // All PIDs active
    pitch_pid_out = pitch_controller();
    altitude_pid_out = altitude_controller();

    // Roll follows pilot input
    const float roll_input = plane.channel_roll->get_control_in();
    roll_setpoint_deg = (roll_input / 4500.0f) * plane.g.hydrofoil_max_bank;
    roll_pid_out = roll_controller();

    // Pitch stick modifies altitude target
    const float pitch_input = plane.channel_pitch->get_control_in();
    altitude_offset_cm = (pitch_input / 4500.0f) * plane.g.hydrofoil_alt_stick_range_cm;
}

void ModeHydrofoil::state_touchdown()
{
    // Gradual AoA reduction for controlled descent
    const uint32_t time_in_touchdown = AP_HAL::millis() - state_entry_time_ms;
    const float rampdown = constrain_float(1.0f - (time_in_touchdown / 3000.0f), 0.0f, 1.0f);

    feedforward_front = get_feedforward_front(speed_estimate_ms) * rampdown;
    feedforward_rear = get_feedforward_rear(speed_estimate_ms) * rampdown;

    // Pitch PID keeps hull level
    pitch_pid_out = pitch_controller() * rampdown;

    // Roll holds 0° through touchdown
    roll_setpoint_deg = 0.0f;
    roll_pid_out = roll_controller() * rampdown;

    // Altitude loop inactive - we want to descend
    altitude_pid_out = 0.0f;
}

// ============================================================================
// SPEED ESTIMATION
// ============================================================================

void ModeHydrofoil::update_speed_estimate()
{
    const uint32_t now_ms = AP_HAL::millis();

    // Get GPS speed
    const float gps_speed_ms = plane.gps.ground_speed();
    const uint32_t gps_time = plane.gps.last_fix_time_ms();

    // Check if we have a new GPS reading
    if (gps_time != last_gps_update_ms) {
        // Get IMU acceleration since last GPS update
        const Vector3f accel = plane.ahrs.get_accel();
        const float dt = (now_ms - last_gps_update_ms) * 0.001f;

        // Complementary filter: GPS truth + IMU integration
        const float alpha = 0.7f;  // Trust GPS more
        speed_estimate_ms = alpha * gps_speed_ms + (1.0f - alpha) * (speed_estimate_ms + accel.x * dt);

        // Reset integration
        last_gps_update_ms = gps_time;
        speed_integrated_since_gps = 0.0f;
    } else {
        // Between GPS updates: integrate IMU accel
        const Vector3f accel = plane.ahrs.get_accel();
        const float dt = 0.0025f;  // 400Hz loop rate
        speed_integrated_since_gps += accel.x * dt;
        speed_estimate_ms += accel.x * dt;
    }

    // Clamp to reasonable values
    speed_estimate_ms = constrain_float(speed_estimate_ms, 0.0f, 20.0f);
}

// ============================================================================
// FEEDFORWARD CURVES
// ============================================================================

float ModeHydrofoil::get_feedforward_front(float speed_ms)
{
    if (speed_ms < 0.1f) {
        return 0.0f;
    }

    // K / v² hyperbola - K is tuned to produce normalized output directly
    const float K = plane.g2.hydrofoil_K_front;
    return K / (speed_ms * speed_ms);
}

float ModeHydrofoil::get_feedforward_rear(float speed_ms)
{
    if (speed_ms < 0.1f) {
        return 0.0f;
    }

    // K / v² hyperbola - K is tuned to produce normalized output directly
    const float K = plane.g2.hydrofoil_K_rear;
    return K / (speed_ms * speed_ms);
}

// ============================================================================
// PID CONTROLLERS
// ============================================================================

float ModeHydrofoil::pitch_controller()
{
    // Target: 0° hull pitch at all times
    const float target_pitch_deg = 0.0f;
    const float current_pitch_deg = plane.ahrs.pitch_sensor * 0.01f;  // centidegrees to degrees
    const float pitch_error = target_pitch_deg - current_pitch_deg;

    // Get pitch rate directly from gyro (cleaner than derivative of error)
    const Vector3f gyro = plane.ahrs.get_gyro();
    const float pitch_rate_degps = degrees(gyro.y);

    // Apply gain scheduling
    const float gain_scale = get_gain_scale_factor();

    // PID calculation
    const float dt = 0.0025f;  // 400Hz
    const float P = plane.g2.hydrofoil_pitch_P * gain_scale;
    const float I = plane.g2.hydrofoil_pitch_I * gain_scale;
    const float D = plane.g2.hydrofoil_pitch_D * gain_scale;

    // P term
    float output = P * pitch_error;

    // I term with anti-windup
    pitch_integrator += I * pitch_error * dt;
    pitch_integrator = constrain_float(pitch_integrator, -5.0f, 5.0f);
    output += pitch_integrator;

    // D term on rate (not derivative of error)
    output -= D * pitch_rate_degps;

    return output;
}

float ModeHydrofoil::altitude_controller()
{
    // Only active in TRANSITION and FOILING
    if (current_state != State::TRANSITION && current_state != State::FOILING) {
        return 0.0f;
    }

    // Target altitude with RC offset
    const float target_alt_cm = plane.g.hydrofoil_target_alt_cm + altitude_offset_cm;
    const float current_alt_cm = filtered_altitude_cm;
    const float altitude_error = target_alt_cm - current_alt_cm;

    // Estimate vertical velocity from pitch and forward speed
    const float pitch_rad = radians(plane.ahrs.pitch_sensor * 0.01f);
    const float vertical_vel_mps = speed_estimate_ms * sinf(pitch_rad);

    // Apply gain scheduling
    const float gain_scale = get_gain_scale_factor();

    // PID calculation (outer loop runs at rangefinder rate, inner loop at IMU rate)
    const float dt = 0.0025f;
    const float P = plane.g2.hydrofoil_alt_P * gain_scale;
    const float I = plane.g2.hydrofoil_alt_I * gain_scale;
    const float D = plane.g2.hydrofoil_alt_D * gain_scale;

    // P term
    float output = P * altitude_error * 0.01f;  // cm to meters

    // I term with anti-windup
    altitude_integrator += I * altitude_error * 0.01f * dt;
    altitude_integrator = constrain_float(altitude_integrator, -3.0f, 3.0f);
    output += altitude_integrator;

    // D term on vertical velocity (damping)
    output -= D * vertical_vel_mps;

    return output;
}

float ModeHydrofoil::roll_controller()
{
    // Target roll angle
    const float target_roll_deg = roll_setpoint_deg;
    const float current_roll_deg = plane.ahrs.roll_sensor * 0.01f;  // centidegrees to degrees
    const float roll_error = target_roll_deg - current_roll_deg;

    // Get roll rate directly from gyro
    const Vector3f gyro = plane.ahrs.get_gyro();
    const float roll_rate_degps = degrees(gyro.x);

    // Apply gain scheduling
    const float gain_scale = get_gain_scale_factor();

    // PID calculation
    const float dt = 0.0025f;
    const float P = plane.g2.hydrofoil_roll_P * gain_scale;
    const float I = plane.g2.hydrofoil_roll_I * gain_scale;
    const float D = plane.g2.hydrofoil_roll_D * gain_scale;

    // P term
    float output = P * roll_error;

    // I term with anti-windup
    roll_integrator += I * roll_error * dt;
    roll_integrator = constrain_float(roll_integrator, -3.0f, 3.0f);
    output += roll_integrator;

    // D term on rate
    output -= D * roll_rate_degps;

    return output;
}

// ============================================================================
// GAIN SCHEDULING
// ============================================================================

float ModeHydrofoil::get_gain_scale_factor()
{
    // Scale gains by (v_ref / v_current)² to compensate for v² plant dynamics
    const float v_ref = plane.g.hydrofoil_gain_sched_ref_speed;
    const float v_current = speed_estimate_ms;

    if (v_current < 1.0f || v_ref < 1.0f) {
        return 1.0f;  // Avoid division by zero
    }

    const float ratio = v_ref / v_current;
    const float scale = ratio * ratio;

    // Limit scaling to prevent extreme gains
    return constrain_float(scale, 0.25f, 4.0f);
}

// ============================================================================
// SERVO MIXING AND OUTPUT
// ============================================================================

void ModeHydrofoil::mix_and_output_servos()
{
    // Front wings: feedforward + altitude (collective) + roll (differential)
    const float front_collective = feedforward_front + altitude_pid_out;
    const float front_left_norm = front_collective + roll_pid_out;
    const float front_right_norm = front_collective - roll_pid_out;

    // Rear wing: feedforward + pitch
    const float rear_norm = feedforward_rear + pitch_pid_out;

    // Convert normalized values (-1.0 to +1.0) to servo output (-4500 to +4500)
    const int16_t left_servo = constrain_int16(front_left_norm * 4500.0f, -4500, 4500);
    const int16_t right_servo = constrain_int16(front_right_norm * 4500.0f, -4500, 4500);
    const int16_t rear_servo = constrain_int16(rear_norm * 4500.0f, -4500, 4500);

    // Output to servos
    // Front left = aileron left, front right = aileron right, rear = elevator
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, left_servo);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, rear_servo);

    // Set right aileron separately if available
    SRV_Channels::set_output_scaled(SRV_Channel::k_dspoilerRight1, right_servo);

    // Throttle passthrough
    const float throttle = plane.get_throttle_input(true);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle * 100.0f);
}

// ============================================================================
// UTILITY METHODS
// ============================================================================

float ModeHydrofoil::get_filtered_rangefinder_cm()
{
    const uint32_t now_ms = AP_HAL::millis();

    // Check if rangefinder has new data
    if (plane.rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::Status::Good) {
        const float reading_cm = plane.rangefinder.distance_orient(ROTATION_PITCH_270) * 100.0f;

        // Apply median filter
        filtered_altitude_cm = median_filter(reading_cm);
        last_rangefinder_update_ms = now_ms;
    }

    return filtered_altitude_cm;
}

float ModeHydrofoil::median_filter(float new_value)
{
    // Add new value to circular buffer
    rangefinder_buffer[rangefinder_buffer_idx] = new_value;
    rangefinder_buffer_idx = (rangefinder_buffer_idx + 1) % MEDIAN_FILTER_SIZE;

    // Sort buffer to find median
    float sorted[MEDIAN_FILTER_SIZE];
    memcpy(sorted, rangefinder_buffer, sizeof(rangefinder_buffer));

    // Simple bubble sort
    for (uint8_t i = 0; i < MEDIAN_FILTER_SIZE - 1; i++) {
        for (uint8_t j = 0; j < MEDIAN_FILTER_SIZE - i - 1; j++) {
            if (sorted[j] > sorted[j + 1]) {
                const float temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }

    // Return median
    return sorted[MEDIAN_FILTER_SIZE / 2];
}

bool ModeHydrofoil::validate_rangefinder()
{
    // Check for rangefinder with correct orientation
    if (!plane.rangefinder.has_data_orient(ROTATION_PITCH_270)) {
        return false;
    }

    // Check sensor type (MaxBotixSerial preferred)
    const RangeFinder::Type type = plane.rangefinder.get_type(ROTATION_PITCH_270);
    if (type == RangeFinder::Type::NONE) {
        return false;
    }

    return true;
}

float ModeHydrofoil::get_vertical_velocity_ms()
{
    // Estimate vertical velocity from pitch and forward speed
    const float pitch_rad = radians(plane.ahrs.pitch_sensor * 0.01f);
    return speed_estimate_ms * sinf(pitch_rad);
}

// ============================================================================
// LOGGING AND TELEMETRY
// ============================================================================

void ModeHydrofoil::log_data()
{
    // Log at 10Hz to avoid overwhelming the logger
    const uint32_t now_ms = AP_HAL::millis();
    static uint32_t last_log_ms = 0;
    if (now_ms - last_log_ms < 100) {
        return;
    }
    last_log_ms = now_ms;

    // Prepare log data structure
    struct PACKED {
        uint64_t time_us;
        uint8_t state;
        float speed_ms;
        float altitude_cm;
        float feedforward_front;
        float feedforward_rear;
        float pitch_pid;
        float altitude_pid;
        float roll_pid;
        float pitch_error;
        float altitude_error;
        float roll_error;
        float gain_scale;
    } pkt;

    pkt.time_us = AP_HAL::micros64();
    pkt.state = static_cast<uint8_t>(current_state);
    pkt.speed_ms = speed_estimate_ms;
    pkt.altitude_cm = filtered_altitude_cm;
    pkt.feedforward_front = feedforward_front;
    pkt.feedforward_rear = feedforward_rear;
    pkt.pitch_pid = pitch_pid_out;
    pkt.altitude_pid = altitude_pid_out;
    pkt.roll_pid = roll_pid_out;
    pkt.pitch_error = -plane.ahrs.pitch_sensor * 0.01f;  // Target is 0
    pkt.altitude_error = (plane.g.hydrofoil_target_alt_cm + altitude_offset_cm) - filtered_altitude_cm;
    pkt.roll_error = roll_setpoint_deg - (plane.ahrs.roll_sensor * 0.01f);
    pkt.gain_scale = get_gain_scale_factor();

    // Write to dataflash
    plane.logger.WriteBlock(&pkt, sizeof(pkt));

    // Also write to standard attitude/rate logs for cross-reference
    // These are automatically logged by ArduPilot core
}
