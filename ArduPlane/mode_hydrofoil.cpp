#include "mode.h"
#include "Plane.h"

// Constructor - initialize all member variables to safe defaults
ModeHydrofoil::ModeHydrofoil() :
    Mode(),
    current_state(State::IDLE),
    state_entry_time_ms(0),
    speed_estimate_ms(0.0f),
    last_gps_update_ms(0),
    speed_integrated_since_gps(0.0f),
    rangefinder_buffer{0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    rangefinder_buffer_idx(0),
    filtered_altitude_cm(0.0f),
    last_rangefinder_update_ms(0),
    pitch_integrator(0.0f),
    altitude_integrator(0.0f),
    roll_integrator(0.0f),
    speed_integrator(0.0f),
    last_pitch_error(0.0f),
    last_altitude_error(0.0f),
    last_roll_error(0.0f),
    last_speed_error(0.0f),
    feedforward_front(0.0f),
    feedforward_rear(0.0f),
    pitch_pid_out(0.0f),
    altitude_pid_out(0.0f),
    roll_pid_out(0.0f),
    speed_pid_out(0.0f),
    altitude_offset_cm(0.0f),
    roll_setpoint_deg(0.0f),
    speed_setpoint_ms(0.0f)
{
}

// Initialize state on mode entry
bool ModeHydrofoil::_enter()
{
    // Validate rangefinder is present and configured correctly
    if (!validate_rangefinder()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Hydrofoil: No valid rangefinder");
        return false;
    }

    // GPS not required - speed estimation will use GPS when available, IMU otherwise
    // if (plane.gps.status() < AP_GPS_FixType::FIX_3D) {
    //     gcs().send_text(MAV_SEVERITY_WARNING, "Hydrofoil: GPS required");
    //     return false;
    // }

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
    speed_integrator = 0.0f;
    last_pitch_error = 0.0f;
    last_altitude_error = 0.0f;
    last_roll_error = 0.0f;
    last_speed_error = 0.0f;

    // Reset control outputs
    pitch_pid_out = 0.0f;
    altitude_pid_out = 0.0f;
    roll_pid_out = 0.0f;
    speed_pid_out = 0.0f;
    feedforward_front = 0.0f;
    feedforward_rear = 0.0f;

    // Reset RC modifiers
    altitude_offset_cm = 0.0f;
    roll_setpoint_deg = 0.0f;
    speed_setpoint_ms = plane.g.hydrofoil_target_speed;

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
    speed_integrator = 0.0f;

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

    // GPS not required - speed estimation will use GPS when available
    // if (plane.gps.status() < AP_GPS_FixType::FIX_3D) {
    //     hal.util->snprintf(buffer, buflen, "Hydrofoil: GPS fix required");
    //     return false;
    // }

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

    // Debug telemetry: filtered by HFOL_DEBUG_MASK bitmask
    const uint32_t now_ms = AP_HAL::millis();
    static uint32_t last_debug_log_ms = 0;
    if (now_ms - last_debug_log_ms > 1000) {
        last_debug_log_ms = now_ms;
        const uint8_t debug_mask = plane.g.hydrofoil_debug_mask.get();

        // Bit 4: State/General info
        if (debug_mask & 16) {
            gcs().send_text(MAV_SEVERITY_INFO, "HFOL: Alt=%.1fcm Spd=%.1fm/s St=%d CtrlEn=%d",
                            filtered_altitude_cm, speed_estimate_ms, (int)current_state,
                            (int)plane.g.hydrofoil_ctrl_enable.get());
        }

        // Bit 0: Pitch debug (shows setpoint, current, error, output)
        if (debug_mask & 1) {
            const float pitch_setpoint = 0.0f + altitude_pid_out;  // 0° + altitude correction
            const float pitch_current = plane.ahrs.pitch_sensor * 0.01f;
            const float pitch_error = pitch_setpoint - pitch_current;
            gcs().send_text(MAV_SEVERITY_INFO, "PITCH: SP=%.2f Cur=%.2f Err=%.2f Out=%.3f P/I/D=%.3f/%.3f/%.3f",
                            (double)pitch_setpoint, (double)pitch_current, (double)pitch_error,
                            (double)pitch_pid_out,
                            (double)plane.g2.hydrofoil_pitch_P.get(),
                            (double)plane.g2.hydrofoil_pitch_I.get(),
                            (double)plane.g2.hydrofoil_pitch_D.get());
        }

        // Bit 1: Altitude debug
        if (debug_mask & 2) {
            gcs().send_text(MAV_SEVERITY_INFO, "ALT: P=%.3f I=%.3f D=%.3f Out=%.3f",
                            (double)plane.g2.hydrofoil_alt_P.get(),
                            (double)plane.g2.hydrofoil_alt_I.get(),
                            (double)plane.g2.hydrofoil_alt_D.get(),
                            (double)altitude_pid_out);
        }

        // Bit 2: Roll debug
        if (debug_mask & 4) {
            gcs().send_text(MAV_SEVERITY_INFO, "ROLL: P=%.3f I=%.3f D=%.3f Out=%.3f",
                            (double)plane.g2.hydrofoil_roll_P.get(),
                            (double)plane.g2.hydrofoil_roll_I.get(),
                            (double)plane.g2.hydrofoil_roll_D.get(),
                            (double)roll_pid_out);
        }

        // Bit 3: Speed debug
        if (debug_mask & 8) {
            gcs().send_text(MAV_SEVERITY_INFO, "SPEED: P=%.3f I=%.3f D=%.3f Out=%.3f",
                            (double)plane.g2.hydrofoil_speed_P.get(),
                            (double)plane.g2.hydrofoil_speed_I.get(),
                            (double)plane.g2.hydrofoil_speed_D.get(),
                            (double)speed_pid_out);
        }
    }

    // Run state machine
    update_state_machine();

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

    if (throttle < plane.g.hydrofoil_throttle_min * 0.5f) {
        new_state = State::IDLE;
        gcs().send_text(MAV_SEVERITY_INFO, "Hydrofoil: IDLE");
    }

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
            // OR if rangefinder has been failed for extended time and speed is low
            {
                const bool rangefinder_dead = (now_ms - last_rangefinder_update_ms > 3000);
                const bool altitude_condition = altitude_cm < plane.g.hydrofoil_liftoff_detect_cm * 0.5f || rangefinder_dead;
                const bool speed_condition = speed < plane.g.hydrofoil_min_foiling_speed * 0.3f;

                if (altitude_condition && speed_condition) {
                    new_state = State::IDLE;
                    if (rangefinder_dead) {
                        gcs().send_text(MAV_SEVERITY_INFO, "Hydrofoil: IDLE (rangefinder failed)");
                    } else {
                        gcs().send_text(MAV_SEVERITY_INFO, "Hydrofoil: IDLE");
                    }
                }
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

    // Throttle passthrough
    speed_pid_out = 0;
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
    altitude_pid_out = altitude_controller();

    // Throttle control (manual passthrough or PID)
    speed_pid_out = speed_controller();
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
    const float roll_input = plane.channel_roll->norm_input();
    roll_setpoint_deg = (roll_input) * plane.g.hydrofoil_max_bank / 2;
    roll_pid_out = roll_controller();

    // Apply fading bias to rear
    feedforward_rear += proactive_bias;

    // Throttle control (manual passthrough or PID)
    speed_pid_out = speed_controller();
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
    const float roll_input = plane.channel_roll->norm_input();
    roll_setpoint_deg = (roll_input) * plane.g.hydrofoil_max_bank;
    roll_pid_out = roll_controller();

    // Pitch stick modifies altitude target
    const float pitch_input = plane.channel_pitch->norm_input();
    altitude_offset_cm = (pitch_input) * plane.g.hydrofoil_alt_stick_range_cm;

    // Speed controller for throttle
    speed_setpoint_ms = plane.g.hydrofoil_target_speed;  // Could add throttle stick offset here
    speed_pid_out = speed_controller();
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

    // Throttle control (manual passthrough or PID)
    speed_pid_out = speed_controller();
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
        const float alpha = 0.9f;  // Trust GPS more
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
    return constrain_float(K / (speed_ms * speed_ms), -0.5, 0.5);
}

float ModeHydrofoil::get_feedforward_rear(float speed_ms)
{
    if (speed_ms < 0.1f) {
        return 0.0f;
    }

    // K / v² hyperbola - K is tuned to produce normalized output directly
    const float K = plane.g2.hydrofoil_K_rear;
    return constrain_float(K / (speed_ms * speed_ms), -0.7, 0.7);
}

// ============================================================================
// PID CONTROLLERS
// ============================================================================

float ModeHydrofoil::pitch_controller()
{
    // CASCADE ARCHITECTURE: Pitch controller tracks commanded pitch angle
    // Commanded pitch = 0° (level) + altitude_pid_out (±3° for altitude control)
    // Controls rear wing to achieve desired hull pitch

    // Check if pitch controller is enabled (bit 0)
    if (!(plane.g.hydrofoil_ctrl_enable.get() & 1)) {
        return plane.pitch_in_expo(false) / 4500.0f;
    }

    // Target pitch: 0° baseline + altitude correction from outer loop
    // altitude_pid_out contains pitch command (±3°) from altitude controller
    const float target_pitch_deg = 0.0f + altitude_pid_out;
    const float current_pitch_deg = plane.ahrs.pitch_sensor * 0.01f;  // centidegrees to degrees
    const float pitch_error = target_pitch_deg - current_pitch_deg;

    // Get pitch rate directly from gyro (cleaner than derivative of error)
    const Vector3f gyro = plane.ahrs.get_gyro();
    const float pitch_rate_degps = degrees(gyro.y);

    // Apply gain scheduling
    const float gain_scale = get_gain_scale_factor();

    // PID calculation
    // Input: pitch error in degrees
    // Output: normalized wing deflection (-1 to +1)
    const float dt = 0.0025f;  // 400Hz
    const float P = plane.g2.hydrofoil_pitch_P * gain_scale;
    const float I = plane.g2.hydrofoil_pitch_I * gain_scale;
    const float D = plane.g2.hydrofoil_pitch_D * gain_scale;

    // P term (degrees → normalized)
    float output = P * pitch_error;

    // I term with anti-windup (normalized units)
    pitch_integrator += I * pitch_error * dt;
    pitch_integrator = constrain_float(pitch_integrator, -0.5f, 0.5f);
    output += pitch_integrator;

    // D term on rate (degrees/s → normalized)
    output -= D * pitch_rate_degps;

    // Constrain output to normalized range
    return constrain_float(output, -1.0f, 1.0f);
}

float ModeHydrofoil::altitude_controller()
{
    // CASCADE ARCHITECTURE: Altitude controller outputs PITCH COMMAND (degrees)
    // At constant speed, pitch angle controls altitude:
    // - Pitch up → more AoA on all surfaces → more lift → altitude increases
    // - Pitch down → less AoA → less lift → altitude decreases
    // Front wings stay at feedforward AoA (decoupled, reserved for roll)

    // Check if altitude controller is enabled (bit 1)
    if (!(plane.g.hydrofoil_ctrl_enable.get() & 2)) {
        // Disabled: return 0° pitch correction (maintain level flight)
        return 0.0f;
    }

    // Target altitude with RC offset
    const float target_alt_cm = plane.g.hydrofoil_target_alt_cm + altitude_offset_cm;
    const float current_alt_cm = filtered_altitude_cm;
    const float altitude_error = target_alt_cm - current_alt_cm;

    // Estimate vertical velocity from pitch and forward speed
    const float vertical_vel_mps = get_vertical_velocity_ms();

    // Apply gain scheduling
    const float gain_scale = get_gain_scale_factor();

    // PID calculation - outputs desired pitch angle correction (degrees)
    const float dt = 0.0025f;
    const float P = plane.g2.hydrofoil_alt_P * gain_scale;
    const float I = plane.g2.hydrofoil_alt_I * gain_scale;
    const float D = plane.g2.hydrofoil_alt_D * gain_scale;

    // P term - altitude error (cm) to pitch command (degrees)
    float pitch_cmd = P * altitude_error * 0.01f;  // cm to meters

    // I term with anti-windup
    altitude_integrator += I * altitude_error * 0.01f * dt;
    altitude_integrator = constrain_float(altitude_integrator, -3.0f, 3.0f);
    pitch_cmd += altitude_integrator;

    // D term on vertical velocity (damping)
    pitch_cmd -= D * vertical_vel_mps;

    // Limit pitch command to ±3° for safety
    // This prevents aggressive pitch changes that could destabilize
    return constrain_float(pitch_cmd, -3.0f, 3.0f);
}

float ModeHydrofoil::roll_controller()
{
    // Check if roll controller is enabled (bit 2)
    if (!(plane.g.hydrofoil_ctrl_enable.get() & 4)) {
        // Disabled: return manual roll stick input (normalized -1 to +1)
        // Roll stick controls front wing differential when roll PID is off
        return plane.roll_in_expo(false) / 4500.0f;
    }

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

float ModeHydrofoil::speed_controller()
{
    // Check if speed controller is enabled (bit 3)
    if (!(plane.g.hydrofoil_ctrl_enable.get() & 8)) {
        // Disabled: return manual throttle input (percentage 0-100)
        return plane.get_throttle_input(true) / 100.0f;
    }

    // Target speed with potential RC offset
    const float target_speed_ms = speed_setpoint_ms;
    const float current_speed_ms = speed_estimate_ms;
    const float speed_error = (target_speed_ms - current_speed_ms) / 100;

    // Get forward acceleration from IMU
    const Vector3f accel = plane.ahrs.get_accel();
    const float forward_accel_mps2 = accel.x;

    // Apply gain scheduling
    const float gain_scale = get_gain_scale_factor();

    // PID calculation
    const float dt = 0.0025f;  // 400Hz
    const float P = plane.g2.hydrofoil_speed_P * gain_scale;
    const float I = plane.g2.hydrofoil_speed_I * gain_scale;
    const float D = plane.g2.hydrofoil_speed_D * gain_scale;
    const float FF = plane.g2.hydrofoil_speed_FF;

    // Feedforward: baseline throttle proportional to target speed
    float output = FF * target_speed_ms;

    // P term
    output += P * speed_error;

    // I term with anti-windup
    speed_integrator += I * speed_error * dt;
    speed_integrator = constrain_float(speed_integrator, -0.5f, 0.5f);  // Limit to ±50% throttle
    output += speed_integrator;

    // D term on forward acceleration (provides damping)
    output += D * forward_accel_mps2;

    // Output is throttle percentage (0-100%)
    return constrain_float(output * 100.0f, 0.0f, 100.0f);
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
    // CASCADE ARCHITECTURE:
    // Front wings: feedforward ONLY (decoupled from altitude, reserved for roll)
    // Altitude control now happens via pitch angle, not front wing deflection

    // Front wings: feedforward + roll (NO altitude_pid_out!)
    const float front_collective = feedforward_front;
    const float front_left_norm = front_collective + roll_pid_out;
    const float front_right_norm = front_collective - roll_pid_out;

    // Rear wing: feedforward + pitch (pitch now includes altitude command)
    const float rear_norm = feedforward_rear + pitch_pid_out;

    // Apply per-wing trims (after calculations, before servo conversion)
    // Trims compensate for mechanical misalignment without affecting servo scaling
    const float front_left_trimmed = front_left_norm + plane.g.hydrofoil_trim_front_left.get();
    const float front_right_trimmed = front_right_norm + plane.g.hydrofoil_trim_front_right.get();
    const float rear_trimmed = rear_norm + plane.g.hydrofoil_trim_rear.get();

    // Convert to servo output - EXACT SAME AS BEFORE for roll
    const int16_t left_servo = constrain_int16(front_left_trimmed * 4500.0f, -4500, 4500);
    const int16_t right_servo = constrain_int16(front_right_trimmed * 4500.0f, -4500, 4500);
    const int16_t rear_servo = constrain_int16(rear_trimmed * 4500.0f, -4500, 4500);

    // Output to servos
    // Front left = aileron left, front right = aileron right, rear = elevator
    SRV_Channels::set_output_scaled(SRV_Channel::k_scripting1, left_servo);
    SRV_Channels::set_output_scaled(SRV_Channel::k_scripting3, right_servo);
    SRV_Channels::set_output_scaled(SRV_Channel::k_scripting2, rear_servo);

    // Throttle output (unchanged)
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, speed_pid_out * 100.0f);
}

// ============================================================================
// UTILITY METHODS
// ============================================================================

float ModeHydrofoil::get_filtered_rangefinder_cm()
{
    const uint32_t now_ms = AP_HAL::millis();

    // Check if rangefinder has new data
    if (plane.rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::Status::Good) {
        // Get raw rangefinder distance
        const float raw_distance_cm = plane.rangefinder.distance_orient(ROTATION_PITCH_270) * 100.0f;

        // Subtract sensor ground clearance to get actual altitude above surface
        const float ground_clearance_cm = plane.rangefinder.ground_clearance_orient(ROTATION_PITCH_270) * 100.0f;
        const float reading_cm = raw_distance_cm - ground_clearance_cm;

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

    return true;
}

float ModeHydrofoil::get_vertical_velocity_ms()
{
    // Estimate vertical velocity from pitch and forward speed
    const float pitch_rad = radians(plane.ahrs.pitch_sensor * 0.01f);
    return speed_estimate_ms * sinf(pitch_rad);
}