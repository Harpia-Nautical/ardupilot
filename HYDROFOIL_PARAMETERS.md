# Hydrofoil Mode Parameters Reference

This document describes all parameters used by the HYDROFOIL flight mode in ArduPlane.

## Table of Contents
- [State Machine Parameters](#state-machine-parameters)
- [Feedforward Parameters](#feedforward-parameters)
- [PID Controller Gains](#pid-controller-gains)
- [Hardware Configuration](#hardware-configuration)

---

## State Machine Parameters

### HFOL_TARGET_ALT
- **Description**: Target altitude above water surface when foiling, measured by rangefinder
- **Units**: cm (centimeters)
- **Default**: 50
- **Range**: Typically 20-200 cm depending on hydrofoil design
- **User Level**: Standard
- **Notes**: This is the setpoint altitude that the altitude controller maintains during FOILING state. Can be adjusted in-flight using pitch stick (see HFOL_ALT_STICK).

### HFOL_MIN_SPEED
- **Description**: Minimum speed required to sustain foiling flight
- **Units**: m/s (meters per second)
- **Default**: 6.0
- **Range**: Typically 4-15 m/s depending on hydrofoil design
- **User Level**: Standard
- **Notes**: Used for state transitions. If speed drops below this threshold during FOILING, the mode transitions to TOUCHDOWN. Set this based on your hydrofoil's minimum planing speed.

### HFOL_LIFTOFF_CM
- **Description**: Rangefinder altitude threshold to detect liftoff from water
- **Units**: cm (centimeters)
- **Default**: 15
- **Range**: Typically 10-50 cm
- **User Level**: Advanced
- **Notes**: During ACCELERATION_RUN, when rangefinder reading exceeds this value AND speed exceeds minimum, the mode transitions to TRANSITION state.

### HFOL_THR_MIN
- **Description**: Minimum throttle percentage to trigger acceleration run state
- **Units**: % (percentage)
- **Default**: 10
- **Range**: 0-100
- **User Level**: Standard
- **Notes**: When in IDLE state, throttle input above this threshold triggers transition to ACCELERATION_RUN. Set high enough to prevent accidental triggering.

---

## Acceleration Run Parameters

### HFOL_REAR_BIAS
- **Description**: Proactive bias applied to rear wing during acceleration run to counteract nosedive tendency
- **Units**: Normalized control output (-1.0 to +1.0)
- **Default**: 0.2
- **Range**: 0.0 to 1.0
- **User Level**: Standard
- **Notes**: Applied as a constant offset to the rear wing during ACCELERATION_RUN state. Positive values create nose-up moment. Prevents bow-diving during the critical acceleration phase. Tune empirically by observing behavior during acceleration.

### HFOL_FRONT_FLOOR
- **Description**: Minimum front wing deflection during acceleration run to unload bow
- **Units**: Normalized control output (-1.0 to +1.0)
- **Default**: 0.1
- **Range**: 0.0 to 1.0
- **User Level**: Standard
- **Notes**: Sets a minimum AoA for front wings during ACCELERATION_RUN to help lift the bow and reduce drag. Positive values create upward force at bow. Works together with HFOL_REAR_BIAS to manage pitch during acceleration.

---

## Feedforward Parameters

### HFOL_K_FRONT
- **Description**: Empirically determined K constant for front wing feedforward curve (AoA = K/v²)
- **Units**: Dimensionless (relates to rad·(m/s)²)
- **Default**: 80.0
- **Range**: 10-200 (highly dependent on hydrofoil geometry)
- **User Level**: Advanced
- **Notes**: Characterize by recording equilibrium servo positions at multiple steady-state speeds during manual flight, then fit to K/v² curve. This feedforward compensates for speed-dependent lift requirements.

### HFOL_K_REAR
- **Description**: Empirically determined K constant for rear wing feedforward curve (AoA = K/v²)
- **Units**: Dimensionless (relates to rad·(m/s)²)
- **Default**: 40.0
- **Range**: 10-200 (highly dependent on hydrofoil geometry)
- **User Level**: Advanced
- **Notes**: Same characterization process as HFOL_K_FRONT. Rear wing typically requires less AoA than front wings due to different loading and geometry.

**Feedforward Characterization Procedure:**
1. Fly manually at various steady speeds (6-12 m/s)
2. Record servo positions and speeds when maintaining stable altitude
3. Plot servo position vs 1/v²
4. K = slope of linear fit
5. Update HFOL_K_FRONT and HFOL_K_REAR parameters

---

## PID Controller Gains

All PID gains use normalized control outputs (-1.0 to +1.0). The gains are significantly smaller than traditional degree-based PIDs because the normalized values are converted to servo units (×4500) internally.

### Pitch Controller (Rear Wing)

Controls pitch attitude to maintain level flight. Runs at IMU rate (~400 Hz).

#### HFOL_PITCH_P
- **Description**: P gain for pitch controller (rear wing)
- **Units**: Normalized output per degree error
- **Default**: 0.133
- **Range**: 0.0 to 1.0
- **User Level**: Advanced
- **Notes**: Start with default and increase until oscillations appear, then reduce by 30%. Higher values = more aggressive pitch correction.

#### HFOL_PITCH_I
- **Description**: I gain for pitch controller (rear wing)
- **Units**: Normalized output per degree-second error
- **Default**: 0.033
- **Range**: 0.0 to 0.2
- **User Level**: Advanced
- **Notes**: Eliminates steady-state pitch errors. Too high causes slow oscillations and overshoot. Typical value is ~25% of P gain.

#### HFOL_PITCH_D
- **Description**: D gain for pitch controller (rear wing)
- **Units**: Normalized output per degree/second pitch rate
- **Default**: 0.02
- **Range**: 0.0 to 0.5
- **User Level**: Advanced
- **Notes**: Adds damping to pitch response. Too high amplifies noise. Typical value is ~15% of P gain.

**Pitch PID Tuning Tips:**
- Tune at reference speed (HFOL_GAIN_REF_SPD)
- Target ~1-2 Hz bandwidth with minimal overshoot
- Verify stability across full speed range due to gain scheduling

---

### Altitude Controller (Front Wings Collective)

Controls altitude above water by commanding pitch setpoint. Runs at rangefinder update rate (~10-20 Hz). This is the outer loop in a cascaded control structure.

#### HFOL_ALT_P
- **Description**: P gain for altitude controller (front wings collective)
- **Units**: Degrees per cm altitude error
- **Default**: 0.15
- **Range**: 0.0 to 1.0
- **User Level**: Advanced
- **Notes**: Converts altitude error to pitch command. Start conservative and increase for faster altitude response. Too high causes altitude oscillations.

#### HFOL_ALT_I
- **Description**: I gain for altitude controller (front wings collective)
- **Units**: Degrees per cm-second altitude error
- **Default**: 0.03
- **Range**: 0.0 to 0.2
- **User Level**: Advanced
- **Notes**: Eliminates steady-state altitude offset. Set to ~20% of P gain. Too high causes pumping motion.

#### HFOL_ALT_D
- **Description**: D gain for altitude controller (front wings collective)
- **Units**: Degrees per cm/s altitude rate
- **Default**: 0.02
- **Range**: 0.0 to 0.5
- **User Level**: Advanced
- **Notes**: Provides damping based on vertical velocity. Uses complementary filter (pitch + speed) for velocity estimation. Helps prevent altitude overshoot.

**Altitude PID Tuning Tips:**
- Tune AFTER pitch PID is stable (cascaded control)
- Lower bandwidth than pitch loop (~0.5-1 Hz)
- Test with step inputs and verify no sustained oscillations
- May need adjustment if rangefinder filtering is changed

---

### Roll Controller (Front Wings Differential)

Controls roll angle for turns and disturbance rejection. Runs at IMU rate (~400 Hz).

#### HFOL_ROLL_P
- **Description**: P gain for roll controller (front wings differential)
- **Units**: Normalized output per degree roll error
- **Default**: 0.25
- **Range**: 0.0 to 1.0
- **User Level**: Advanced
- **Notes**: Higher values = tighter roll control. Start with default and increase until oscillations appear, then back off.

#### HFOL_ROLL_I
- **Description**: I gain for roll controller (front wings differential)
- **Units**: Normalized output per degree-second roll error
- **Default**: 0.04
- **Range**: 0.0 to 0.2
- **User Level**: Advanced
- **Notes**: Compensates for steady-state roll trim errors (e.g., weight imbalance). Too high causes Dutch roll.

#### HFOL_ROLL_D
- **Description**: D gain for roll controller (front wings differential)
- **Units**: Normalized output per degree/second roll rate
- **Default**: 0.01
- **Range**: 0.0 to 0.5
- **User Level**: Advanced
- **Notes**: Adds roll damping. Lower than pitch D because roll is typically less dynamic. Increase if roll feels underdamped.

**Roll PID Tuning Tips:**
- Tune while flying straight at cruise speed
- Test coordinated turns at HFOL_MAX_BANK angle
- Should return to 0° quickly after disturbances
- May couple with pitch—tune pitch first

---

## Gain Scheduling

### HFOL_GAIN_REF_SPD
- **Description**: Reference speed for PID gain scheduling. PIDs are tuned at this speed.
- **Units**: m/s (meters per second)
- **Default**: 8.0
- **Range**: Typically 6-15 m/s
- **User Level**: Advanced
- **Notes**: Gain scheduling automatically adjusts PID gains across the speed range to maintain consistent control response. Set this to your typical cruising speed where you tune the PIDs. Gains are scaled as (v_ref/v) at other speeds.

**Gain Scheduling Formula:**
```
actual_gain = tuned_gain × (reference_speed / current_speed)
```

This compensates for speed-dependent changes in control effectiveness.

---

## Flight Control Parameters

### HFOL_MAX_BANK
- **Description**: Maximum bank angle for turns in foiling mode
- **Units**: deg (degrees)
- **Default**: 20.0
- **Range**: 5-45 degrees
- **User Level**: Standard
- **Notes**: Roll stick input is scaled to this maximum bank angle. Reduce for gentler turns and more stability. Increase for tighter turns (but verify structural limits).

### HFOL_ALT_STICK
- **Description**: Range of altitude adjustment from pitch stick input
- **Units**: cm (centimeters)
- **Default**: 20.0
- **Range**: 10-100 cm
- **User Level**: Standard
- **Notes**: Pitch stick deflection adjusts target altitude ±HFOL_ALT_STICK/2 from HFOL_TARGET_ALT. Full forward stick = -10cm, full back = +10cm (with default 20cm range). Allows pilot to adjust ride height during flight.

---

## Hardware Configuration

### Rangefinder Requirements

The hydrofoil mode requires a rangefinder with the following configuration:

- **RNGFND1_TYPE**: Must be set to appropriate sensor type (e.g., 48 for MaxBotixSerial)
- **RNGFND1_ORIENT**: Must be 25 (ROTATION_PITCH_270, down-facing)
- **RNGFND1_MIN_CM**: Minimum range (typically 10-30 cm)
- **RNGFND1_MAX_CM**: Maximum range (must exceed maximum foiling altitude, typically 500-1000 cm)

The mode will refuse to arm or enter if:
- No rangefinder is detected with orientation 25
- Rangefinder status is NotConnected or NoData

---

## Parameter Tuning Workflow

### Initial Setup
1. Set **HFOL_TARGET_ALT** based on your design target altitude
2. Set **HFOL_MIN_SPEED** based on minimum planing speed
3. Set **HFOL_MAX_BANK** conservatively (15-20°)
4. Leave feedforward K values at defaults initially

### Acceleration Phase Tuning
1. Test acceleration runs manually
2. Adjust **HFOL_REAR_BIAS** if nose dives during acceleration (increase) or pitches up too much (decrease)
3. Adjust **HFOL_FRONT_FLOOR** to prevent bow from digging in

### Feedforward Characterization
1. Fly manually at 3-5 different steady speeds (6-12 m/s)
2. Record equilibrium servo positions and speeds
3. Fit data to K/v² curve to determine **HFOL_K_FRONT** and **HFOL_K_REAR**

### PID Tuning Order
1. **Set HFOL_GAIN_REF_SPD** to your primary tuning speed
2. **Tune Pitch PID** (HFOL_PITCH_P/I/D) first—this is the inner loop
3. **Tune Altitude PID** (HFOL_ALT_P/I/D) second—this is the outer loop
4. **Tune Roll PID** (HFOL_ROLL_P/I/D) last—independent from pitch/altitude
5. **Verify gain scheduling** works across full speed range

### Final Validation
1. Test all state transitions (IDLE → ACCELERATION_RUN → TRANSITION → FOILING → TOUCHDOWN)
2. Test altitude hold at various speeds
3. Test coordinated turns up to HFOL_MAX_BANK
4. Test disturbance rejection (waves, wind gusts)
5. Test failsafe scenarios (rangefinder loss, low speed)

---

## Units Summary Table

| Parameter | Units | Normalized? | Default |
|-----------|-------|-------------|---------|
| HFOL_TARGET_ALT | cm | No | 50 |
| HFOL_MIN_SPEED | m/s | No | 6.0 |
| HFOL_REAR_BIAS | -1 to +1 | Yes | 0.2 |
| HFOL_FRONT_FLOOR | -1 to +1 | Yes | 0.1 |
| HFOL_MAX_BANK | deg | No | 20.0 |
| HFOL_ALT_STICK | cm | No | 20.0 |
| HFOL_GAIN_REF_SPD | m/s | No | 8.0 |
| HFOL_THR_MIN | % | No | 10 |
| HFOL_LIFTOFF_CM | cm | No | 15 |
| HFOL_K_FRONT | dimensionless | No | 80.0 |
| HFOL_K_REAR | dimensionless | No | 40.0 |
| HFOL_PITCH_P | normalized/deg | Yes | 0.133 |
| HFOL_PITCH_I | normalized/deg·s | Yes | 0.033 |
| HFOL_PITCH_D | normalized/(deg/s) | Yes | 0.02 |
| HFOL_ALT_P | deg/cm | Yes | 0.15 |
| HFOL_ALT_I | deg/cm·s | Yes | 0.03 |
| HFOL_ALT_D | deg/(cm/s) | Yes | 0.02 |
| HFOL_ROLL_P | normalized/deg | Yes | 0.25 |
| HFOL_ROLL_I | normalized/deg·s | Yes | 0.04 |
| HFOL_ROLL_D | normalized/(deg/s) | Yes | 0.01 |

---

## Notes on Normalized Control Outputs

Parameters marked as "normalized" use values in the range -1.0 to +1.0, which are internally converted to servo PWM values:

```
servo_pwm = 1500 + (normalized_value × 4500)
```

Where:
- -1.0 → 1000 PWM (minimum/negative max)
- 0.0 → 1500 PWM (neutral)
- +1.0 → 2000 PWM (maximum/positive max)

This normalization makes the control system hardware-agnostic and allows for clean PID tuning without worrying about specific servo scales or physical angle limits.

---

## Logging and Telemetry

The following data is available via MAVLink and should be logged for tuning:

- Current state (IDLE, ACCELERATION_RUN, TRANSITION, FOILING, TOUCHDOWN)
- Rangefinder altitude (filtered)
- Speed estimate (GPS+IMU fusion)
- Target altitude and current altitude error
- Pitch angle and pitch rate
- Roll angle and roll rate
- PID outputs (pitch, altitude, roll)
- Feedforward outputs (front, rear)
- Servo commands (front left, front right, rear, throttle)

Enable full logging with `LOG_BITMASK = 65535` during initial tuning.
