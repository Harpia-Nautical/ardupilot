# Hydrofoil Custom Flight Mode — Control Theory Reference

> This document describes the theoretical control architecture for a custom ArduPilot flight mode
> for a hydrofoil boat. It covers state machine design, control axis decomposition, feedforward
> theory, PID roles, speed compensation, RC integration, and the acceleration nosedive phenomenon
> observed during manual testing. No implementation code is included — this is a pure theory
> reference for the programmer.

---

## Platform Configuration

| Parameter | Value |
|-----------|-------|
| Total weight | ~37 kg |
| Strut height | ~1.5 m |
| Front wing | Left/right independent control (ailerons) |
| Rear wing | Full collective control + motor mount |
| Rudder | None — roll-to-turn only |
| Weight distribution | ~70% front, ~30% rear |
| Sensors | GPS (speed), ultrasonic rangefinder (~7 Hz), IMU (gyro + accel) |

---

## 1. State Machine

The control laws are fundamentally different at each speed regime. A state machine must gate
which algorithms are active. Transitions are primarily speed-gated, with rangefinder confirming
actual liftoff.

```
IDLE → ACCELERATION RUN → TRANSITION → FOILING → TOUCHDOWN → IDLE
```

See **Section 2** for a detailed explanation of the nosedive phenomenon that drove the addition
of the ACCELERATION RUN state.

### IDLE
Throttle at or near zero. Boat stationary or drifting. No control algorithms active.

- Front wings: neutral AoA
- Rear wing: neutral
- All PIDs inactive
- Exit condition: throttle exceeds threshold

### ACCELERATION RUN
Throttle applied. Hull still on water. Speed building toward foiling threshold.

This is the state where the nosedive phenomenon is most dangerous (see Section 2). Wings must
be actively working from the moment throttle is applied — treating them as passive during the
run is not safe for this hull.

- **Rear wing:** proactive fixed AoA bias (empirically characterized) + pitch PID active at
  full IMU rate. The bias opens immediately at state entry, before any pitch error accumulates.
- **Front wings:** feedforward curve active from the start with a nonzero floor AoA (see
  Section 2 — Feedforward Floor). Partially unloads the bow from the water to reduce
  bow-down hydrodynamic force and lower drag.
- **Roll PID:** active, holding 0° roll. Any wing dip during the run is destabilizing.
- **Altitude loop:** inactive — hull is on water, rangefinder is not meaningful.
- **Throttle:** passthrough.
- Exit condition: speed crosses foiling threshold AND rangefinder begins reading liftoff.

### TRANSITION
Speed at or above foiling threshold. Hull partially airborne.

This is the most unstable regime. Gains should be more aggressive than in FOILING.

- Proactive rear AoA bias fades out as feedforward takes over full responsibility
- Altitude loop activates as rangefinder becomes meaningful
- Pitch PID remains at full IMU rate
- Roll PID active
- Exit condition: rangefinder confirms hull fully off water at stable altitude

### FOILING
Fully foil-borne. All control loops active.

- Full feedforward + PID stack running on all axes
- Altitude hold active (rangefinder outer loop + IMU inner loop)
- Pitch hold at 0° active
- Roll/turn active
- See Sections 3–8 for full detail

### TOUCHDOWN
Speed dropping below foiling threshold.

- Controlled ramp-down of AoA to prevent hard splash
- Pitch PID keeps hull level during descent
- Roll PID holds 0° roll through touchdown
- Exit to IDLE when rangefinder confirms hull contact and speed is low

---

## 2. Acceleration Nosedive Phenomenon

> **Observed behavior:** During manual testing, the craft strongly tends to nosedive when
> throttle is applied unless immediate pitch-up control input is given. The hull cannot build
> speed without the wings generating at least partial lift from the start of the run.

### Causes

Three forces combine during the acceleration run to pitch the bow down:

**Hull hydrodynamics.** As the hull accelerates, dynamic water pressure builds against the bow
entry. The hull form attempts to climb its own bow wave, which loads the bow downward. This
force increases with speed² and is the dominant cause of the nosedive tendency.

**Thrust vector moment.** The motor is mounted at the rear, at or below the waterline. Thrust
is applied rearward and low; the hull's drag resistance is centered higher and distributed
along the hull. This creates a bow-down pitching moment that scales directly with throttle.
More throttle = more bow-down torque before any speed has built.

**Asymmetric foil liftoff.** The front foil carries 70% of the weight and begins generating
meaningful lift at a lower speed than the rear. At very low speed this can cause a brief
bow-up tendency, but hull hydrodynamics dominate and the net result is bow-down.

The compound effect: the more aggressively throttle is applied to build speed quickly, the
stronger the bow-down pitching moment — which increases drag, which requires even more throttle,
which worsens the moment. Without corrective wing input, the boat cannot escape this loop.

### Why Wings Are Not Passive During the Run

At low speed, dynamic pressure on the foils is low (lift scales with v²). The rear wing has
limited authority early in the run. By the time pitch error is large enough for a reactive PID
to respond, the wing still cannot generate enough force to recover quickly.

The rear wing must therefore apply a **proactive bias** — a fixed AoA offset applied
immediately at throttle application, before any pitch error exists. The intent is not to
correct an error but to pre-load a corrective moment that counteracts the known thrust and
hydrodynamic nose-down forces. Think of it as holding back-pressure through the takeoff roll.

### Feedforward Floor (Front Wings)

The standard feedforward curve produces zero (or near-zero) AoA at zero speed, rising as speed
builds. For this hull, that is wrong during the acceleration run. The front wings should be
commanded to a nonzero minimum AoA — a **floor value** — from the moment throttle is applied.

Even at 30–40% of the full foiling AoA, the front wings partially unload the bow from the
water. A lighter bow means less hydrodynamic nose-down force, which means the rear wing has
less pitching moment to fight, which means the run is faster and more stable. The effects
compound positively.

The feedforward curve is therefore:

```
feedforward_front(v) = max(AoA_floor, K_front / v²)
```

Where `AoA_floor` is the empirically determined minimum command during the run. As speed builds
and `K/v²` drops below `AoA_floor`, the curve takes over normally.

### Roll Stability During the Run

Any roll during the acceleration run causes one front strut to dig deeper than the other. At
speed this creates highly asymmetric drag that can yaw and capsize the craft. The roll PID
must hold 0° roll from the start of the ACCELERATION RUN state — not just from FOILING.

### Characterizing the Proactive Rear Bias

The rear bias angle cannot be computed from first principles — it depends on hull form,
thrust line, motor power, and run conditions. Characterize empirically:

1. Start with a conservative bias (small angle)
2. Run at typical takeoff throttle and observe pitch behavior
3. Increase bias until the hull runs level or very slightly bow-up during the run
4. Confirm the bias does not cause the bow to rise uncontrollably at higher throttle settings
5. If throttle level significantly changes the required bias, consider making it a linear
   function of throttle rather than a fixed constant

### Summary of ACCELERATION RUN Control Actions

| Surface | Command |
|---------|---------|
| Rear wing | `proactive_bias + pitch_PID_out` (no feedforward yet — speed too low) |
| Front left | `max(AoA_floor, feedforward_front(v)) + roll_PID_out` |
| Front right | `max(AoA_floor, feedforward_front(v)) − roll_PID_out` |
| ESC | Throttle passthrough |

---

## 3. Control Axis Decomposition

The three control axes are independent and each has a dedicated actuator:

| Axis | Actuator | Primary Sensor | Setpoint |
|------|----------|----------------|----------|
| **Pitch** | Rear wing (collective) | IMU pitch angle + pitch rate | 0° |
| **Altitude** | Front wings (collective symmetric) | Rangefinder (+ IMU prediction) | Target foiling height |
| **Roll / Turn** | Front wings (differential) | IMU roll angle | Pilot stick command |

### Why Pitch Must Be Controlled Separately

The wings are rigidly attached to the struts, which are rigidly attached to the hull. Therefore:

```
effective_AoA = servo_commanded_AoA + hull_pitch_angle
```

If hull pitch is non-zero, every feedforward calculation is wrong by that pitch angle. The entire
control system assumes hull pitch = 0°. The rear wing's sole job in FOILING mode is to enforce
this assumption.

Keeping hull pitch at 0° also means the two front wings are always seeing equal effective AoA
for a given symmetric command — a precondition for clean roll control.

### Final Servo Mixing

At the servo output level:

```
front_left_servo  = feedforward_front(v) + altitude_correction + roll_PID_out
front_right_servo = feedforward_front(v) + altitude_correction − roll_PID_out
rear_servo        = feedforward_rear(v)  + pitch_PID_out
motor/ESC         = RC throttle (full passthrough, no algorithm involvement)
```

`feedforward_front(v)` and `feedforward_rear(v)` are separate curves, since the two wings
have different target lift fractions and potentially different areas.

---

## 4. Feedforward — Core Concept

### What Feedforward Is

Feedforward is a predictive open-loop command based on known physics. It answers:
*"Given current speed, what AoA is theoretically required to support the boat's weight?"*

A PID is purely reactive — it waits for error to appear, then responds. Feedforward computes
the correct command before any error exists. The PID then only corrects the small residual
difference between theory and reality.

### Why It Matters

Without feedforward, as the boat accelerates, lift increases, altitude rises, and the altitude
PID must reduce AoA to compensate. The PID is perpetually chasing a moving target driven by
speed changes. Altitude will oscillate with every acceleration and deceleration event.

With feedforward, the moment speed increases, AoA is reduced immediately — before the boat
has moved. The altitude PID only handles disturbances (waves, model error, gusts), not the
bulk physics load.

**Rule of thumb:** If the PID correction is regularly exceeding 20–30% of the feedforward value
at steady cruise, the feedforward model is wrong and should be re-characterized rather than
compensated with higher PID gains.

### The Lift Equation

```
L = ½ · ρ · v² · CL(α) · A
```

Where:
- `ρ` = fluid density (~1000 kg/m³ freshwater, ~1025 kg/m³ saltwater)
- `v` = foil speed (m/s)
- `CL(α)` = lift coefficient as a function of AoA (linear in normal operating range: CL ≈ CLα · α)
- `A` = wing planform area (m²)

Required total lift = 37 × 9.81 ≈ **363 N**

With the 70/30 split:
- Front wings target: ~254 N
- Rear wing target: ~109 N

### Solving for Required AoA

In the linear regime, solving for α:

```
α_required = (2 · L_target) / (ρ · v² · CLα · A)
```

All terms except `v` are constants for a given wing. This collapses to:

```
α = K / v²
```

Where `K` lumps together all fixed parameters for that wing. This is a hyperbola — steep at low
speed (high AoA required), shallow at high speed (very little AoA required).

`K` is best determined **empirically** rather than from wing geometry calculations, because real
CL, effective area, strut interference, and spray effects are difficult to compute accurately.

### Empirical Characterization of K

1. Hold the boat at target foiling altitude at a known, stable speed
2. Record the servo position at equilibrium — this is the empirical feedforward AoA at that speed
3. Repeat at multiple speeds
4. Fit a `K / v²` curve through the data points
5. Store as a lookup table with linear interpolation, indexed by speed
6. Compute separate tables for front and rear wings

### Minimum Foiling Speed

Below a certain speed, even maximum servo deflection cannot generate 363 N of total lift. This
is the hard floor of the feedforward curve — the minimum speed at which FOILING mode can be
entered. Characterize this experimentally.

---

## 5. Speed Integration

Speed affects the control system in three distinct ways:

| Role | Mechanism | Where It Acts |
|------|-----------|---------------|
| Lift feedforward | `K/v²` AoA curve | Base command, before PIDs |
| Control surface effectiveness | Plant gain scales as `v²` | PID gain scheduling |
| Speed estimation accuracy | GPS + IMU fusion | Input to both of the above |

### Speed Estimation — GPS Lag Problem

GPS speed typically has 1+ second of lag. At 8 m/s foiling speed, this means the feedforward
is working from speed data that's 8+ meters stale. During acceleration, the feedforward thinks
the boat is slower than it is, commands too much AoA, and altitude rises — the altitude PID
must then compensate.

**Mitigation:** Fuse GPS speed with IMU longitudinal acceleration integration between GPS
updates. Use GPS as the low-frequency truth anchor; use IMU integration for high-frequency
short-term accuracy. This is a standard complementary filter:

```
v_estimate = α · (v_gps_last + accel_integrated_since_last_fix)
           + (1 − α) · v_gps_current  (applied each new GPS fix)
```

### Control Surface Effectiveness Scaling

The lift change per degree of servo deflection is:

```
ΔL/Δα = ½ · ρ · v² · CLα · A
```

This scales with `v²`. At twice the speed, the same servo deflection produces four times the
lift change. A PID tuned at low speed will be dangerously over-aggressive at high speed.

**Correct solution — Gain Scheduling:**

Scale PID gains inversely with `v²` so total loop gain stays constant:

```
P_actual = P_tuned · (v_ref / v)²
I_actual = I_tuned · (v_ref / v)²
D_actual = D_tuned · (v_ref / v)²
```

Where `v_ref` is the speed at which the PIDs were originally tuned. This makes the PID behavior
identical at all speeds from the pilot's perspective — tune once, consistent everywhere.

---

## 6. Pitch Control Loop (Rear Wing)

**Goal:** Keep hull pitch at 0° at all times during FOILING.

**Inputs:** IMU pitch angle, IMU pitch rate  
**Output:** Rear wing collective servo command

### Structure

```
pitch_error = 0° − current_pitch_angle
pitch_PID_out = P · pitch_error + I · ∫pitch_error + D · pitch_rate
rear_servo = feedforward_rear(v) + pitch_PID_out
```

The D term is applied directly to pitch rate from the gyro (not to the derivative of pitch
error) for cleaner, lower-noise damping.

### Liftoff Behavior

During TRANSITION, the hull naturally pitches bow-up because the front foil (carrying 70% of
weight) breaks free of the water first. The pitch PID immediately sees a positive pitch error
and increases rear AoA. This both corrects attitude *and* assists the rear strut in breaking
free. Assisted rear liftoff is an emergent benefit of the pitch controller — no special-case
logic is needed.

### Loop Speed

Pitch loop runs at full IMU update rate. This is the fastest loop in the system. It must be
significantly faster (3–5×) than the altitude loop bandwidth to prevent coupled oscillation.

---

## 7. Altitude Control Loop (Front Wings — Collective)

**Goal:** Maintain hull at target foiling altitude above water surface.

**Inputs:** Rangefinder (7 Hz), IMU pitch + speed (for vertical rate estimation)  
**Output:** Symmetric (collective) trim added to both front wing servos

### The 7 Hz Problem

A raw 7 Hz outer loop is too slow for stable altitude control. Between rangefinder updates,
vertical motion continues. The solution is a two-rate cascaded structure:

**Outer loop (7 Hz — rangefinder-gated):**
- Computes altitude error: `error = target_alt − rangefinder_reading`
- P and I terms generate a target correction
- I term handles persistent steady-state bias (e.g. consistently running 5 cm low)

**Inner loop (IMU rate — predictor):**
- Between rangefinder updates, estimate vertical velocity:
  ```
  v_vertical ≈ sin(hull_pitch) · v_forward
  ```
  (This is near-zero since hull pitch is held at 0°, but provides D-term damping during
  transients)
- D term on estimated vertical velocity provides damping, preventing altitude hunting
- GPS/IMU-fused speed used here for accuracy

The rangefinder is the **truth anchor** (low frequency). The IMU is the **predictor** between
truth updates (high frequency). This is a complementary filter in the time domain.

### Rangefinder Noise Mitigation

At foiling speed over choppy water, the rangefinder may return surface wave heights rather
than mean water level. Apply a median filter or low-pass filter to rangefinder readings before
using them as altitude truth. Tune the filter cutoff carefully — too aggressive and you lose
real altitude information; too gentle and wave noise enters the loop.

### Coupling With Pitch Loop

Commanding more front collective AoA raises the bow slightly, which the pitch loop immediately
corrects by raising rear AoA. The loops settle each other naturally because pitch bandwidth is
much higher than altitude bandwidth.

### Coordinated Turn Compensation (Optional)

When banking, the vertical component of total lift decreases:

```
vertical_lift = total_lift · cos(roll_angle)
```

To prevent altitude loss in turns, increase collective AoA proportionally:

```
altitude_correction_in_turn = altitude_correction_base / cos(roll_angle)
```

This keeps altitude constant through banked turns. Omit for initial implementation; add once
baseline loops are stable.

---

## 8. Roll Control Loop (Front Wings — Differential)

**Goal:** Bank the hull to a commanded roll angle; turning is achieved via banking.

**Inputs:** IMU roll angle, pilot roll stick  
**Output:** Differential command added/subtracted to front wing servos

### Structure

```
roll_setpoint = map(pilot_stick, ±max_bank_angle)   // e.g. ±20°
roll_error = roll_setpoint − current_roll_angle
roll_PID_out = P · roll_error + I · ∫roll_error + D · roll_rate

front_left  += roll_PID_out
front_right −= roll_PID_out
```

Roll PID output is purely differential — it does not affect collective and therefore does not
contaminate the altitude loop. The sum of left and right commands remains constant.

### Turning Dynamics

No rudder means turning like an aircraft — bank to turn. Turn radius is a function of speed
and bank angle. At higher speed, more bank is needed for the same turn rate. This is physics,
not a control problem.

---

## 9. RC Override Integration

RC inputs are treated as **setpoint commands**, not direct servo overrides. The control
algorithms always run; the pilot commands what they want the system to target.

| RC Channel | In FOILING Mode | In ACCELERATION RUN / IDLE |
|------------|-----------------|---------------------------|
| Throttle | 100% passthrough — bypasses all logic | 100% passthrough |
| Roll stick | Sets roll angle setpoint (e.g. ±20°) | Ignored — roll PID holds 0° |
| Pitch stick | Shifts target altitude up/down (e.g. ±20 cm) | Ignored — pitch PID + bias in control |

This means the pilot always has full authority — by commanding setpoints, the PIDs chase
whatever is commanded. Maximum servo authority is retained at all times.

A dedicated mode switch to IDLE (full manual) should be available at all times as a
safety fallback.

---

## 10. Loop Bandwidth Hierarchy

To prevent coupled oscillations between loops, maintain a clear bandwidth separation:

```
Pitch loop (IMU rate)    >>  Altitude loop (7 Hz outer / IMU inner)  >>  RC input bandwidth
       fastest                          middle                                  slowest
```

Recommended minimum ratio: pitch loop bandwidth ≥ 3–5× altitude loop bandwidth.

If this separation is violated, the pitch and altitude loops will fight each other, producing
a coupled oscillation that is difficult to diagnose and cannot be resolved by PID tuning alone.

---

## 11. Unknowns to Characterize Empirically

The following cannot be reliably computed from first principles and must be measured from the
actual boat:

| Unknown | Why It Matters | How to Characterize |
|---------|---------------|---------------------|
| CLα for each foil | Feedforward curve accuracy | Steady-speed equilibrium servo angle at multiple speeds |
| K constant (front) | Front feedforward curve | Same as above — front servo position at equilibrium |
| K constant (rear) | Rear feedforward curve | Same as above — rear servo position at equilibrium |
| Minimum foiling speed | State machine transition threshold | Lowest speed at which boat fully lifts off |
| Rangefinder noise floor | Filter cutoff selection | Log raw rangefinder at speed, observe noise amplitude |
| Transition instability band | State machine gain scheduling | Speed range where gains need to be most aggressive |
| Rear wing proactive bias angle | ACCELERATION RUN pitch control | Incremental runs at takeoff throttle; increase until hull runs level |
| Bias throttle-dependence | Whether bias should be fixed or throttle-scaled | Compare required bias at low vs high takeoff throttle settings |
| Front AoA floor value | Feedforward floor during the run | Minimum front AoA that meaningfully unloads the bow without handling issues |

---

## 12. Known Coupling Effects

| Coupling | Cause | Mitigation |
|----------|-------|-----------|
| Altitude → Pitch | More front collective lifts bow slightly | Pitch loop faster than altitude loop |
| Pitch → Altitude | Rear correction adds total lift slightly | Self-limiting; loops settle naturally |
| Speed → Both loops | v² plant gain change | Gain scheduling (Section 5) |
| Roll → Altitude | cos(bank) reduces vertical lift component | Coordinated turn compensation (Section 7) |
| Throttle → Pitch (run) | Thrust line below drag center creates bow-down moment | Proactive rear bias scaled to throttle if needed |
| Front floor AoA → Pitch (run) | Partial front lift raises bow slightly | Captured by pitch PID; self-correcting |

---

## Summary: What Each Component Does

### In ACCELERATION RUN
```
Throttle             →  motor ESC  (passthrough)
IMU pitch            →  pitch PID  →  rear wing (proactive_bias + pitch_PID_out)
IMU roll             →  roll PID   →  front differential (holds 0°)
Speed                →  max(AoA_floor, feedforward_front(v))  →  front collective

Front left  = max(AoA_floor, feedforward_front(v)) + roll_PID_out
Front right = max(AoA_floor, feedforward_front(v)) − roll_PID_out
Rear        = proactive_bias + pitch_PID_out
ESC         = throttle (passthrough)
```

### In FOILING
```
GPS + IMU  →  v_estimate  →  feedforward_front(v), feedforward_rear(v)
                          →  PID gain scheduling factor (v_ref/v)²

Rangefinder + IMU  →  altitude error  →  altitude PID  →  front collective trim
IMU pitch          →  pitch error     →  pitch PID     →  rear collective trim
IMU roll           →  roll error      →  roll PID      →  front differential trim
Pilot throttle     →  motor ESC       (direct passthrough)
Pilot roll stick   →  roll setpoint   (PID chases this)
Pilot pitch stick  →  altitude setpoint offset (PID chases this)

Front left  = feedforward_front(v) + altitude_trim + roll_out
Front right = feedforward_front(v) + altitude_trim − roll_out
Rear        = feedforward_rear(v)  + pitch_out
ESC         = throttle (passthrough)
```
