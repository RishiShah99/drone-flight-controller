# AI Handoff Prompt: Diagnose Immediate Corner Lean in `New_v16.cpp`

You are diagnosing a brushed-quad ESP32 flight-controller issue. Produce a root-cause analysis and a concrete fix plan. Do not default to generic PID advice. Work from the exact facts below.

## Problem Statement

The quad behaves differently in two modes:

- With `ALL_MOTORS_SAME_THRUST_MODE` enabled, the drone lifts mostly flat for the first `10-15 cm`, then eventually tips because stabilization is bypassed.
- With normal stabilized flight mode enabled, the drone leans immediately toward one corner, often the `GPIO25` corner.

This strongly suggests the problem is in the stabilized control path, motor mapping/sign conventions, or asymmetric closed-loop motor commands, not just "lack of thrust".

## Hard Facts

- Authoritative physical motor layout:
  - `GPIO25 = front-left`
  - `GPIO26 = front-right`
  - `GPIO27 = back-right`
  - `GPIO14 = back-left`
- Current firmware under review:
  - `C:\Users\ferrm\Downloads\1B\drone\Clear versions\New_v16.cpp`
- Controller telemetry source:
  - `C:\Users\ferrm\Downloads\1B\drone\Clear versions\controller_v13.cpp`
- Reference mixer source:
  - `C:\Users\ferrm\Downloads\1B\drone\Firmware condidates\dRehmFlight-master\dRehmFlight-master\Versions\dRehmFlight_Teensy_BETA_1.0\dRehmFlight_Teensy_BETA_1.0.ino`

## Confirmed Code Facts

### 1. Current symbolic mixer matches the dRehmFlight quad-X mixer

Reference dRehm mixer:

```cpp
m1 = thro - pitch + roll + yaw;
m2 = thro - pitch - roll - yaw;
m3 = thro + pitch - roll + yaw;
m4 = thro + pitch + roll - yaw;
```

Current `New_v16.cpp` mixer:

```cpp
m1_cmd = thro_des - pitch_PID + roll_PID + yaw_PID;
m2_cmd = thro_des - pitch_PID - roll_PID - yaw_PID;
m3_cmd = thro_des + pitch_PID - roll_PID + yaw_PID;
m4_cmd = thro_des + pitch_PID + roll_PID - yaw_PID;
```

Relevant lines in `New_v16.cpp`:

- Pins: `83-86`
- Mixer: `993-1011`

Conclusion: the symbolic mixer algebra itself currently matches the dRehm reference. If the quad still leans instantly in stabilized mode, the highest-probability causes are not "the raw mixer formula is random", but one of:

- frame-to-motor identity mismatch elsewhere
- wrong correction direction / FC alignment / IMU-frame alignment
- asymmetric motor scaling
- weak corner / prop / motor / driver issue under load
- non-neutral command bias

### 2. There is intentional per-motor asymmetry in normal mode

`scaleToDuty()` does not scale all motors equally:

```cpp
m1_duty = (int)lroundf(m1_cmd * 255.0f * 0.98);
m2_duty = (int)lroundf(m2_cmd * 255.0f);
m4_duty = (int)lroundf(m4_cmd * 255.0f);
m3_duty = (int)lroundf(m3_cmd * 255.0f);
```

Relevant lines:

- `1018-1024`

This means `M1` / `GPIO25` is intentionally reduced by `2%` in normal stabilized mode.

That alone may not fully explain an immediate strong lean, but it is a real built-in bias and must be removed from the diagnosis before blaming PID.

### 3. `ALL_MOTORS_SAME_THRUST_MODE` is not actually equal-thrust

The code comments say "apply exactly the same duty to all motors", but the implementation does this:

```cpp
m1_duty = 0.98 * thrust_duty;
m2_duty = thrust_duty;
m3_duty = 0.99 * thrust_duty;
m4_duty = 0.99 * thrust_duty;
```

Relevant lines:

- `1315-1319`

So the diagnostic "equal-thrust mode" is currently biased too. Even though the quad still lifts mostly flat there, this mode is not a clean proof that all four outputs are truly equal.

### 4. The IMU/body-axis remap comments disagree with the code

In `seedAttitudeFromCurrentAccel()` the comments say:

```cpp
// X_body =  AccY
// Y_body = -AccX
```

But the actual code is:

```cpp
float ax = -AccY;
float ay =  AccX;
float az =  AccZ;
```

Relevant lines:

- comments: `709-714`
- actual code: `715-717`

This is important because prior debugging already found sign/orientation confusion around roll and pitch. The code, not the comments, is the source of truth.

### 5. The current control path still depends on the chosen gyro/accel remap

Relevant lines:

- desired-state mapping: `882-896`
- roll/pitch derivative mapping: `926-944`
- Madgwick call in loop: `1460-1461`
- accel seeding path: `708-750`

This means the stabilized corner-lean problem can still be caused by a control-direction mismatch even if one earlier hand-tilt test "looked close enough".

## Important Interpretation

The fact that same-thrust mode lifts mostly flat means:

- gross thrust symmetry is probably at least close enough for a short straight lift
- the immediate corner lean in normal mode is more likely caused by stabilization commanding the wrong output pattern, or by a built-in output asymmetry being amplified by the controller

It does not prove that hardware is perfect. A weak motor/prop/driver corner can still show up more clearly once the closed-loop controller starts demanding differential thrust.

## Ranked Likely Causes

### 1. Wrong correction direction or FC/IMU alignment mismatch

Why this is high probability:

- equal-thrust mode is mostly okay
- normal mode fails immediately
- official FC guidance says wrong FC orientation causes the PID controller to misinterpret flight attitude and corrective actions

What this would look like:

- the quad "tries to correct" but pushes the wrong corner
- pure edge-tilt tests may fail even if corner tilts seem plausible

### 2. Built-in motor-output asymmetry in the firmware

Why this matters:

- `GPIO25` is already reduced in normal mode
- equal-thrust mode is also biased
- the user often reports leaning toward the `GPIO25` corner

This asymmetry may be the whole problem or an amplifier of another problem.

### 3. Weak corner under load: motor, prop, driver channel, or wiring

Why this remains possible:

- a corner can look "close enough" in same-thrust mode but still underperform once the controller asks for fast thrust changes
- immediate leaning toward a recurring corner can come from a weak corner, wrong prop on one corner, or a driver/output issue

### 4. Non-neutral control commands from the controller path

Why this must be checked:

- if telemetry does not show true neutral sticks, the FC can be commanding a lean even while sitting on the ground
- earlier logs in this project already showed cases where calibration happened while non-zero command values were still present

### 5. Misleading corner-tilt interpretation

Corner tilts are not a clean sign test. A corner-down posture mixes roll and pitch simultaneously, so it is not valid to expect both adjacent low-corner motors to always be the top pair. Pure edge tilts must be used for sign verification.

## Research-Backed Takeaways

### Betaflight: FC orientation must match the frame

Betaflight's FC gyro orientation guide says the firmware must know the FC axes relative to frame axes, otherwise the PID controller will misinterpret flight attitude and corrective actions.

Source:

- [Betaflight: Flight Controller Gyro Orientation](https://www.betaflight.com/docs/wiki/guides/current/Flight-Controller-Gyro-Orientation)

### ArduPilot: immediate flip is usually setup, not tuning

ArduPilot troubleshooting says a copter that tilts/flips over on takeoff is almost always configured or set up wrong, and explicitly points to:

- wrong motor order
- wrong motor direction
- wrong props
- wrong board orientation
- bad calibration / non-flat startup

Source:

- [ArduPilot Copter Troubleshooting (archived)](https://ardupilot.org/copter/docs/troubleshooting.html)

### ArduPilot community: even after "usual suspects" are checked, setup/path issues are still the first place to look

Relevant example:

- [ArduPilot Discourse: "Quadcopter flips over on takeoff. All usual suspects checked. SOLVED"](https://discuss.ardupilot.org/t/quadcopter-flips-over-on-takeoff-all-usual-suspects-checked-solved/116156)

Use this as supporting evidence that this symptom family is usually caused by mapping/orientation/setup faults before it is caused by PID gains.

### x-io Fusion: gain/recovery affects convergence, not this corner-lean symptom

The x-io Fusion documentation explains that higher startup/recovery gain helps orientation rapidly converge from an arbitrary initial value, and acceleration rejection/recovery control when accel is ignored or trusted.

This is relevant because it helps separate two issues:

- slow return of roll/pitch to zero after moving the drone by hand
- immediate corner lean during stabilized takeoff

The first is an estimator/recovery issue.
The second is more likely a control-direction / mapping / asymmetric-output issue.

Source:

- [x-io Fusion](https://github.com/xioTechnologies/Fusion)

## Required Test Matrix

Run these tests in order. Do not skip to PID tuning first.

### 1. Neutral-command telemetry check

Props off. Drone flat. Controller centered.

Verify telemetry shows:

- `thr=0`
- `yaw=0`
- `pitch=0`
- `roll=0`
- `IMU r/p` near zero

If not:

- controller centering / deadband / mapping is still injecting commands

### 2. Pure edge tilt tests only

Do not use corner tilts for sign verification.

Expected behavior for this exact frame:

- front low -> `GPIO25` and `GPIO26` should rise
- back low -> `GPIO14` and `GPIO27` should rise
- left low -> `GPIO25` and `GPIO14` should rise
- right low -> `GPIO26` and `GPIO27` should rise

Expected angle signs:

- front low -> `pitch < 0`
- back low -> `pitch > 0`
- left low -> `roll < 0`
- right low -> `roll > 0`

If these fail:

- the issue is still in control direction / FC alignment / IMU-frame mapping

### 3. Re-test "same thrust" only after removing code-level asymmetry from the diagnosis

Before treating same-thrust mode as valid evidence, remember:

- current code scales `M1` by `0.98`
- current code scales `M3` and `M4` by `0.99`

Interpretation:

- if equal-thrust mode looks flat even with these biases, the gross thrust mismatch is probably not huge
- but same-thrust mode is not a clean symmetry proof until all four outputs are truly identical

### 4. Does the problem follow hardware or stay with the code-defined corner?

Swap one of the following:

- motor + prop between corners, or
- output channel / driver connection between corners

Then re-test.

Interpretation:

- if the lean follows the physical motor/prop, the problem is hardware
- if the lean stays on the same code-defined corner, the problem is in mapping/control/output logic

### 5. Optional cleanup test: temporarily zero yaw gains for tilt-sign testing

Only for props-off sign validation:

- `Kp_yaw = 0`
- `Ki_yaw = 0`
- `Kd_yaw = 0`

This removes yaw contamination while validating roll/pitch correction direction.

## Recommended First Fixes Before Any PID Retune

Do these first, in order:

1. Remove all intentional per-motor scaling from the diagnosis path.
   - normal mode: remove `M1 * 0.98`
   - equal-thrust mode: remove `0.98/0.99` reductions
2. Re-run neutral telemetry verification.
3. Re-run pure edge tilt tests only.
4. Confirm motor order, spin direction, and prop type against the real frame.
5. Only after the above passes, do a short stabilized lift test.

Do not start by increasing or decreasing PID just because the quad leans to a corner. A recurring immediate corner lean is more often setup/mapping/asymmetry than tuning.

## What the Implementer Should Audit in `New_v16.cpp`

- Pins and authoritative motor identity:
  - `83-86`
- IMU/body-axis remap comments vs code:
  - `709-717`
- Accel-seeded roll/pitch convention:
  - `732-750`
- Control derivative sign conventions:
  - `926-944`
- Mixer:
  - `993-1011`
- Normal-mode duty scaling:
  - `1018-1024`
- Same-thrust test-mode scaling:
  - `1315-1319`
- Main fusion call:
  - `1460-1461`
- Controller-to-desired-state path:
  - `882-896`

## Bottom Line

Treat this as a stabilized-control-path diagnosis, not a "tune PID from scratch" problem.

The highest-value next actions are:

- eliminate code-level motor scaling bias
- validate pure edge correction direction against the real frame
- verify neutral command inputs
- determine whether the bad behavior follows hardware or stays with the code-defined corner

Only after those pass should PID tuning resume.
