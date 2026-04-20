# ESP32 Brushed Quadcopter

A full flight controller, a full RC radio link, and a complete ground telemetry system — built from two $5 ESP32 chips, with no flight-controller firmware, no RC receiver, no ESCs, and no external libraries beyond the ESP32 Arduino core.

Total electronics cost: **under $30**. Total code: **two `.cpp` files**. Madgwick AHRS, PID, mixer, radio protocol, all written from scratch and readable in an afternoon.

## Why This Project Is Unusual

A typical hobby quad stack is a $200+ pile of black boxes: a dedicated flight controller running Betaflight (half a million lines of C), an ELRS / Crossfire RC transmitter and receiver, four brushless ESCs running BLHeli, an OSD chip, an optional telemetry radio. You flash firmware, tune PIDs in a GUI, and learn nothing about what's actually happening inside.

This project replaces all of it with two generic WiFi microcontrollers:

| Part of the stack | Typical approach | This project |
|---|---|---|
| Flight controller | F4/F7 board + Betaflight | $5 ESP32, 1600 lines of `.cpp` |
| RC transmitter | $50–200 TX module | $5 ESP32 + 2 joysticks |
| RC protocol | ELRS / CRSF / SBUS | ESP-NOW (WiFi broadcast) |
| Receiver | Separate radio module | Same ESP32 as the FC |
| ESCs | 4x brushless ESCs, $15–30 | 2x TB6612FNG H-bridges, $2 each |
| Sensor fusion | Library / DMP | Inline Madgwick 6DOF (100 lines) |
| Telemetry | Extra hardware or none | Bidirectional over the same ESP-NOW link |

Because ESP-NOW is built into the ESP32's WiFi radio, the entire control **and** telemetry link is a software feature of the processor that was already there for IoT. Sub-millisecond latency, ~200 m line-of-sight range, no pairing ceremony, and you can OTA-reflash both ends over the same radio.

The full story (including why the educational value matters and what this unlocks): see `05_Ideas/esp32-drone-novelty-2026-04-20.md` in the Obsidian vault.

## Repository Layout

- `TX_Final.cpp` — controller / transmitter firmware
- `RX_Final.cpp` — drone / receiver firmware (the flight controller)
- `MadgwickAHRS.c` — reference implementation kept for comparison (not compiled into the flight controller — the drone uses its own inline 6DOF version)
- `CLAUDE.md` — implementation notes and “do not touch” list
- `TESTING_PLAN.md` — step-by-step PID tuning procedure

Each `.cpp` file is a standalone Arduino sketch. There is no multi-file build. Flash `TX_Final.cpp` to the controller ESP32, flash `RX_Final.cpp` to the drone ESP32.

## Hardware

### Bill of Materials (approx.)

| Part | Qty | Cost |
|---|---|---|
| ESP32 DevKit V1 | 2 | ~$10 |
| GY-87 (MPU6050 6-DOF IMU) | 1 | ~$3 |
| TB6612FNG H-bridge breakout | 2 | ~$4 |
| Brushed coreless motor (720/820) | 4 | ~$4 |
| 1S LiPo (350–500 mAh) | 1 | ~$5 |
| 2x analog thumb joystick with switch | — | ~$3 |
| Frame (3D-printed or foam) + props | — | ~$2 |
| **Total** | | **~$31** |

### Drone wiring

- I2C: **SDA = GPIO22, SCL = GPIO21** (swapped vs. ESP32 default; matches this build)
- Motors via TB6612FNG:
  - M1 → **GPIO25** (front-left)
  - M2 → **GPIO26** (front-right)
  - M3 → **GPIO27** (back-right)
  - M4 → **GPIO14** (back-left)
- TB6612 STBY (both boards tied together) → **GPIO33**, with a 10 kΩ pull-down to ground so motors stay disabled during boot

### Controller wiring

All joystick axes must be on ADC1 pins — ADC2 is unusable while WiFi is active on ESP32.

| Input | GPIO |
|---|---|
| Throttle axis | 33 |
| Yaw axis | 32 |
| Pitch axis | 35 |
| Roll axis | 34 |
| Left stick switch (L_SW) | 26 |
| Right stick switch (R_SW) | 27 |
| Status LED (optional) | 2 |

## Radio Link — ESP-NOW

The drone and controller are paired by hard-coding each other’s MAC addresses.

- In `TX_Final.cpp`: `receiverMac[]` = **drone** MAC
- In `RX_Final.cpp`: `CONTROLLER_MAC[]` = **controller** MAC

First-time pairing:

1. Flash both boards once with placeholder MACs.
2. Open Serial Monitor on each at 115200 baud; the boards print their own MAC at boot.
3. Paste the drone MAC into `TX_Final.cpp` and the controller MAC into `RX_Final.cpp`.
4. Reflash.

### Packet structures

Both structs are `__attribute__((packed))` and must match byte-for-byte across both files.

- `ControlPacket` (15 bytes): `t_ms`, `armed`, `calibrate_level`, `thr (0..1000)`, `yaw/pitch/roll (-500..500)`
- `TelemetryPacket` (30 bytes): IMU angles, PID outputs, all four motor duties, accel magnitude, arming/standby state, packet age

Control packets are sent at 50 Hz; telemetry at 20 Hz. The drone disarms automatically if no valid control packet arrives for **250 ms** (failsafe).

## Firmware Architecture

### Drone main loop — 500 Hz

```
getCommandsFromEspNow()   // pull latest control packet, apply failsafe
  → getIMUdata()          // raw I2C, bias correct, LPF
  → Madgwick6DOF()        // quaternion fusion with accel-trust gating
  → getDesState()         // sticks → desired roll/pitch angles + yaw rate
  → controlANGLE()        // PID: angle mode for roll/pitch, rate mode for yaw
  → mixerQuad()           // quad-X mixer with per-motor spin sign
  → scaleToDuty()         // 0..1 → 0..255 PWM duty
  → throttleCutAndCommand()  // arm/failsafe check, write PWM
  → sendTelemetry()
```

### Control strategy

- **Roll / Pitch**: angle-mode PID. Desired state is an absolute angle in degrees. Derivative-on-measurement (uses raw gyro, not error derivative) — avoids derivative kick on setpoint changes.
- **Yaw**: rate-mode PID. Desired state is a rotation rate in deg/s. This is standard for quads because yaw has no gravity reference and you want to allow continuous spins.
- **Anti-windup**: integrators reset when throttle < 1060 (drone grounded).
- **Adaptive Madgwick beta**: normally 0.06 while armed (vibration-tolerant). When the estimator drifts >5° from the accel-only attitude and the accel reads ~1 g, beta boosts to 0.20 for fast recovery, then ramps back down.

### Mixer

```cpp
m1 = T - P + R + (M1_SPIN * Y);   // front-left
m2 = T - P - R + (M2_SPIN * Y);   // front-right
m3 = T + P - R + (M3_SPIN * Y);   // back-right
m4 = T + P + R + (M4_SPIN * Y);   // back-left
```

`M1_SPIN..M4_SPIN` are compile-time constants (`MOTOR_SPIN_CW = -1`, `MOTOR_SPIN_CCW = +1`). The default configuration is:

- M1 (FL), M3 (BR) = **CW** viewed from above
- M2 (FR), M4 (BL) = **CCW** viewed from above

If your physical motors spin the other way, **change only these four constants** — do not rewrite the mixer. Yaw authority and sign correctness follow automatically.

## Calibration

### Startup calibration (every boot)

Drone sits still for ~3 seconds after power-on. The firmware averages 800 IMU samples to compute gyro bias and accel bias along the measured gravity direction. It does **not** assume the frame is perfectly level — it only pulls the resting accel magnitude toward 1.00 g.

Keep the drone still on a flat surface during the boot LED-blink sequence.

### Level-trim gesture (in-flight re-calibration)

While **disarmed**, hold this stick combination for 3 seconds:

- Left stick: bottom-right (throttle low + yaw right)
- Right stick: bottom-left (pitch low + roll left)

The controller sets `calibrate_level = 1` in the outgoing packet. The drone runs a still-IMU calibration and stores the current pose as “this is level” — useful when the drone develops a small steady lean.

### Stick-center calibration

On controller boot, the ESP32 samples the joystick analog centers. Keep both sticks centered while the controller boots.

## Diagnostic Modes

Compile-time flags at the top of `RX_Final.cpp`:

- `MOTOR_ISOLATION_MODE = 1` — bypasses stabilization and drives **one motor at a time** by stick position. THROTTLE stick → M1, YAW → M2, ROLL → M3, PITCH → M4. Use this to verify which GPIO goes to which physical corner, and to confirm each motor’s spin direction.
- `ALL_MOTORS_SAME_THRUST_MODE = 1` — drives all four motors at the throttle value, no stabilization. Use this to confirm that the frame lifts straight up (any lean isolates a thrust or CG issue, not a PID issue).
- `PRINT_TEL_ON_USB = 1` — detailed USB serial telemetry at 20 Hz for debugging PID/IMU behavior.

**Use diagnostic modes with props off.**

## Safety

- Pull-down resistor on `STBY_PIN` (10 kΩ) so the TB6612 drivers stay disabled during ESP32 boot.
- Failsafe: 250 ms without a valid control packet → auto-disarm + throttle cut.
- Arming requires throttle already low (< 1100).
- Disarm cuts `STBY` (hard motor disable) and zeroes all PWM outputs.
- Props off for first flash, first wiring check, first MAC pairing, and every diagnostic mode.
- Verify motor spin directions match `M1_SPIN..M4_SPIN` **before** enabling yaw. A mismatch turns the yaw loop into positive feedback and the drone will shoot to a corner.

## Build & Flash

- Arduino IDE with ESP32 board support, or PlatformIO with an ESP32 target
- Board: **ESP32 Dev Module**
- Baud: **115200**
- Dependencies: none beyond the ESP32 Arduino core (`Arduino.h`, `Wire.h`, `WiFi.h`, `esp_now.h`)

## Tuning

Current gains (in `RX_Final.cpp`):

- Roll / Pitch: `Kp=0.33`, `Ki=0.18`, `Kd=0.06`
- Yaw: `Kp=0.005`, `Ki=0`, `Kd=0`, output clamped to `±0.08`

Follow `TESTING_PLAN.md` for the step-by-step bring-up procedure (props-off checks → equal-thrust lift test → low hover → increasing Kp until oscillation → adding Ki → adding Kd).

## Known Assumptions

- PWM: 20 kHz, 8-bit resolution
- Control loop: 500 Hz target
- No altitude hold — pure manual throttle
- No magnetometer — yaw is rate-controlled, not heading-held
- Yaw authority depends on `M1_SPIN..M4_SPIN` matching physical hardware

## Next Improvements

- Altitude hold using the BMP180 that’s already on the GY-87 board (currently unused)
- Position hold with a PMW3901 optical flow sensor (~$5)
- Web-based PID tuning GUI served from the ESP32’s WiFi AP
- Multi-drone ESP-NOW swarm — one controller, many drones
- Automated PID tuning (relay method or Ziegler-Nichols)
