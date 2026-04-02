/*
ESP32 Brushed Flight Controller (ESP‑NOW RX)
-------------------------------------------
This is the same “starter port” flight controller sketch I gave you earlier
(Madgwick 6DOF + angle PID + quad mixer + TB6612FNG brushed PWM), BUT:

✅ It receives commands over ESP‑NOW using your ControlPacket
   instead of reading PWM receiver channels.

Controller packet (must match your transmitter code exactly):
  uint32_t t_ms;
  uint8_t  armed;
  int16_t  thr;   // 0..1000
  int16_t  yaw;   // -500..500
  int16_t  pitch; // -500..500
  int16_t  roll;  // -500..500

WIRING (matches your project summary):
  I2C: SDA=21, SCL=22
  Motors PWM: M1=25, M2=26, M3=27, M4=14
  TB6612 STBY: 33 (tie both boards' STBY together)

SAFETY: 
  - PROPS OFF for first tests.
  - Keep a 10k pull‑down on STBY so motors can’t spin at boot.

NOTES:
  - Yaw only works if 2 motors are reversed (swap motor wires).
  - Includes a simple failsafe: if no packets for FAILSAFE_MS -> disarm + throttle=0.
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>

// ========================= DIAGNOSTIC MODES =========================
// Set to 1 to replace normal stabilization/mixing with a single-motor test.
// This helps you identify which motor/prop causes the most vibration/noise.
//
// Safety tips:
// - PROPS OFF for initial tests.
// - Keep your hand off the frame while a motor is running.
// - Use small throttle first; only go higher if needed.
//
// Motor test mapping (one motor at a time):
// - THROTTLE full (and other sticks centered) -> Motor 1
// - YAW stick deflection -> Motor 2
// - ROLL stick deflection -> Motor 3
// - PITCH stick deflection -> Motor 4
#define MOTOR_ISOLATION_MODE 0

// Set to 1 to bypass stabilization and send the same thrust command to all
// four motors. This is useful when you want to compare motor sound/speed
// without roll/pitch/yaw corrections changing each motor differently.
//
// Important:
// - Use this only for diagnostics, not for real stabilized flight.
// - Keep MOTOR_ISOLATION_MODE at 0 when this mode is enabled.
#define ALL_MOTORS_SAME_THRUST_MODE 0

// Set to 1 to print a detailed TEL line over USB Serial (helps debugging).
// For actual flight you may set this to 0 to reduce timing jitter.
#define PRINT_TEL_ON_USB 1


// ========================= USER SETTINGS =========================
// This section contains all tunable parameters for the flight controller.
// Most of these should be adjusted based on your specific drone hardware.

// Loop rate: Target frequency for main control loop (Hz)
// WiFi/ESP‑NOW can cause jitter, so 500 Hz is a good starting point.
// Higher = more responsive but more CPU usage. Lower = more stable but sluggish.
static const int LOOP_HZ = 500;

// PWM frequency and resolution for motor control
// PWM_FREQ: 20kHz is standard for brushed DC motor control (inaudible, efficient)
// PWM_RES: 8-bit = 256 possible duty cycle values (0-255)
static const int PWM_FREQ = 20000;
static const int PWM_RES  = 8;     // Resolution: 0..255

// Motor pin assignments (ESP32 GPIO pins connected to TB6612FNG motor driver)
static const int M1_PIN = 25; // Front-Left motor PWM
static const int M2_PIN = 26; // Front-Right motor PWM
static const int M3_PIN = 27; // Back-Right motor PWM
static const int M4_PIN = 14; // Back-Left motor PWM
// STBY_PIN: Standby pin for motor driver (LOW=disabled, HIGH=enabled)
// Tie both TB6612FNG boards' STBY pins together to this pin
static const int STBY_PIN = 33;


// I2C pin assignment (set to match YOUR wiring)
static const int I2C_SDA_PIN = 22;
static const int I2C_SCL_PIN = 21;

// Failsafe mechanism: if no radio packets received for this many milliseconds,
// automatically disarm and set throttle to 0 for safety
static const uint32_t FAILSAFE_MS = 250;

// Control limits: maximum angles the drone will try to achieve
// These prevent over-aggressive movements and loss of control
float i_limit   = 25.0f;        // Anti-windup limit for PID integrators
float maxRoll   = 30.0f;        // Max roll angle drone will try to achieve (degrees)
float maxPitch  = 30.0f;        // Max pitch angle drone will try to achieve (degrees)
float maxYaw    = 160.0f;       // Max yaw rotation rate (degrees/second)

// ============= Angle PID Controller Gains (Roll & Pitch) =============
// These PID gains control how aggressively the drone corrects roll and pitch angles.
// Kp (Proportional): Responds to current angle error. Higher = more aggressively correct.
// Ki (Integral): Accumulates error over time to eliminate steady-state error.
// Kd (Derivative): Dampens response by considering rate of change. Higher = smoother but slower.
float Kp_roll_angle  = 0.2f;    // dRehmFlight default (was 0.05 — 4x too low)
float Ki_roll_angle  = 0.3f;    // dRehmFlight default (eliminates steady-state lean)
float Kd_roll_angle  = 0.0f;    // Keep at 0 until hover confirmed, then try 0.05
float Kp_pitch_angle = 0.2f;    // dRehmFlight default
float Ki_pitch_angle = 0.3f;    // dRehmFlight default
float Kd_pitch_angle = 0.0f;    // Keep at 0 until hover confirmed, then try 0.05

// ============= Yaw Rate PID Controller Gains =============
// Unlike roll/pitch (which control angles), yaw is controlled via rotation rate (degrees/sec).
// This is because yaw doesn't have a natural "neutral" gravity reference like pitch/roll do.
float Kp_yaw = 0.3f;            // dRehmFlight default (was 0.10 — 3x too low)
float Ki_yaw = 0.05f;           // dRehmFlight default (corrects yaw drift)
float Kd_yaw = 0.0f;            // Keep at 0 for brushed motors (risk of overheating)

// ============= Filter Constants (Low-Pass Filters) =============
// These parameters (0.0 to 1.0) control the weight of the NEW sensor sample in
// the exponential low-pass filter below.
// Higher values = LESS filtering (faster, noisier). Lower values = MORE
// filtering (smoother, but with more lag).
float B_madgwick_armed    = 0.02f; // Flight mode: keep accel correction conservative under vibration
float B_madgwick_disarmed = 0.50f; // Used during still-calibration settle windows
float B_accel    = 0.12f;       // More accel smoothing for vibration-heavy tests
float B_gyro     = 0.10f;       // More gyro smoothing to calm high-frequency jitter

// Adaptive Madgwick recovery: temporarily boosts beta when a large attitude
// error is detected and the accel reading is trustworthy (|a| ≈ 1g).
// Gives fast recovery after disturbances without hurting vibration rejection.
float B_madgwick_recovery  = 0.20f;   // Beta used during recovery (high)
float recovery_threshold_deg = 5.0f;  // Error above this triggers recovery
float recovery_ramp_rate   = 0.01f;   // How fast beta ramps back to B_madgwick_armed

// Minimum motor duty cycle: helps weak/sticky motors start rotating.
// Set to 0 for first tests. Increase if motors won't spin at low throttle.
int MIN_DUTY_WHEN_RUNNING = 0;

// Accel magnitude (in g). Used for telemetry/debug (should be ~1.00g when still)
float accMag_g = 1.0f;
float accTrustWeight = 1.0f;   // 1.0 = accel fully trusted, 0.0 = fully rejected
float accRejectPct = 0.0f;     // Percent of accel correction currently rejected
float currentBeta = 0.02f;    // Adaptive beta, ramps between B_madgwick_armed and B_madgwick_recovery

// Accel trust gate used by the attitude estimator.
// If accel magnitude is far from 1 g, we reduce or reject accel correction so
// vibration and bad measurements do not steer the attitude estimate too much.
static const float ACC_TRUST_HARD_LOW_G  = 0.70f;
static const float ACC_TRUST_HARD_HIGH_G = 1.30f;
static const float ACC_TRUST_SOFT_ERR_G  = 0.25f;

// Still IMU calibration timing:
// - The same timing constants are used when the joystick gesture requests a
//   still IMU calibration.
// - Accel bias is corrected only along the measured gravity direction so we
//   pull the resting accel magnitude closer to 1.00 g without assuming the
//   frame is perfectly level.
static const int STARTUP_IMU_CAL_DISCARD_SAMPLES = 100;
static const int STARTUP_IMU_CAL_SAMPLES         = 800;
static const int STARTUP_IMU_CAL_DELAY_MS        = 2;
static const int LEVEL_CAPTURE_DISCARD_SAMPLES   = 30;
static const int LEVEL_CAPTURE_SAMPLES           = 80;

// ========================= ESP‑NOW SETTINGS-Controller   =========================
static uint8_t CONTROLLER_MAC[6] = {0x4C,0xC3,0x82,0xDA,0xF8,0x38};


// ========================= ESP‑NOW CONTROL PACKET =========================
// ESP-NOW is a wireless protocol built into ESP32 for low-latency, connectionless communication.
// The transmitter (controller) sends ControlPacket structs to this receiver at regular intervals.
// This packet format MUST match exactly between transmitter and receiver code.

// Control packet structure - packed tightly in memory for wireless transmission
// The "__attribute__((packed))" ensures no memory padding between fields
typedef struct __attribute__((packed)) {
  uint32_t t_ms;      // Timestamp (milliseconds) when packet was sent - helps detect stale packets
  uint8_t  armed;     // Arming status: 0=disarmed (safe), non-zero=armed (ready to fly)
  uint8_t  calibrate_level; // 1 while the controller requests "store current pose as level"
  int16_t  thr;       // Throttle command: 0 (min) to 1000 (max)
  int16_t  yaw;       // Yaw command: -500 (left) to +500 (right)
  int16_t  pitch;     // Pitch command: -500 (backward) to +500 (forward)
  int16_t  roll;      // Roll command: -500 (left) to +500 (right)
} ControlPacket;


typedef struct __attribute__((packed)) {
  uint32_t t_ms;          // drone time
  uint8_t  seq;           // increments each telemetry frame
  uint8_t  rx_ok;         // 1 if a control packet is considered valid
  uint16_t rx_age_ms;     // age of last control packet

  uint8_t  armed_cmd;     // from controller packet
  uint8_t  armedFly;      // flight controller state
  uint8_t  stby;          // actual STBY pin level (1=enabled)
  uint8_t  reserved0;     // accel rejection percent (0..100), kept in the spare slot

  int16_t  thr, yaw, pitch, roll;  // controller commands

  int16_t  roll_imu_cdeg;   // roll_IMU * 100
  int16_t  pitch_imu_cdeg;  // pitch_IMU * 100

  int16_t  roll_pid_milli;  // roll_PID * 1000
  int16_t  pitch_pid_milli; // pitch_PID * 1000
  int16_t  yaw_pid_milli;   // yaw_PID * 1000

  uint8_t  m1_duty, m2_duty, m3_duty, m4_duty;  // 0..255
  uint16_t acc_mg;   // accel magnitude in milli-g (1000 = 1.000g)
} TelemetryPacket;




// Thread-safe packet storage and synchronization
// Uses a spinlock to protect concurrent access (ISR can interrupt main loop)
static portMUX_TYPE packetMux = portMUX_INITIALIZER_UNLOCKED;
static ControlPacket lastPacket{};        // Most recent packet received from controller
static uint32_t lastPacketRxMs = 0;       // Timestamp of last packet reception
static bool havePacket = false;           // Flag: have we received at least one packet?

// ESP-NOW receive callback (runs in ISR context, very time-critical!)
// Called whenever an ESP-NOW packet arrives. Must be fast to avoid blocking other code.
static void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info;  // MAC address info available but we don't use it
  
  // Verify packet size matches expected ControlPacket structure
  if (len != (int)sizeof(ControlPacket)) return;
  
  // Thread-safe copy: acquire spinlock, update packet data, release spinlock
  portENTER_CRITICAL(&packetMux);
  memcpy(&lastPacket, data, sizeof(ControlPacket));
  lastPacketRxMs = millis();
  havePacket = true;
  portEXIT_CRITICAL(&packetMux);
}

// Safe getter: retrieve latest packet and calculate age (how old it is)
// Returns true if we've ever received a packet, false otherwise
// ageMs is set to the time since last packet (0xFFFFFFFFu if no packet ever received)
static bool getLatestPacket(ControlPacket &out, uint32_t &ageMs) {
  uint32_t rxMs;
  bool ok;
  
  // Thread-safe read: acquire spinlock, copy data, release spinlock
  portENTER_CRITICAL(&packetMux);
  out = lastPacket;
  rxMs = lastPacketRxMs;
  ok = havePacket;
  portEXIT_CRITICAL(&packetMux);
  
  // Calculate packet age (how long since we received it)
  uint32_t now = millis();
  ageMs = ok ? (now - rxMs) : 0xFFFFFFFFu;
  return ok;
}

// ========================= MPU6050 SENSOR INTERFACE =========================
// The MPU6050 is a 6-DOF IMU (6 Degrees of Freedom: 3-axis accel + 3-axis gyro)
// Communication: I2C (SDA=GPIO21, SCL=GPIO22) at 400kHz
// This section provides the low-level hardware interface to read sensor data.

// MPU6050 I2C Address and Register Addresses
static const uint8_t MPU_ADDR       = 0x68;            // Default I2C address (AD0 pin low)
static const uint8_t REG_WHOAMI     = 0x75;            // Device ID register (should read 0x68)
static const uint8_t REG_PWR_MGMT_1 = 0x6B;            // Power management register
static const uint8_t REG_SMPLRT_DIV = 0x19;            // Sample rate divider
static const uint8_t REG_CONFIG     = 0x1A;            // Configuration register (filter bandwidth)
static const uint8_t REG_GYRO_CFG   = 0x1B;            // Gyroscope configuration (range)
static const uint8_t REG_ACCEL_CFG  = 0x1C;            // Accelerometer configuration (range)
static const uint8_t REG_DATA       = 0x3B;            // Base address for sensor data (14 bytes)

// Sensor scaling factors: raw 16-bit values must be divided by these to get
// real physical units.
// Accel: ±2g range  -> 16384 LSB/g
// Gyro:  ±250°/s    -> 131 LSB/(°/s)
static const float GYRO_SCALE_FACTOR  = 131.0f;      // LSB per deg/sec
static const float ACCEL_SCALE_FACTOR = 16384.0f;    // LSB per g

// I2C helper function: Write one byte to MPU6050 register
static bool i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);     // Start I2C transaction with MPU6050
  Wire.write(reg);                  // Send register address
  Wire.write(val);                  // Send data byte
  return Wire.endTransmission() == 0; // Return true if successful (no error)
}

// I2C helper function: Read n bytes from MPU6050 starting at register reg
static bool i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n) {
  Wire.beginTransmission(addr);     // Start I2C transaction
  Wire.write(reg);                  // Send register address to read from
  if (Wire.endTransmission(false) != 0) return false;  // Send with repeated start (wait for more)
  
  // Request n bytes from the register address
  size_t got = Wire.requestFrom((int)addr, (int)n, (int)true);  // Send stop after reading
  if (got != n) return false;       // Verify we got the right number of bytes
  
  // Copy bytes into buffer
  for (size_t i=0;i<n;i++) buf[i] = Wire.read();
  return true;
}

// Initialize MPU6050: verify device is present, wake it up, configure sensitivity ranges
static bool mpuInit() {
  uint8_t who=0;
  
  // Verify device is present by reading WHO_AM_I register (should be 0x68)
  if (!i2cReadBytes(MPU_ADDR, REG_WHOAMI, &who, 1)) return false;
  if (who != 0x68) return false;
  
  // Wake up the device (default is sleep mode to save power)
  if (!i2cWriteByte(MPU_ADDR, REG_PWR_MGMT_1, 0x00)) return false;
  delay(50);  // Wait for power-up to complete
  
  // Configure sample rate: divider=0 means max sample rate (~1kHz for accel, ~8kHz for gyro)
  i2cWriteByte(MPU_ADDR, REG_SMPLRT_DIV, 0x00);
  
  // Filter config: 0x03 = ~44Hz bandwidth, which is a better starting point
  // when motor vibration is still dominating the accelerometer.
  i2cWriteByte(MPU_ADDR, REG_CONFIG, 0x03);
  
  // Gyroscope range: 0x00 = ±250°/s (most sensitive, good for drones)
  i2cWriteByte(MPU_ADDR, REG_GYRO_CFG, 0x00);
  
  // Accelerometer range: 0x00 = ±2g (most sensitive, matches ACCEL_SCALE_FACTOR = 16384)
  i2cWriteByte(MPU_ADDR, REG_ACCEL_CFG, 0x00);
  
  delay(10);
  return true;
}

// Read raw 16-bit sensor data from MPU6050
// Returns: accelerometer (ax, ay, az) and gyroscope (gx, gy, gz) in raw ADC counts
static bool mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t b[14];  // Buffer for all sensor data (14 bytes from register 0x3B)
  
  // MPU6050 data layout starting at 0x3B:
  // Bytes 0-1: AccX, 2-3: AccY, 4-5: AccZ, 6-7: Temp, 8-9: GyroX, 10-11: GyroY, 12-13: GyroZ
  if (!i2cReadBytes(MPU_ADDR, REG_DATA, b, 14)) return false;
  
  // Combine high and low bytes into signed 16-bit values (big-endian format)
  ax = (int16_t)((b[0]<<8)  | b[1]);
  ay = (int16_t)((b[2]<<8)  | b[3]);
  az = (int16_t)((b[4]<<8)  | b[5]);
  gx = (int16_t)((b[8]<<8)  | b[9]);
  gy = (int16_t)((b[10]<<8) | b[11]);
  gz = (int16_t)((b[12]<<8) | b[13]);
  return true;
}

// Forward declarations for helper functions that are defined later but are
// used by the joystick-driven calibration helpers below.
static void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float dt);
static void getIMUdata();

// ========================= STATE & CONTROL VARIABLES =========================
// This section declares all variables needed for flight control
// Organized by functional area: timing, channel inputs, sensors, attitude, control outputs

// ========== LOOP TIMING ==========
// Each iteration of the main loop computes dt, needed for PID integration
float dt = 0.0f;           // Time step (seconds) between current and previous iteration
uint32_t tPrev = 0;        // Timestamp of previous iteration (microseconds)

// ========== PSEUDO PWM CONTROL CHANNELS ==========
// Instead of RC receiver PWM inputs, we use wireless ESP-NOW packets
// Converted to standard PWM format (1000-2000) to reuse flight control logic
// Channel 0: Throttle (1000=off, 2000=full)
// Channel 1: Roll (1000=left, 1500=neutral, 2000=right)
// Channel 2: Pitch (1000=back, 1500=neutral, 2000=forward)
// Channel 3: Yaw (1000=left spin, 1500=neutral, 2000=right spin)
// Channel 4: Cut switch (1000=armed, 2000=disarmed/cut)
float channel_pwm[5] = {1000,1500,1500,1500,2000};  // Current values
const float channel_Failsafe[5] = {1000,1500,1500,1500,2000};  // Failsafe defaults

// ========== RAW IMU SENSOR DATA ==========
// Accelerometer and gyroscope sensor readings (after scaling and filtering)
float AccX=0, AccY=0, AccZ=0;      // Acceleration (g-units)
float GyroX=0, GyroY=0, GyroZ=0;   // Angular velocity (degrees/second)
// Previous values for exponential low-pass filtering
float AccX_prev=0, AccY_prev=0, AccZ_prev=0;
float GyroX_prev=0, GyroY_prev=0, GyroZ_prev=0;

// Sensor calibration offsets (zero them at rest to remove bias)
float AccErrorX = 0.0f, AccErrorY = 0.0f, AccErrorZ = 0.0f;
float GyroErrorX = 0.0f, GyroErrorY = 0.0f, GyroErrorZ = 0.0f;

// ========== ATTITUDE ESTIMATION (from Madgwick Sensor Fusion) ==========
// Estimated drone orientation in 3D space
float roll_IMU=0, pitch_IMU=0, yaw_IMU=0;    // Euler angles (degrees)
float q0=1, q1=0, q2=0, q3=0;               // Quaternion (w,x,y,z)
float levelRollTrimDeg = 0.0f;              // Stored roll trim for "this pose is level"
float levelPitchTrimDeg = 0.0f;             // Stored pitch trim for "this pose is level"
bool calibrateLevelCmd = false;             // Latest wireless request to store a level pose
bool prevCalibrateLevelCmd = false;         // Edge detector so one hold only calibrates once

// ========== DESIRED STATE FROM CONTROLLER ==========
// What the pilot commanded through wireless control
float thro_des=0;    // Desired throttle (0.0=off, 1.0=full power)
float roll_des=0;    // Desired roll angle (degrees)
float pitch_des=0;   // Desired pitch angle (degrees)
float yaw_des=0;     // Desired yaw rate (degrees/second)

// ========== PID CONTROLLER STATES ==========
// Roll angle controller
float error_roll=0;      // Desired - actual roll angle
float integral_roll=0;   // Sum of past errors
float derivative_roll=0; // Rate of change of error
float roll_PID=0;        // Output

// Pitch angle controller
float error_pitch=0;
float integral_pitch=0;
float derivative_pitch=0;
float pitch_PID=0;

// Yaw rate controller (controls rotation rate, not angle)
float error_yaw=0;
float error_yaw_prev=0;  // Previous error for derivative
float integral_yaw=0;
float derivative_yaw=0;
float yaw_PID=0;

// ========== MOTOR CONTROL OUTPUTS ==========
// After mixer combines throttle + PID → individual motor commands
float m1_cmd=0, m2_cmd=0, m4_cmd=0, m3_cmd=0;  // Normalized (0.0-1.0)
int m1_duty=0, m2_duty=0, m4_duty=0, m3_duty=0;  // PWM duty cycle (0-255)

// ========== ARMING STATE ==========
// Safety flag: motors only run when armed==true
bool armedFly = false;

// Measure gyro bias and correct the steady-state accel magnitude while the
// drone is sitting still on the surface you want to use as the reference.
static bool calibrateImuStill() {
  float sumAx = 0.0f, sumAy = 0.0f, sumAz = 0.0f;
  float sumGx = 0.0f, sumGy = 0.0f, sumGz = 0.0f;
  int validSamples = 0;

  for (int i = 0; i < STARTUP_IMU_CAL_DISCARD_SAMPLES + STARTUP_IMU_CAL_SAMPLES; ++i) {
    int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
    if (mpuReadRaw(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw)) {
      if (i >= STARTUP_IMU_CAL_DISCARD_SAMPLES) {
        sumAx += (float)axRaw / ACCEL_SCALE_FACTOR;
        sumAy += (float)ayRaw / ACCEL_SCALE_FACTOR;
        sumAz += (float)azRaw / ACCEL_SCALE_FACTOR;
        sumGx += (float)gxRaw / GYRO_SCALE_FACTOR;
        sumGy += (float)gyRaw / GYRO_SCALE_FACTOR;
        sumGz += (float)gzRaw / GYRO_SCALE_FACTOR;
        ++validSamples;
      }
    }
    delay(STARTUP_IMU_CAL_DELAY_MS);
  }

  if (validSamples < (STARTUP_IMU_CAL_SAMPLES / 2)) {
    return false;
  }

  const float invCount = 1.0f / (float)validSamples;
  const float avgAx = sumAx * invCount;
  const float avgAy = sumAy * invCount;
  const float avgAz = sumAz * invCount;
  const float avgGx = sumGx * invCount;
  const float avgGy = sumGy * invCount;
  const float avgGz = sumGz * invCount;
  const float avgAccNorm = sqrtf(avgAx*avgAx + avgAy*avgAy + avgAz*avgAz);

  // Gyro should read 0 deg/s while the drone is standing still.
  GyroErrorX = avgGx;
  GyroErrorY = avgGy;
  GyroErrorZ = avgGz;

  // Remove only the gravity-parallel accel error. This corrects the common
  // "resting accel magnitude is 1.04 g" problem without forcing the frame to
  // be perfectly flat during startup.
  if (avgAccNorm > 0.80f && avgAccNorm < 1.20f) {
    const float gravityBias = avgAccNorm - 1.0f;
    const float gravityDirX = avgAx / avgAccNorm;
    const float gravityDirY = avgAy / avgAccNorm;
    const float gravityDirZ = avgAz / avgAccNorm;

    AccErrorX = gravityBias * gravityDirX;
    AccErrorY = gravityBias * gravityDirY;
    AccErrorZ = gravityBias * gravityDirZ;
  } else {
    AccErrorX = 0.0f;
    AccErrorY = 0.0f;
    AccErrorZ = 0.0f;
  }

  // Seed the low-pass filters with the corrected startup reading so the first
  // few loop iterations do not begin with a large transient.
  AccX_prev = avgAx - AccErrorX;
  AccY_prev = avgAy - AccErrorY;
  AccZ_prev = avgAz - AccErrorZ;
  GyroX_prev = 0.0f;
  GyroY_prev = 0.0f;
  GyroZ_prev = 0.0f;
  accMag_g = sqrtf(AccX_prev*AccX_prev + AccY_prev*AccY_prev + AccZ_prev*AccZ_prev);

  Serial.printf(
    "Still IMU calibration: accNormRaw=%.3f g accNormCorrected=%.3f g | accBias=%.4f,%.4f,%.4f g | gyroBias=%.3f,%.3f,%.3f dps\n",
    avgAccNorm,
    accMag_g,
    AccErrorX, AccErrorY, AccErrorZ,
    GyroErrorX, GyroErrorY, GyroErrorZ
  );

  return true;
}

// ========================= MADGWICK SENSOR FUSION =========================
// Fast inverse square root helper for normalizing quaternions
static float invSqrt(float x) { return 1.0f / sqrtf(x); }

// Madgwick 6DOF Sensor Fusion Algorithm
// Combines accelerometer (stable but slow) + gyroscope (fast but drifts) into reliable attitude estimate
// Input: gx, gy, gz (rotation rates in deg/sec), ax, ay, az (acceleration in g), dt (time step)
// Output: Updates global q0-q3 (quaternion) and roll_IMU, pitch_IMU, yaw_IMU (Euler angles)
//
// How it works:
// 1. Dead-reckoning: integrate gyroscope rates to predict next orientation (fast)
// 2. Error correction: use accelerometer gravity vector to detect drift and apply correction (slow)
// 3. Beta blend: balance responsiveness (trust gyro) vs stability (trust accel)
static void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float recipNorm;
  float s0, s1, s2, s3;  // Error quaternion components
  float qDot1, qDot2, qDot3, qDot4;  // Quaternion derivatives (rate of change)
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Convert gyro from deg/s to rad/s (0.0174533 = π/180)
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  // Step 1: Dead-reckoning step - integrate gyro to update quaternion
  // These equations come from quaternion kinematics (how quaternion changes with rotation rates)
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);


  // ----- Accel reliability check (vibration gating) -----
  float accMag = sqrtf(ax*ax + ay*ay + az*az);   // in g

  // Weight: 1.0 when near 1g, fades to 0 as it deviates
  float accW = 1.0f;
  if (accMag < ACC_TRUST_HARD_LOW_G || accMag > ACC_TRUST_HARD_HIGH_G) {
    accW = 0.0f;  // ignore accel completely this cycle
  } else {
    // Soft fade: at +/-0.25g error -> weight becomes 0
    float err = fabsf(accMag - 1.0f);
    accW = 1.0f - (err / ACC_TRUST_SOFT_ERR_G);
    accW = constrain(accW, 0.0f, 1.0f);
  }

  // Expose the estimator's accel trust as easy-to-read telemetry.
  accTrustWeight = accW;
  accRejectPct = (1.0f - accW) * 100.0f;

  
  // Step 2: Error correction if accelerometer data is valid (not all zeros)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // Normalize accelerometer vector (convert to unit vector pointing "down")
    recipNorm = invSqrt(ax*ax + ay*ay + az*az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Pre-compute products of quaternion components (optimization to avoid repeated math)
    _2q0 = 2.0f*q0;
    _2q1 = 2.0f*q1;
    _2q2 = 2.0f*q2;
    _2q3 = 2.0f*q3;
    _4q0 = 4.0f*q0;
    _4q1 = 4.0f*q1;
    _4q2 = 4.0f*q2;
    _8q1 = 8.0f*q1;
    _8q2 = 8.0f*q2;
    q0q0 = q0*q0;
    q1q1 = q1*q1;
    q2q2 = q2*q2;
    q3q3 = q3*q3;

    // Compute error between estimated gravity direction and measured gravity
    // s0-s3 are the gradient components showing how to adjust quaternion to reduce error
    s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
    s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
    s2 = 4.0f*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
    s3 = 4.0f*q1q1*q3 - _2q1*ax + 4.0f*q2q2*q3 - _2q2*ay;

    // Normalize error quaternion
    recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply correction: subtract B (beta) * error_quaternion from rate of change
    // Higher B = stronger correction from accelerometer, more stable but slower response
    float betaBase;
    if (!armedFly) {
      betaBase = B_madgwick_disarmed;
      currentBeta = B_madgwick_armed;  // reset for next arm
    } else {
      // Adaptive recovery: compare Madgwick estimate against accel-only estimate.
      // ax, ay, az are already normalized at this point.
      float accelRoll  = atan2f(ay, az) * 57.29577951f;
      float accelPitch = -atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.29577951f;
      float errDeg = max(fabsf(roll_IMU - accelRoll), fabsf(pitch_IMU - accelPitch));

      if (accW > 0.8f && errDeg > recovery_threshold_deg) {
        currentBeta = B_madgwick_recovery;
      } else {
        currentBeta -= recovery_ramp_rate;
        if (currentBeta < B_madgwick_armed) currentBeta = B_madgwick_armed;
      }
      betaBase = currentBeta;
    }
    float betaEff = betaBase * accW;
    qDot1 -= betaEff*s0;
    qDot2 -= betaEff*s1;
    qDot3 -= betaEff*s2;
    qDot4 -= betaEff*s3;
  }

  // Step 3: Integrate to get new quaternion estimate
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  // Normalize quaternion (constraint: q0² + q1² + q2² + q3² = 1)
  recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  // Convert quaternion to Euler angles (roll, pitch, yaw) in degrees
  // These are more intuitive for drone control than quaternions
  // 57.29577951 = 180/π (convert radians to degrees)
  roll_IMU  = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * 57.29577951f;
  pitch_IMU = -asinf(constrain(-2.0f*(q1*q3 - q0*q2), -0.999999f, 0.999999f)) * 57.29577951f;
  yaw_IMU   = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3) * 57.29577951f;
}

// ------------------------- Core functions -------------------------
static void getIMUdata() {
  int16_t ax,ay,az,gx,gy,gz;
  if (!mpuReadRaw(ax,ay,az,gx,gy,gz)) return;

  AccX = (float)ax  / ACCEL_SCALE_FACTOR;
  AccY = (float)ay  / ACCEL_SCALE_FACTOR;
  AccZ = (float)az  / ACCEL_SCALE_FACTOR;
  GyroX = (float)gx / GYRO_SCALE_FACTOR;
  GyroY = (float)gy / GYRO_SCALE_FACTOR;
  GyroZ = (float)gz / GYRO_SCALE_FACTOR;

  AccX -= AccErrorX; AccY -= AccErrorY; AccZ -= AccErrorZ;
  GyroX -= GyroErrorX; GyroY -= GyroErrorY; GyroZ -= GyroErrorZ;

  AccX = (1.0f - B_accel)*AccX_prev + B_accel*AccX;
  AccY = (1.0f - B_accel)*AccY_prev + B_accel*AccY;
  AccZ = (1.0f - B_accel)*AccZ_prev + B_accel*AccZ;
  AccX_prev = AccX; AccY_prev = AccY; AccZ_prev = AccZ;

  GyroX = (1.0f - B_gyro)*GyroX_prev + B_gyro*GyroX;
  GyroY = (1.0f - B_gyro)*GyroY_prev + B_gyro*GyroY;
  GyroZ = (1.0f - B_gyro)*GyroZ_prev + B_gyro*GyroZ;
  GyroX_prev = GyroX; GyroY_prev = GyroY; GyroZ_prev = GyroZ;
  // Compute accel magnitude (g). Good indicator of vibration/noise.
  accMag_g = sqrtf(AccX*AccX + AccY*AccY + AccZ*AccZ);

}

static void getCommandsFromEspNow() {
  ControlPacket p{};
  uint32_t age;
  bool ok = getLatestPacket(p, age);

  if (!ok || age > FAILSAFE_MS) {
    // failsafe
    for (int i=0;i<5;i++) channel_pwm[i] = channel_Failsafe[i];
    armedFly = false;
    calibrateLevelCmd = false;
    return;
  }

  // Clamp packet values (defensive)
  int16_t thr   = constrain((int)p.thr,   0, 1000);
  int16_t roll  = constrain((int)p.roll,  -500, 500);
  int16_t pitch = constrain((int)p.pitch, -500, 500);
  int16_t yaw   = constrain((int)p.yaw,   -500, 500);

  // Convert packet -> pseudo PWM channels
  channel_pwm[0] = 1000.0f + (float)thr;     // throttle
  channel_pwm[1] = 1500.0f + (float)roll;    // roll
  channel_pwm[2] = 1500.0f + (float)pitch;   // pitch
  channel_pwm[3] = 1500.0f + (float)yaw;     // yaw

  // “Cut channel”: >1500 means CUT
  bool pktArmed = (p.armed != 0);
  channel_pwm[4] = pktArmed ? 1000.0f : 2000.0f;
  calibrateLevelCmd = (!pktArmed) && (p.calibrate_level != 0);

  // Final arming decision (latched):
  // - Disarm immediately if pktArmed=0
  // - To transition from DISARMED -> ARMED, require low-ish throttle
  // - Once armed, stay armed as long as packets keep arriving
  if (!pktArmed) {
    armedFly = false;
  } else if (!armedFly) {
    if (channel_pwm[0] < 1100.0f) armedFly = true;
  }
}

// Apply the stored trim so the selected flat pose reads as zero roll/pitch.
static void applyLevelTrimToAttitude() {
  roll_IMU -= levelRollTrimDeg;
  pitch_IMU -= levelPitchTrimDeg;
}

// Convert the current corrected accelerometer reading into a starting attitude
// estimate. Yaw is reset to zero because accel does not observe heading.
static void seedAttitudeFromCurrentAccel() {
  // Body-axis remap for this airframe/IMU mounting:
  // - Sensor Y points toward the drone nose
  // - Sensor X points toward the drone left side
  // Madgwick expects body X=forward and body Y=right, so we remap to:
  //   X_body =  AccY
  //   Y_body = -AccX
  float ax = -AccY;
  float ay =  AccX;
  float az =  AccZ;

  const float norm = sqrtf(ax*ax + ay*ay + az*az);
  if (norm < 0.5f) {
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    roll_IMU = 0.0f;
    pitch_IMU = 0.0f;
    yaw_IMU = 0.0f;
    return;
  }

  ax /= norm;
  ay /= norm;
  az /= norm;

  const float rollRad = atan2f(ay, az);
  const float pitchRad = -atan2f(-ax, sqrtf(ay*ay + az*az));
  const float yawRad = 0.0f;

  const float cr = cosf(rollRad * 0.5f);
  const float sr = sinf(rollRad * 0.5f);
  const float cp = cosf(pitchRad * 0.5f);
  const float sp = sinf(pitchRad * 0.5f);
  const float cy = cosf(yawRad * 0.5f);
  const float sy = sinf(yawRad * 0.5f);

  q0 = cr*cp*cy + sr*sp*sy;
  q1 = sr*cp*cy - cr*sp*sy;
  q2 = cr*sp*cy + sr*cp*sy;
  q3 = cr*cp*sy - sr*sp*cy;

  roll_IMU  = rollRad  * 57.29577951f;
  pitch_IMU = pitchRad * 57.29577951f;
  yaw_IMU   = 0.0f;
}

// Re-seed the quaternion from the current still pose and immediately re-apply
// the stored level trim. This avoids the "snaps back to zero slowly" feel
// after a fresh level calibration.
static void refreshAttitudeEstimateFromCurrentPose() {
  getIMUdata();
  seedAttitudeFromCurrentAccel();
  applyLevelTrimToAttitude();
}

// After the still IMU calibration, let the estimator settle for a short window
// and average the resulting roll/pitch. This makes the current flat pose become
// a cleaner software zero than a single instantaneous sample would.
static bool captureAverageLevelTrim(float &rollTrimOut, float &pitchTrimOut) {
  const float settleDt = max(STARTUP_IMU_CAL_DELAY_MS, 1) * 0.001f;
  float sumRoll = 0.0f;
  float sumPitch = 0.0f;
  int validSamples = 0;

  getIMUdata();
  seedAttitudeFromCurrentAccel();

  for (int i = 0; i < LEVEL_CAPTURE_DISCARD_SAMPLES + LEVEL_CAPTURE_SAMPLES; ++i) {
    delay(STARTUP_IMU_CAL_DELAY_MS);
    getIMUdata();
    Madgwick6DOF(-GyroY, GyroX, -GyroZ, -AccY, AccX, AccZ, settleDt);

    if (i >= LEVEL_CAPTURE_DISCARD_SAMPLES) {
      sumRoll += roll_IMU;
      sumPitch += pitch_IMU;
      ++validSamples;
    }
  }

  if (validSamples < (LEVEL_CAPTURE_SAMPLES / 2)) {
    return false;
  }

  rollTrimOut = sumRoll / (float)validSamples;
  pitchTrimOut = sumPitch / (float)validSamples;
  return true;
}

// Force the vehicle into a safe, disarmed state and clear controller memory so
// a calibration cannot leave stale motor or PID commands behind.
static void forceOutputsSafeAndResetControllers() {
  armedFly = false;
  channel_pwm[0] = 1000.0f;
  channel_pwm[1] = 1500.0f;
  channel_pwm[2] = 1500.0f;
  channel_pwm[3] = 1500.0f;
  channel_pwm[4] = 2000.0f;
  thro_des = 0.0f;
  roll_des = 0.0f;
  pitch_des = 0.0f;
  yaw_des = 0.0f;
  m1_cmd = m2_cmd = m3_cmd = m4_cmd = 0.0f;
  m1_duty = m2_duty = m3_duty = m4_duty = 0;
  error_roll = 0.0f;
  error_pitch = 0.0f;
  error_yaw = 0.0f;
  error_yaw_prev = 0.0f;
  integral_roll = 0.0f;
  integral_pitch = 0.0f;
  integral_yaw = 0.0f;
  derivative_roll = 0.0f;
  derivative_pitch = 0.0f;
  derivative_yaw = 0.0f;
  roll_PID = 0.0f;
  pitch_PID = 0.0f;
  yaw_PID = 0.0f;
  currentBeta = B_madgwick_armed;

  digitalWrite(STBY_PIN, LOW);
  ledcWrite(M1_PIN, 0);
  ledcWrite(M2_PIN, 0);
  ledcWrite(M3_PIN, 0);
  ledcWrite(M4_PIN, 0);
}

// Use the joystick gesture to run a still IMU calibration and then mark the
// current pose as the new level reference.
static void maybeApplyLevelCalibration() {
  const bool canCalibrateNow = calibrateLevelCmd && !armedFly && (channel_pwm[0] < 1050.0f);

  if (canCalibrateNow && !prevCalibrateLevelCmd) {
    forceOutputsSafeAndResetControllers();

#if PRINT_TEL_ON_USB
    Serial.println("Keep the drone still: running joystick IMU + level calibration...");
#endif

    if (calibrateImuStill()) {
      float measuredRollTrim = 0.0f;
      float measuredPitchTrim = 0.0f;
      const bool haveLevelTrim = captureAverageLevelTrim(measuredRollTrim, measuredPitchTrim);

      // Make the current physical pose become roll=0 / pitch=0 in software.
      if (haveLevelTrim) {
        levelRollTrimDeg = measuredRollTrim;
        levelPitchTrimDeg = measuredPitchTrim;
      } else {
        levelRollTrimDeg = roll_IMU;
        levelPitchTrimDeg = pitch_IMU;
      }
      refreshAttitudeEstimateFromCurrentPose();
      if (fabsf(roll_IMU) < 0.25f) roll_IMU = 0.0f;
      if (fabsf(pitch_IMU) < 0.25f) pitch_IMU = 0.0f;
      yaw_IMU = 0.0f;

#if PRINT_TEL_ON_USB
      Serial.printf(
        "Joystick IMU + level calibration applied: rollTrim=%.2f deg pitchTrim=%.2f deg acc|=%.3f g\n",
        levelRollTrimDeg,
        levelPitchTrimDeg,
        accMag_g
      );
#endif
    } else {
#if PRINT_TEL_ON_USB
      Serial.println("Joystick IMU calibration failed. Keeping previous offsets.");
#endif
    }
  }

  prevCalibrateLevelCmd = canCalibrateNow;
}

// ========================= DESIRED STATE CALCULATION =========================
// Convert wireless stick inputs → desired drone attitude angles
// Maps PWM-style inputs (1000-2000) to actual angle setpoints in degrees
static void getDesState() {
  // Normalize each channel to -1.0 to +1.0 range
  // PWM 1000 = -1.0, 1500 = 0.0, 2000 = +1.0
  float thr_norm  = (channel_pwm[0] - 1000.0f)/1000.0f;  // 0.0 to 1.0 (no negative throttle)
  float roll_norm = (channel_pwm[1] - 1500.0f)/500.0f;   // -1.0 to +1.0
  float pit_norm  = (channel_pwm[2] - 1500.0f)/500.0f;   // -1.0 to +1.0
  float yaw_norm  = (channel_pwm[3] - 1500.0f)/500.0f;   // -1.0 to +1.0

  // Apply limits and scale by maximum angle setpoints:
  // maxRoll, maxPitch (degrees), maxYaw (degrees/second)
  // constrain() ensures we never exceed the limits even with receiver noise
  thro_des  = constrain(thr_norm, 0.0f, 1.0f);
  roll_des  = constrain(roll_norm, -1.0f, 1.0f) * maxRoll;
  pitch_des = constrain(pit_norm,  -1.0f, 1.0f) * maxPitch;
  yaw_des   = constrain(yaw_norm,  -1.0f, 1.0f) * maxYaw;
}

// ========================= PID ATTITUDE CONTROL =========================
// Implements PID control for roll and pitch angles, plus yaw rotation rate
// Goal: make actual attitude match desired attitude by commanding motor adjustments
//
// PID (Proportional-Integral-Derivative) formula:
//   Output = Kp*error + Ki*integral(error) + Kd*derivative(error)
//
// Roll & Pitch: Angle control mode - maintains level flight
// Yaw: Rate control mode - allows continuous spinning with damping
//
static void controlANGLE() {
  // ========== ROLL CONTROL ==========
  // Keeps drone level (actual roll matches desired roll angle)
  
  // Proportional term: how much angle error exists right now?
  error_roll = roll_des - roll_IMU;
  
  // Integral term: accumulates error over time to eliminate steady-state drift
  integral_roll += error_roll * dt;
  
  // Anti-windup: reset integrator if throttle is very low (not flying)
  // Prevents integral from building up while drone sits on ground
  if (channel_pwm[0] < 1060) integral_roll = 0;
  
  // Limit integral to prevent it from growing unbounded and causing instability
  integral_roll = constrain(integral_roll, -i_limit, i_limit);
  
  // Roll axis is the drone's forward axis. With this IMU mounting, that maps
  // to the sensor Y gyro channel.
  derivative_roll = -GyroY;
  
  // Combine P+I+D terms with tuned gains, scale by 0.01f for motor command conversion
  roll_PID = 0.01f * (Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - Kd_roll_angle*derivative_roll);

  // ========== PITCH CONTROL ==========
  // Keeps drone level (actual pitch matches desired pitch angle)
  // Same structure as roll (two-axis stabilization)
  
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch += error_pitch * dt;
  if (channel_pwm[0] < 1060) integral_pitch = 0;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit);
  // Pitch axis is the drone's right axis. With this IMU mounting, that maps
  // to the negative sensor X gyro channel.
  derivative_pitch = -GyroX;
  pitch_PID = 0.01f * (Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch);

  // ========== YAW CONTROL ==========
  // Rate control (not angle control like roll/pitch)
  // Desired yaw is a rotation rate (deg/sec), not an absolute angle
  // This allows continuous 360° spins without angle accumulation issues
  
  // Error: difference between desired and actual rotation rate
  error_yaw = yaw_des - GyroZ;
  
  // Integral: accumulates to apply sustained torque for yaw
  integral_yaw += error_yaw * dt;
  if (channel_pwm[0] < 1060) integral_yaw = 0;
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit);
  
  // Derivative: how fast is the yaw error changing?
  // Provides damping to prevent oscillation/overshoot
  derivative_yaw = (error_yaw - error_yaw_prev) / max(dt, 1e-6f);
  
  // Yaw PID output
  yaw_PID = 0.01f * (Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw);
  
  // Save error for next iteration (needed for derivative calculation)
  error_yaw_prev = error_yaw;
}

// ========================= QUADCOPTER MOTOR MIXER =========================
// Combines throttle + PID control outputs → individual motor commands
// The mixer translates attitude corrections into differential motor speeds
//
// Quadcopter geometry (viewed from above):
//       M1(FL)    M2(FR)
//         |         |
//    -----|---------|-----
//    |                   |
//    -----------+---------
//    |                   |
//    -----|---------|-----
//         |         |
//       M4(BL)    M(BR)
//
// Motor layout: 1=Front-Left, 2=Front-Right, 3=Back-Right, 4=Back-Left
//
// Mixing logic for quad (X configuration):
//   - Throttle is added to all motors (more throttle = all faster)
//   - Pitch: tilt forward = slow front (M1,M2), fast back (M3,M4), or vice versa
//   - Roll: tilt right = slow left (M1,M4), fast right (M2,M3), or vice versa
//   - Yaw: rotate = opposite pairs spin opposite directions (M1,M3 vs M2,M4)
//
static void mixerQuad() {
  // Front-Left: Base throttle + pitch backward correction + roll right correction + yaw counter-spin
  m1_cmd = thro_des - pitch_PID + roll_PID + yaw_PID;
  
  // Front-Right: Base throttle + pitch backward correction - roll right correction - yaw counter-spin
  m2_cmd = thro_des - pitch_PID - roll_PID - yaw_PID;

  // Back-right:   
  m3_cmd = thro_des + pitch_PID - roll_PID + yaw_PID;
  
  // Back-left: 
  m4_cmd = thro_des + pitch_PID + roll_PID - yaw_PID;

  // Clamp motor commands: values outside 0.0-1.0 can't be represented in PWM
  // This prevents "saturation" from causing weird behavior or priority inversion
  m1_cmd = constrain(m1_cmd, 0.0f, 1.0f);
  m2_cmd = constrain(m2_cmd, 0.0f, 1.0f);
  m4_cmd = constrain(m4_cmd, 0.0f, 1.0f);
  m3_cmd = constrain(m3_cmd, 0.0f, 1.0f);
}

// ========================= SCALE MOTOR COMMANDS TO PWM DUTY CYCLE =========================
// Converts normalized motor commands (0.0-1.0) to PWM duty cycles (0-255)
// Also implements minimum duty feature to help weak/sticky motors start spinning
//
static void scaleToDuty() {
  // Convert normalized values (0.0-1.0) to 8-bit PWM duty cycle (0-255)
  // 0.0 → 0 (off), 0.5 → ~127, 1.0 → 255 (full power)
  m1_duty = (int)lroundf(m1_cmd * 255.0f);
  m2_duty = (int)lroundf(m2_cmd * 255.0f);
  m4_duty = (int)lroundf(m4_cmd * 255.0f);
  m3_duty = (int)lroundf(m3_cmd * 255.0f);

  // Optional: Minimum duty for brushed motors
  // Some cheap brushed motors won't spin at very low PWM (static friction).
  // If enabled, any motor that should be spinning gets a minimum boost.
  if (MIN_DUTY_WHEN_RUNNING > 0 && thro_des > 0.05f) {
    if (m1_duty>0 && m1_duty<MIN_DUTY_WHEN_RUNNING) m1_duty = MIN_DUTY_WHEN_RUNNING;
    if (m2_duty>0 && m2_duty<MIN_DUTY_WHEN_RUNNING) m2_duty = MIN_DUTY_WHEN_RUNNING;
    if (m4_duty>0 && m4_duty<MIN_DUTY_WHEN_RUNNING) m4_duty = MIN_DUTY_WHEN_RUNNING;
    if (m3_duty>0 && m3_duty<MIN_DUTY_WHEN_RUNNING) m3_duty = MIN_DUTY_WHEN_RUNNING;
  }

  // Final saturation: ensure duty cycle stays in valid 0-255 range
  m1_duty = constrain(m1_duty, 0, 255);
  m2_duty = constrain(m2_duty, 0, 255);
  m4_duty = constrain(m4_duty, 0, 255);
  m3_duty = constrain(m3_duty, 0, 255);
}

// ========================= FINAL SAFETY CHECK & MOTOR OUTPUT =========================
// Final safety check: implements "motor kill switch"
// Forces all motors off if disarmed or if cut channel is activated
//
static void throttleCutAndCommand() {
  // Determine if motors should be disabled:
  // 1. Cut channel > 1500 (pilot activated kill switch)
  // 2. Not armed (safety: no motor spin when armedFly=false)
  const bool cut = (channel_pwm[4] > 1500) || (!armedFly);
  
  if (cut) {
    // MOTOR KILL: all motors off
    armedFly = false;  // Force disarm state
    digitalWrite(STBY_PIN, LOW);  // Disable TB6612FNG motor driver
    ledcWrite(M1_PIN, 0);  // Set all PWM to 0
    ledcWrite(M2_PIN, 0);
    ledcWrite(M4_PIN, 0);
    ledcWrite(M3_PIN, 0);
    return;
  }
  
  // Motors enabled: activate driver and write commands
  digitalWrite(STBY_PIN, HIGH);  // Enable TB6612FNG motor driver
  ledcWrite(M1_PIN, m1_duty);
  ledcWrite(M2_PIN, m2_duty);
  ledcWrite(M4_PIN, m4_duty);
  ledcWrite(M3_PIN, m3_duty);
}

// ========================= LOOP RATE CONTROL =========================
// Maintains consistent control loop frequency by busy-waiting
// Important: flight controller must run at consistent rate for PID timing to work
//
// How it works:
// 1. Calculate target period: period_us = 1,000,000 / hz
// 2. On first call, record current time as tNext
// 3. Busy-wait (spin doing nothing) until tNext is reached
// 4. Move tNext forward by one period for next iteration
// 5. If loop is too fast, this waits. If loop is too slow, we skip a frame.
//
static void loopRate(int hz) {
  static uint32_t tNext = 0;  // Next wakeup time (only initialized once)
  if (tNext == 0) tNext = micros();  // Initialize on first call
  
  // Calculate period in microseconds: U = 1,000,000 / frequency
  uint32_t period = 1000000UL / (uint32_t)hz;
  
  // Busy-wait until tNext time is reached
  // This keeps consistent loop frequency even with variable code execution time
  while ((int32_t)(micros() - tNext) < 0) { }
  
  // Schedule next execution time
  tNext += period;
}


// ========================= TELEMETRY SENDING =========================
static uint32_t lastTelTxMs = 0;
static uint8_t  telSeq = 0;

static void sendTelemetry() {
  if (millis() - lastTelTxMs < 50) return; // 20 Hz
  lastTelTxMs = millis();

  ControlPacket p; 
  uint32_t age = 0;
  bool ok = getLatestPacket(p, age);

  TelemetryPacket t{};
  t.t_ms = millis();
  t.seq  = telSeq++;
  t.rx_ok = ok ? 1 : 0;
  t.rx_age_ms = (uint16_t)min(age, (uint32_t)65535);

  t.armed_cmd = (uint8_t)p.armed;
  t.armedFly  = armedFly ? 1 : 0;
  t.stby      = digitalRead(STBY_PIN) ? 1 : 0;
  t.reserved0 = (uint8_t)constrain((int)lroundf(accRejectPct), 0, 100);

  t.thr = p.thr; t.yaw = p.yaw; t.pitch = p.pitch; t.roll = p.roll;

  t.roll_imu_cdeg  = (int16_t)lroundf(roll_IMU  * 100.0f);
  t.pitch_imu_cdeg = (int16_t)lroundf(pitch_IMU * 100.0f);

  t.roll_pid_milli  = (int16_t)lroundf(roll_PID  * 1000.0f);
  t.pitch_pid_milli = (int16_t)lroundf(pitch_PID * 1000.0f);
  t.yaw_pid_milli   = (int16_t)lroundf(yaw_PID   * 1000.0f);

  t.m1_duty = (uint8_t)constrain(m1_duty, 0, 255);
  t.m2_duty = (uint8_t)constrain(m2_duty, 0, 255);
  t.m4_duty = (uint8_t)constrain(m4_duty, 0, 255);
  t.m3_duty = (uint8_t)constrain(m3_duty, 0, 255);

  t.acc_mg = (uint16_t)constrain((int)(accMag_g * 1000.0f), 0, 4000);


  esp_now_send(CONTROLLER_MAC, (uint8_t*)&t, sizeof(t));

#if PRINT_TEL_ON_USB
// Match the familiar debug format:
// <millis> -> TEL age=<..>ms ok=<..> seq=<..> | armedCmd=.. armedFly=.. STBY=.. | thr=.. yaw=.. pitch=.. roll=.. | IMU r=.. p=.. | PID r=.. p=.. y=.. | duty ... acc|=.. g aRej=..%
static uint32_t lastTelPrintMs = 0;
if (millis() - lastTelPrintMs >= 50) { // ~20 Hz
  lastTelPrintMs = millis();
  Serial.printf(
    "%lu -> TEL age=%lums ok=%d seq=%u | armedCmd=%d armedFly=%d STBY=%d | thr=%d yaw=%d pitch=%d roll=%d | IMU r=%.2f p=%.2f | PID r=%.3f p=%.3f y=%.3f | duty %3d %3d %3d %3d acc|=%.3f g aRej=%.1f%%\n",
    (unsigned long)millis(),
    (unsigned long)age,
    ok ? 1 : 0,
    (unsigned int)t.seq,
    (int)t.armed_cmd,
    (int)t.armedFly,
    (int)t.stby,
    (int)t.thr, (int)t.yaw, (int)t.pitch, (int)t.roll,
    roll_IMU, pitch_IMU,
    roll_PID, pitch_PID, yaw_PID,
    (int)constrain(m1_duty, 0, 255),
    (int)constrain(m2_duty, 0, 255),
    (int)constrain(m3_duty, 0, 255),
    (int)constrain(m4_duty, 0, 255),
    accMag_g,
    accRejectPct
  );
}
#endif
}




void calculate_IMU_error() {
  // Keep the original function name, but use the current IMU calibration path
  // that already matches this codebase's MPU6050 access, filtering, and units.
  // The drone must be sitting still on the surface you want to use as the
  // startup reference.
  forceOutputsSafeAndResetControllers();
  (void)calibrateImuStill();
}

void calibrateAttitude() {
  // Keep the original function name, but use the current attitude-seeding and
  // averaged level-trim capture so setup calibration and joystick calibration
  // both zero pitch/roll the same way.
  forceOutputsSafeAndResetControllers();

  float measuredRollTrim = 0.0f;
  float measuredPitchTrim = 0.0f;
  const bool haveLevelTrim = captureAverageLevelTrim(measuredRollTrim, measuredPitchTrim);

  if (haveLevelTrim) {
    levelRollTrimDeg = measuredRollTrim;
    levelPitchTrimDeg = measuredPitchTrim;
  } else {
    getIMUdata();
    seedAttitudeFromCurrentAccel();
    levelRollTrimDeg = roll_IMU;
    levelPitchTrimDeg = pitch_IMU;
  }

  refreshAttitudeEstimateFromCurrentPose();
  if (fabsf(roll_IMU) < 0.25f) roll_IMU = 0.0f;
  if (fabsf(pitch_IMU) < 0.25f) pitch_IMU = 0.0f;
  yaw_IMU = 0.0f;
}



// ========================= MOTOR ISOLATION TEST =========================
// This mode bypasses stabilization and drives ONE motor at a time based on stick inputs.
// It lets you watch accMag_g and identify which motor/prop introduces the most vibration.
//
// Selection priority:
// 1) If throttle is near full AND other axes are near center -> test Motor 1
// 2) Else whichever axis (yaw/roll/pitch) has the largest deflection selects Motor 2/3/4.
//
// Note: We still apply the same arming + CUT logic via throttleCutAndCommand().
static void motorIsolationTest() {
  // Convert channels to normalized commands
  const float thr_norm  = constrain((channel_pwm[0] - 1000.0f)/1000.0f, 0.0f, 1.0f);
  const float roll_norm = constrain((channel_pwm[1] - 1500.0f)/500.0f, -1.0f, 1.0f);
  const float pit_norm  = constrain((channel_pwm[2] - 1500.0f)/500.0f, -1.0f, 1.0f);
  const float yaw_norm  = constrain((channel_pwm[3] - 1500.0f)/500.0f, -1.0f, 1.0f);

  const float aRoll = fabsf(roll_norm);
  const float aPit  = fabsf(pit_norm);
  const float aYaw  = fabsf(yaw_norm);

  // Deadband to avoid accidental selection from tiny stick noise
  const float DB = 0.12f;   // ~60/500

  // Optional safety limit (set to 255 if you truly need full power in tests)
  const int TEST_MAX_DUTY = 200;

  // Map 0..1 -> 0..TEST_MAX_DUTY
  auto map01ToDuty = [&](float x)->int {
    x = constrain(x, 0.0f, 1.0f);
    int d = (int)lroundf(x * (float)TEST_MAX_DUTY);
    return constrain(d, 0, TEST_MAX_DUTY);
  };

  // Start all motors OFF
  m1_duty = m2_duty = m3_duty = m4_duty = 0;

  // Decide which motor to run
  int which = 0; // 0=none, 1..4 = motor number

  const bool otherCentered = (aRoll < DB && aPit < DB && aYaw < DB);

  // Motor 1 test when throttle is (almost) full and other sticks centered
  if (thr_norm > 0.95f && otherCentered) {
    which = 1;
    m1_duty = map01ToDuty(thr_norm);   // near full -> near TEST_MAX_DUTY
  } 
  else {
    // Axis-driven selection (choose largest deflection above DB)
    float best = 0.0f;

    if (aYaw > DB && aYaw >= best) { best = aYaw; which = 2; }
    if (aRoll > DB && aRoll >  best) { best = aRoll; which = 3; }
    if (aPit > DB && aPit >  best) { best = aPit;  which = 4; }

    if (which == 2) m2_duty = map01ToDuty(aYaw);
    if (which == 3) m3_duty = map01ToDuty(aRoll);
    if (which == 4) m4_duty = map01ToDuty(aPit);
  }

  // Optional minimum duty to ensure motors actually spin (only during tests)
  const int TEST_MIN_DUTY = 0; // set to e.g. 30-60 if a motor won't start
  if (TEST_MIN_DUTY > 0) {
    if (m1_duty > 0 && m1_duty < TEST_MIN_DUTY) m1_duty = TEST_MIN_DUTY;
    if (m2_duty > 0 && m2_duty < TEST_MIN_DUTY) m2_duty = TEST_MIN_DUTY;
    if (m3_duty > 0 && m3_duty < TEST_MIN_DUTY) m3_duty = TEST_MIN_DUTY;
    if (m4_duty > 0 && m4_duty < TEST_MIN_DUTY) m4_duty = TEST_MIN_DUTY;
  }

  // Update normalized cmds too (for consistency in telemetry/debug)
  m1_cmd = (float)m1_duty / 255.0f;
  m2_cmd = (float)m2_duty / 255.0f;
  m3_cmd = (float)m3_duty / 255.0f;
  m4_cmd = (float)m4_duty / 255.0f;

  // Low-rate debug print (USB)
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.printf("[MOTOR TEST] which=%d duty=%d,%d,%d,%d acc|=%.3f g gyro=%.1f,%.1f,%.1f\n",
      which, m1_duty, m2_duty, m3_duty, m4_duty, accMag_g, GyroX, GyroY, GyroZ);
  }
}

// ========================= ALL MOTORS SAME THRUST MODE =========================
// This mode bypasses stabilization/mixing and applies the same throttle-derived
// duty cycle to every motor. It is useful for checking if the four motors sound
// similar and spin up in a uniform way without PID corrections masking the test.
//
// Note: We still keep the normal arming + CUT safety logic by calling
// throttleCutAndCommand() afterward in the main loop.
static void allMotorsSameThrust() {
  // Normalize throttle from receiver-style channel units:
  // 1000 -> 0.0 (off), 2000 -> 1.0 (full)
  const float thr_norm = constrain((channel_pwm[0] - 1000.0f) / 1000.0f, 0.0f, 1.0f);

  // Convert normalized throttle into 8-bit PWM duty (0..255).
  int thrust_duty = (int)lroundf(thr_norm * 255.0f);

  // Optional minimum duty just for this equal-thrust test.
  // Raise this if some motors buzz but do not actually spin.
  const int TEST_MIN_DUTY = 0;
  if (TEST_MIN_DUTY > 0 && thrust_duty > 0 && thrust_duty < TEST_MIN_DUTY) {
    thrust_duty = TEST_MIN_DUTY;
  }

  // Apply exactly the same duty to all motors.
  m1_duty = thrust_duty;
  m2_duty = thrust_duty;
  m3_duty = thrust_duty;
  m4_duty = thrust_duty;

  // Update normalized commands too so telemetry/debug stays consistent.
  m1_cmd = (float)m1_duty / 255.0f;
  m2_cmd = (float)m2_duty / 255.0f;
  m3_cmd = (float)m3_duty / 255.0f;
  m4_cmd = (float)m4_duty / 255.0f;

  // Print a slow debug line so you can confirm the commanded shared duty.
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.printf(
      "[ALL MOTORS SAME] thr=%.3f duty=%d acc|=%.3f g gyro=%.1f,%.1f,%.1f\n",
      thr_norm,
      thrust_duty,
      accMag_g,
      GyroX,
      GyroY,
      GyroZ
    );
  }
}


// ========================= SETUP & MAIN LOOP =========================

// Initialization function: runs once when board powers up or resets
// Sets up all hardware: I2C, PWM, IMU sensor, wireless ESP-NOW
//
void setup() {
  // Serial port: for debugging output to USB terminal
  Serial.begin(115200);  // 115200 baud rate (standard for ESP32)
  delay(300);  // Wait for serial to stabilize

  // ========== I2C BUS INITIALIZATION ==========
  // Connect to MPU6050 sensor on I2C bus
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);      // SDA/SCL pins (see I2C_SDA_PIN/I2C_SCL_PIN)
  Wire.setClock(400000);   // Fast mode I2C (400 kHz)

  // ========== PWM OUTPUT SETUP (TB6612FNG Motor Driver) ==========
  // Standby pin: disables both motor drive channels when LOW
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, LOW);  // Start with motors disabled (safe)

  // Configure ESP32 LED-PWM peripheral for 4 motor PWM channels
  // PWM frequency: 20 kHz (inaudible to humans, efficient for motors)
  // PWM resolution: 8-bit (256 levels: 0-255)
  ledcAttach(M1_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(M2_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(M4_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(M3_PIN, PWM_FREQ, PWM_RES);
  
  // Initialize all motor outputs to 0 (off)
  ledcWrite(M1_PIN, 0);
  ledcWrite(M2_PIN, 0);
  ledcWrite(M4_PIN, 0);
  ledcWrite(M3_PIN, 0);

  // ========== IMU SENSOR (MPU6050) INITIALIZATION ==========
  // Initialize 6-DOF inertial measurement unit
  if (!mpuInit()) {
    // Sensor not found or communication failed
    Serial.println("MPU6050 not found at 0x68. Check SDA/SCL/3V3/GND.");
    while(true) { delay(1000); }  // Halt execution with error
  }

  // ========== WiFi & ESP-NOW INITIALIZATION ==========
  // ESP-NOW: low-latency 2.4GHz protocol for controller-drone communication
  WiFi.mode(WIFI_STA);  // Station mode (receiver only, not AP)
  Serial.print("Drone MAC: ");
  Serial.println(WiFi.macAddress());  // Print MAC for pairing with transmitter

  // Initialize ESP-NOW library and register callback to handle incoming packets
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP‑NOW init FAILED");
    while(true) delay(1000);  // Halt on communication failure
  }
  esp_now_register_recv_cb(onEspNowRecv);  // Set ISR for incoming packets

  // Add controller as a peer so the drone can SEND telemetry back
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, CONTROLLER_MAC, 6);
  peer.channel = 0;     // 0 = use current WiFi channel (works if your link already works)
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("ESP-NOW: failed to add controller peer");
  }

  // blink LED a few times to indicate successful startup and pairing
  for (int i = 0; i < 3; i++) {
    digitalWrite(STBY_PIN, HIGH);  // Enable motor driver (LED on)
    delay(200);
    digitalWrite(STBY_PIN, LOW);   // Disable motor driver (LED off)
    delay(200);
  }

  // Wait 3 seconds AFTER the blink sequence, then run the startup calibration.
  // Keep the drone perfectly still during this window.
  delay(3000);

#if PRINT_TEL_ON_USB
  Serial.println("Startup calibration: keep the drone still.");
#endif
  calculate_IMU_error();
  calibrateAttitude();

  // ========== TIMING INITIALIZATION ==========
  tPrev = micros();  // Record startup time for dt calculation
  
  // Ready message: pilot needs to remove props before testing.
  // Startup calibration runs once, and the same joystick-based 3-second
  // calibration is still available later while disarmed.
  Serial.println("Ready. PROPS OFF. Startup calibration finished. You can still recalibrate with the joystick gesture.");
}

// Main control loop: runs repeatedly as fast as possible
// Executes complete flight control cycle: read sensors → compute control → update motors
//
void loop() {
  // ========== TIMING ==========
  // Calculate dt (time step) for PID integration
  uint32_t tNow = micros();
  dt = (tNow - tPrev) / 1000000.0f;  // Convert microseconds to seconds
  tPrev = tNow;

  // ========== STEP 1: GET PILOT INPUT ==========
  // Retrieve latest wireless controller commands (or failsafe if lost signal)
  getCommandsFromEspNow();

  // ========== STEP 2: READ SENSORS ==========
  // Read raw IMU data, apply calibration, apply low-pass filter
  getIMUdata();
  
  // Disarmed: use the current accel pose directly so hand-moved bench tests
  // return to level immediately once the frame is put back flat.
  // Armed: use full gyro+accel fusion for flight.
  if (!armedFly) {
    seedAttitudeFromCurrentAccel();
  } else {
    // Note: Sign conventions are flipped on gyroY, gyroZ, AccX to match body axes
    Madgwick6DOF(-GyroY, GyroX, -GyroZ, -AccY, AccX, AccZ, dt);
  }
  applyLevelTrimToAttitude();
  maybeApplyLevelCalibration();

#if MOTOR_ISOLATION_MODE
  // Diagnostic mode: bypass stabilization and drive one motor at a time.
  motorIsolationTest();
  throttleCutAndCommand();
  sendTelemetry();
  loopRate(LOOP_HZ);
  return;
#endif

#if ALL_MOTORS_SAME_THRUST_MODE
  // Diagnostic mode: bypass stabilization and drive all motors identically.
  allMotorsSameThrust();
  throttleCutAndCommand();
  sendTelemetry();
  loopRate(LOOP_HZ);
  return;
#endif

  // ========== STEP 3: ATTITUDE CONTROL ==========
  // Convert pilot stick inputs → desired attitude angles
  getDesState();
  
  // Compute PID correction for roll, pitch, yaw
  controlANGLE();
  
  // Combine throttle + PID outputs → individual motor speeds
  mixerQuad();
  
  // Convert normalized motor commands (0.0-1.0) → PWM duty cycles (0-255)
  scaleToDuty();
  
  // Apply final safety check and write PWM to motors
  throttleCutAndCommand();



  sendTelemetry();
  /*
  // ========== STEP 4: TELEMETRY (Debug Prints) ==========
  // Print status every 50ms for monitoring on USB serial port
  static uint32_t lastPrint=0;
  if (millis() - lastPrint > 50) {
    lastPrint = millis();
    ControlPacket p; uint32_t age;
    bool ok = getLatestPacket(p, age);
    
    // Print format:
    // ok = packet valid, age = ms since last packet, armed = arm status
    // thr/r/p/y = controller stick inputs, IMU r/p = actual attitude angles
    // duty = individual motor PWM values (0-255), armedFly = system armed state
    Serial.printf("ok=%d age=%lu armed=%d thr=%d r=%d p=%d y=%d | IMU r=%.1f p=%.1f | duty %d %d %d %d | armedFly=%d\n",
      (int)ok, (unsigned long)age, (int)p.armed, (int)p.thr, (int)p.roll, (int)p.pitch, (int)p.yaw,
      roll_IMU, pitch_IMU, m1_duty,m2_duty,m3_duty,m4_duty, (int)armedFly);
  }
  */




  // ========== TIMING CONTROL ==========
  // Wait until next scheduled time to maintain constant loop rate
  loopRate(LOOP_HZ);
}
