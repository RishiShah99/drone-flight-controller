  /*
  ESP32 Brushed Flight Controller (ESP-NOW RX)
  --------------------------------------------
  Receives ControlPacket over ESP-NOW, runs Madgwick 6DOF + angle PID +
  quad mixer, drives four brushed motors through TB6612FNG H-bridges.

  WIRING:
    I2C: SDA=21, SCL=22
    Motors PWM: M1=25, M2=26, M3=27, M4=14
    TB6612 STBY: 33 (tie both boards' STBY together, 10k pull-down to GND)

  SAFETY: PROPS OFF for first tests. Failsafe disarms on >250 ms of packet loss.
  */

  #include <Arduino.h>
  #include <Wire.h>
  #include <WiFi.h>
  #include <esp_now.h>

  // ========================= DIAGNOSTIC MODES =========================
  // Motor isolation: bypasses stabilization and drives ONE motor at a time
  // by stick position (THR->M1, YAW->M2, ROLL->M3, PITCH->M4). Used to
  // verify which GPIO drives which physical corner.
  #define MOTOR_ISOLATION_MODE 0

  // Equal-thrust: all four motors at the throttle value, no PID. Used to
  // check that the frame lifts straight up (isolates thrust/CG issues from
  // PID issues).
  #define ALL_MOTORS_SAME_THRUST_MODE 0

  #define PRINT_TEL_ON_USB 1


  // ========================= USER SETTINGS =========================
  static const int LOOP_HZ  = 500;
  static const int PWM_FREQ = 20000;
  static const int PWM_RES  = 8;

  static const int M1_PIN = 25;  // Front-Left
  static const int M2_PIN = 26;  // Front-Right
  static const int M3_PIN = 27;  // Back-Right
  static const int M4_PIN = 14;  // Back-Left
  static const int STBY_PIN = 33;

  static const int I2C_SDA_PIN = 22;
  static const int I2C_SCL_PIN = 21;

  // Yaw mixing is tied to declared motor spin directions. Positive yaw_PID
  // applies +1 to CCW motors and -1 to CW motors. If your real props spin
  // differently, change ONLY these four constants.
  enum MotorSpinDirection : int8_t {
    MOTOR_SPIN_CW  = -1,
    MOTOR_SPIN_CCW =  1,
  };
  static inline const char* motorSpinName(MotorSpinDirection dir) {
    return (dir == MOTOR_SPIN_CCW) ? "CCW" : "CW";
  }
  static const MotorSpinDirection M1_SPIN = MOTOR_SPIN_CW;
  static const MotorSpinDirection M2_SPIN = MOTOR_SPIN_CCW;
  static const MotorSpinDirection M3_SPIN = MOTOR_SPIN_CW;
  static const MotorSpinDirection M4_SPIN = MOTOR_SPIN_CCW;

  static const uint32_t FAILSAFE_MS = 250;

  float i_limit   = 25.0f;
  float maxRoll   = 30.0f;   // deg
  float maxPitch  = 30.0f;   // deg
  float maxYaw    = 160.0f;  // deg/s

  float Kp_roll_angle  = 0.33f;
  float Ki_roll_angle  = 0.18f;
  float Kd_roll_angle  = 0.06f;
  float Kp_pitch_angle = 0.33f;
  float Ki_pitch_angle = 0.18f;
  float Kd_pitch_angle = 0.06f;

  // Yaw is rate mode (deg/s), so the same numeric Kp produces a much larger
  // mixer command than angle mode. Start much lower than roll/pitch.
  float Kp_yaw = 0.005f;
  float Ki_yaw = 0.0f;
  float Kd_yaw = 0.0f;
  float yaw_pid_limit = 0.08f;  // Hard cap on yaw authority in the mixer

  // Higher = less filtering (faster, noisier). Lower = smoother with more lag.
  float B_madgwick_armed    = 0.06f;  // Conservative under vibration
  float B_madgwick_disarmed = 0.50f;  // Used during still-cal settle windows
  float B_accel = 0.27f;
  float B_gyro  = 0.10f;

  // Adaptive Madgwick recovery: boost beta when the estimator has drifted far
  // from the accel-only attitude AND the accel reads ~1 g. Ramps back down.
  float B_madgwick_recovery    = 0.20f;
  float recovery_threshold_deg = 5.0f;
  float recovery_ramp_rate     = 0.01f;

  // Minimum duty to overcome brushed-motor stiction. 0 for first tests.
  int MIN_DUTY_WHEN_RUNNING = 0;

  float accMag_g = 1.0f;
  float accTrustWeight = 1.0f;
  float accRejectPct = 0.0f;
  float currentBeta = 0.02f;

  // Accel trust gate: if |a| strays far from 1 g, reduce or reject the accel
  // correction so vibration/impacts don't steer the attitude estimate.
  static const float ACC_TRUST_HARD_LOW_G  = 0.70f;
  static const float ACC_TRUST_HARD_HIGH_G = 1.30f;
  static const float ACC_TRUST_SOFT_ERR_G  = 0.25f;

  static const int STARTUP_IMU_CAL_DISCARD_SAMPLES = 100;
  static const int STARTUP_IMU_CAL_SAMPLES         = 800;
  static const int STARTUP_IMU_CAL_DELAY_MS        = 2;
  static const int LEVEL_CAPTURE_DISCARD_SAMPLES   = 30;
  static const int LEVEL_CAPTURE_SAMPLES           = 80;

  // Controller MAC — paste the TX ESP32's MAC here.
  static uint8_t CONTROLLER_MAC[6] = {0x4C,0xC3,0x82,0xDA,0xF8,0x38};


  // ========================= ESP-NOW PACKETS =========================
  // Must match TX_Final.cpp byte-for-byte.
  struct __attribute__((packed)) ControlPacket {
    uint32_t t_ms;
    uint8_t  armed;
    uint8_t  calibrate_level;
    int16_t  thr;    // 0..1000
    int16_t  yaw;    // -500..500
    int16_t  pitch;  // -500..500
    int16_t  roll;   // -500..500
  };

  struct __attribute__((packed)) TelemetryPacket {
    uint32_t t_ms;
    uint8_t  seq;
    uint8_t  rx_ok;
    uint16_t rx_age_ms;

    uint8_t  armed_cmd;
    uint8_t  armedFly;
    uint8_t  stby;
    uint8_t  reserved0;  // accel rejection percent (0..100)

    int16_t  thr, yaw, pitch, roll;

    int16_t  roll_imu_cdeg;   // roll_IMU * 100
    int16_t  pitch_imu_cdeg;  // pitch_IMU * 100

    int16_t  roll_pid_milli;
    int16_t  pitch_pid_milli;
    int16_t  yaw_pid_milli;

    uint8_t  m1_duty, m2_duty, m3_duty, m4_duty;
    uint16_t acc_mg;  // milli-g
  };


  // ISR-shared packet state, protected by a spinlock.
  static portMUX_TYPE packetMux = portMUX_INITIALIZER_UNLOCKED;
  static ControlPacket lastPacket{};
  static uint32_t lastPacketRxMs = 0;
  static bool havePacket = false;

  static void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    (void)info;
    if (len != (int)sizeof(ControlPacket)) return;

    portENTER_CRITICAL(&packetMux);
    memcpy(&lastPacket, data, sizeof(ControlPacket));
    lastPacketRxMs = millis();
    havePacket = true;
    portEXIT_CRITICAL(&packetMux);
  }

  static bool getLatestPacket(void *outBuf, size_t outSize, uint32_t &ageMs) {
    uint32_t rxMs;
    bool ok;

    portENTER_CRITICAL(&packetMux);
    if (outBuf != nullptr && outSize >= sizeof(ControlPacket)) {
      memcpy(outBuf, &lastPacket, sizeof(ControlPacket));
    }
    rxMs = lastPacketRxMs;
    ok = havePacket;
    portEXIT_CRITICAL(&packetMux);

    uint32_t now = millis();
    ageMs = ok ? (now - rxMs) : 0xFFFFFFFFu;
    return ok;
  }

  // ========================= MPU6050 =========================
  // Raw I2C driver, no external library. Bytes from REG_DATA (0x3B) are:
  // 0..1 AccX, 2..3 AccY, 4..5 AccZ, 6..7 Temp, 8..9 GyroX, 10..11 GyroY, 12..13 GyroZ
  static const uint8_t MPU_ADDR       = 0x68;
  static const uint8_t REG_WHOAMI     = 0x75;
  static const uint8_t REG_PWR_MGMT_1 = 0x6B;
  static const uint8_t REG_SMPLRT_DIV = 0x19;
  static const uint8_t REG_CONFIG     = 0x1A;
  static const uint8_t REG_GYRO_CFG   = 0x1B;
  static const uint8_t REG_ACCEL_CFG  = 0x1C;
  static const uint8_t REG_DATA       = 0x3B;

  // Gyro ±250°/s → 131 LSB/(°/s). Accel ±2g → 16384 LSB/g.
  static const float GYRO_SCALE_FACTOR  = 131.0f;
  static const float ACCEL_SCALE_FACTOR = 16384.0f;

  static bool i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
  }

  static bool i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;

    size_t got = Wire.requestFrom((int)addr, (int)n, (int)true);
    if (got != n) return false;

    for (size_t i=0;i<n;i++) buf[i] = Wire.read();
    return true;
  }

  static bool mpuInit() {
    uint8_t who=0;
    if (!i2cReadBytes(MPU_ADDR, REG_WHOAMI, &who, 1)) return false;
    if (who != 0x68) return false;

    if (!i2cWriteByte(MPU_ADDR, REG_PWR_MGMT_1, 0x00)) return false;  // wake
    delay(50);

    i2cWriteByte(MPU_ADDR, REG_SMPLRT_DIV, 0x00);
    // DLPF ~44 Hz: good starting point when motor vibration dominates accel.
    i2cWriteByte(MPU_ADDR, REG_CONFIG, 0x03);
    i2cWriteByte(MPU_ADDR, REG_GYRO_CFG, 0x00);   // ±250°/s
    i2cWriteByte(MPU_ADDR, REG_ACCEL_CFG, 0x00);  // ±2 g
    delay(10);
    return true;
  }

  static bool mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
    uint8_t b[14];
    if (!i2cReadBytes(MPU_ADDR, REG_DATA, b, 14)) return false;

    ax = (int16_t)((b[0]<<8)  | b[1]);
    ay = (int16_t)((b[2]<<8)  | b[3]);
    az = (int16_t)((b[4]<<8)  | b[5]);
    gx = (int16_t)((b[8]<<8)  | b[9]);
    gy = (int16_t)((b[10]<<8) | b[11]);
    gz = (int16_t)((b[12]<<8) | b[13]);
    return true;
  }

  static void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float dt);
  static void getIMUdata();

  // ========================= STATE =========================
  float dt = 0.0f;
  uint32_t tPrev = 0;

  // Pseudo-PWM channels from ESP-NOW: [thr, roll, pitch, yaw, cut]
  // thr 1000..2000, roll/pitch/yaw 1000..2000 (1500 = neutral), cut >1500 = disarm.
  float channel_pwm[5] = {1000,1500,1500,1500,2000};
  const float channel_Failsafe[5] = {1000,1500,1500,1500,2000};

  float AccX=0, AccY=0, AccZ=0;         // g
  float GyroX=0, GyroY=0, GyroZ=0;      // deg/s
  float AccX_prev=0, AccY_prev=0, AccZ_prev=0;
  float GyroX_prev=0, GyroY_prev=0, GyroZ_prev=0;

  float AccErrorX = 0.0f, AccErrorY = 0.0f, AccErrorZ = 0.0f;
  float GyroErrorX = 0.0f, GyroErrorY = 0.0f, GyroErrorZ = 0.0f;

  float roll_IMU=0, pitch_IMU=0, yaw_IMU=0;
  float q0=1, q1=0, q2=0, q3=0;
  float levelRollTrimDeg = 0.0f;
  float levelPitchTrimDeg = 0.0f;
  bool calibrateLevelCmd = false;
  bool prevCalibrateLevelCmd = false;

  float thro_des=0, roll_des=0, pitch_des=0, yaw_des=0;

  float error_roll=0, integral_roll=0, derivative_roll=0, roll_PID=0;
  float error_pitch=0, integral_pitch=0, derivative_pitch=0, pitch_PID=0;
  float error_yaw=0, error_yaw_prev=0, integral_yaw=0, derivative_yaw=0, yaw_PID=0;

  float m1_cmd=0, m2_cmd=0, m4_cmd=0, m3_cmd=0;
  int m1_duty=0, m2_duty=0, m4_duty=0, m3_duty=0;

  bool armedFly = false;

  // Still-on-ground calibration: gyro bias + accel bias along the measured
  // gravity direction. Does NOT assume the frame is perfectly level.
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

    if (validSamples < (STARTUP_IMU_CAL_SAMPLES / 2)) return false;

    const float invCount = 1.0f / (float)validSamples;
    const float avgAx = sumAx * invCount;
    const float avgAy = sumAy * invCount;
    const float avgAz = sumAz * invCount;
    const float avgGx = sumGx * invCount;
    const float avgGy = sumGy * invCount;
    const float avgGz = sumGz * invCount;
    const float avgAccNorm = sqrtf(avgAx*avgAx + avgAy*avgAy + avgAz*avgAz);

    GyroErrorX = avgGx;
    GyroErrorY = avgGy;
    GyroErrorZ = avgGz;

    // Remove only the gravity-parallel accel error so a slightly non-level
    // resting pose doesn't pollute the bias estimate.
    if (avgAccNorm > 0.80f && avgAccNorm < 1.20f) {
      const float gravityBias = avgAccNorm - 1.0f;
      const float gravityDirX = avgAx / avgAccNorm;
      const float gravityDirY = avgAy / avgAccNorm;
      const float gravityDirZ = avgAz / avgAccNorm;

      AccErrorX = gravityBias * gravityDirX;
      AccErrorY = gravityBias * gravityDirY;
      AccErrorZ = gravityBias * gravityDirZ;
    } else {
      AccErrorX = AccErrorY = AccErrorZ = 0.0f;
    }

    AccX_prev = avgAx - AccErrorX;
    AccY_prev = avgAy - AccErrorY;
    AccZ_prev = avgAz - AccErrorZ;
    GyroX_prev = GyroY_prev = GyroZ_prev = 0.0f;
    accMag_g = sqrtf(AccX_prev*AccX_prev + AccY_prev*AccY_prev + AccZ_prev*AccZ_prev);

    Serial.printf(
      "Still IMU calibration: accNormRaw=%.3f g accNormCorrected=%.3f g | accBias=%.4f,%.4f,%.4f g | gyroBias=%.3f,%.3f,%.3f dps\n",
      avgAccNorm, accMag_g,
      AccErrorX, AccErrorY, AccErrorZ,
      GyroErrorX, GyroErrorY, GyroErrorZ
    );

    return true;
  }

  // ========================= MADGWICK 6DOF =========================
  // Reference: https://ahrs.readthedocs.io/en/latest/filters/madgwick.html
  static float invSqrt(float x) { return 1.0f / sqrtf(x); }

  static void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // deg/s → rad/s
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    // Accel trust: 1.0 when |a| ≈ 1 g, fades linearly to 0 at ±0.25 g error,
    // hard rejects outside [0.70, 1.30] g.
    float accMag = sqrtf(ax*ax + ay*ay + az*az);
    float accW = 1.0f;
    if (accMag < ACC_TRUST_HARD_LOW_G || accMag > ACC_TRUST_HARD_HIGH_G) {
      accW = 0.0f;
    } else {
      float err = fabsf(accMag - 1.0f);
      accW = 1.0f - (err / ACC_TRUST_SOFT_ERR_G);
      accW = constrain(accW, 0.0f, 1.0f);
    }
    accTrustWeight = accW;
    accRejectPct = (1.0f - accW) * 100.0f;

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
      recipNorm = invSqrt(ax*ax + ay*ay + az*az);
      ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

      _2q0 = 2.0f*q0; _2q1 = 2.0f*q1; _2q2 = 2.0f*q2; _2q3 = 2.0f*q3;
      _4q0 = 4.0f*q0; _4q1 = 4.0f*q1; _4q2 = 4.0f*q2;
      _8q1 = 8.0f*q1; _8q2 = 8.0f*q2;
      q0q0 = q0*q0; q1q1 = q1*q1; q2q2 = q2*q2; q3q3 = q3*q3;

      s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
      s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
      s2 = 4.0f*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
      s3 = 4.0f*q1q1*q3 - _2q1*ax + 4.0f*q2q2*q3 - _2q2*ay;

      recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
      s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

      // Adaptive beta: boost to recovery value when the estimator drifts
      // far from the accel-only attitude and accel is trustworthy; ramp back
      // down to the armed base value over time.
      float betaBase;
      if (!armedFly) {
        betaBase = B_madgwick_disarmed;
        currentBeta = B_madgwick_armed;
      } else {
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

    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;

    roll_IMU  = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * 57.29577951f;
    pitch_IMU = asinf(constrain(-2.0f*(q1*q3 - q0*q2), -0.999999f, 0.999999f)) * 57.29577951f;
    yaw_IMU   = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3) * 57.29577951f;
  }

  // ========================= CORE FUNCTIONS =========================
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

    accMag_g = sqrtf(AccX*AccX + AccY*AccY + AccZ*AccZ);
  }

  static void getCommandsFromEspNow() {
    ControlPacket p{};
    uint32_t age;
    bool ok = getLatestPacket(&p, sizeof(p), age);

    if (!ok || age > FAILSAFE_MS) {
      for (int i=0;i<5;i++) channel_pwm[i] = channel_Failsafe[i];
      armedFly = false;
      calibrateLevelCmd = false;
      return;
    }

    int16_t thr   = constrain((int)p.thr,   0, 1000);
    int16_t roll  = constrain((int)p.roll,  -500, 500);
    int16_t pitch = constrain((int)p.pitch, -500, 500);
    int16_t yaw   = constrain((int)p.yaw,   -500, 500);

    channel_pwm[0] = 1000.0f + (float)thr;
    channel_pwm[1] = 1500.0f + (float)roll;
    channel_pwm[2] = 1500.0f + (float)pitch;
    channel_pwm[3] = 1500.0f + (float)yaw;

    bool pktArmed = (p.armed != 0);
    channel_pwm[4] = pktArmed ? 1000.0f : 2000.0f;
    calibrateLevelCmd = (!pktArmed) && (p.calibrate_level != 0);

    // Latched arming: disarm is immediate; DISARMED→ARMED transition
    // requires throttle near idle so props can't spin up on arm.
    if (!pktArmed) {
      armedFly = false;
    } else if (!armedFly) {
      if (channel_pwm[0] < 1100.0f) armedFly = true;
    }
  }

  static void applyLevelTrimToAttitude() {
    roll_IMU -= levelRollTrimDeg;
    pitch_IMU -= levelPitchTrimDeg;
  }

  // Single source of truth for the sensor→body-frame remap. Used by the
  // estimator, the PID derivative terms, and the debug prints so signs
  // never disagree.
  static inline float bodyRollRateDps()  { return -GyroY; }
  static inline float bodyPitchRateDps() { return  GyroX; }
  static inline float bodyYawRateDps()   { return -GyroZ; }

  static inline void getBodyAccelG(float &axBody, float &ayBody, float &azBody) {
    axBody = -AccY;
    ayBody =  AccX;
    azBody =  AccZ;
  }

  static inline void getBodyGyroDps(float &gxBody, float &gyBody, float &gzBody) {
    gxBody = bodyRollRateDps();
    gyBody = bodyPitchRateDps();
    gzBody = bodyYawRateDps();
  }

  // Seed the quaternion from the current accel reading (yaw reset to 0, since
  // accel does not observe heading).
  static void seedAttitudeFromCurrentAccel() {
    float ax, ay, az;
    getBodyAccelG(ax, ay, az);

    const float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 0.5f) {
      q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
      roll_IMU = pitch_IMU = yaw_IMU = 0.0f;
      return;
    }

    ax /= norm; ay /= norm; az /= norm;

    const float rollRad = atan2f(ay, az);
    const float pitchRad = atan2f(-ax, sqrtf(ay*ay + az*az));
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

  // Re-seed from current still pose and re-apply level trim. Avoids the
  // "snaps back to zero slowly" feel after a fresh level calibration.
  static void refreshAttitudeEstimateFromCurrentPose() {
    getIMUdata();
    seedAttitudeFromCurrentAccel();
    applyLevelTrimToAttitude();
  }

  // After still-IMU calibration, let the estimator settle and average the
  // resulting roll/pitch. Cleaner software-zero than a single sample.
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
      float gxBody, gyBody, gzBody;
      float axBody, ayBody, azBody;
      getBodyGyroDps(gxBody, gyBody, gzBody);
      getBodyAccelG(axBody, ayBody, azBody);
      Madgwick6DOF(gxBody, gyBody, gzBody, axBody, ayBody, azBody, settleDt);

      if (i >= LEVEL_CAPTURE_DISCARD_SAMPLES) {
        sumRoll += roll_IMU;
        sumPitch += pitch_IMU;
        ++validSamples;
      }
    }

    if (validSamples < (LEVEL_CAPTURE_SAMPLES / 2)) return false;

    rollTrimOut = sumRoll / (float)validSamples;
    pitchTrimOut = sumPitch / (float)validSamples;
    return true;
  }

  // Disarm + zero everything so a calibration cannot leave stale motor/PID
  // state behind.
  static void forceOutputsSafeAndResetControllers() {
    armedFly = false;
    channel_pwm[0] = 1000.0f;
    channel_pwm[1] = 1500.0f;
    channel_pwm[2] = 1500.0f;
    channel_pwm[3] = 1500.0f;
    channel_pwm[4] = 2000.0f;
    thro_des = roll_des = pitch_des = yaw_des = 0.0f;
    m1_cmd = m2_cmd = m3_cmd = m4_cmd = 0.0f;
    m1_duty = m2_duty = m3_duty = m4_duty = 0;
    error_roll = error_pitch = error_yaw = error_yaw_prev = 0.0f;
    integral_roll = integral_pitch = integral_yaw = 0.0f;
    derivative_roll = derivative_pitch = derivative_yaw = 0.0f;
    roll_PID = pitch_PID = yaw_PID = 0.0f;
    currentBeta = B_madgwick_armed;

    digitalWrite(STBY_PIN, LOW);
    ledcWrite(M1_PIN, 0);
    ledcWrite(M2_PIN, 0);
    ledcWrite(M3_PIN, 0);
    ledcWrite(M4_PIN, 0);
  }

  // Joystick gesture requests a still IMU calibration and stores the current
  // pose as the new level reference. Only runs while disarmed and at idle.
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
          levelRollTrimDeg, levelPitchTrimDeg, accMag_g
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

  // ========================= DESIRED STATE =========================
  static void getDesState() {
    float thr_norm  = (channel_pwm[0] - 1000.0f)/1000.0f;
    float roll_norm = (channel_pwm[1] - 1500.0f)/500.0f;
    float pit_norm  = (channel_pwm[2] - 1500.0f)/500.0f;
    float yaw_norm  = (channel_pwm[3] - 1500.0f)/500.0f;

    thro_des  = constrain(thr_norm, 0.0f, 1.0f);
    roll_des  = constrain(roll_norm, -1.0f, 1.0f) * maxRoll;
    pitch_des = constrain(pit_norm,  -1.0f, 1.0f) * maxPitch;
    yaw_des   = constrain(yaw_norm,  -1.0f, 1.0f) * maxYaw;
  }

  // ========================= PID CONTROL =========================
  // Roll/pitch: angle mode. Yaw: rate mode (no gravity reference for heading).
  // Derivative-on-measurement for roll/pitch (avoids derivative kick on
  // setpoint changes). 0.01f scaling factor is standard from dRehmFlight.
  static void controlANGLE() {
    error_roll = roll_des - roll_IMU;
    integral_roll += error_roll * dt;
    if (channel_pwm[0] < 1060) integral_roll = 0;  // anti-windup on ground
    integral_roll = constrain(integral_roll, -i_limit, i_limit);
    derivative_roll = bodyRollRateDps();
    roll_PID = 0.01f * (Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - Kd_roll_angle*derivative_roll);

    error_pitch = pitch_des - pitch_IMU;
    integral_pitch += error_pitch * dt;
    if (channel_pwm[0] < 1060) integral_pitch = 0;
    integral_pitch = constrain(integral_pitch, -i_limit, i_limit);
    derivative_pitch = bodyPitchRateDps();
    pitch_PID = 0.01f * (Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch);

    error_yaw = yaw_des - bodyYawRateDps();
    integral_yaw += error_yaw * dt;
    if (channel_pwm[0] < 1060) integral_yaw = 0;
    integral_yaw = constrain(integral_yaw, -i_limit, i_limit);
    derivative_yaw = (error_yaw - error_yaw_prev) / max(dt, 1e-6f);

    yaw_PID = 0.01f * (Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw);
    // yaw_PID = constrain(yaw_PID, -yaw_pid_limit, yaw_pid_limit);

    error_yaw_prev = error_yaw;
  }

  // ========================= QUAD-X MIXER =========================
  // Yaw contribution is derived from each motor's declared spin direction
  // (M1_SPIN..M4_SPIN), not hard-coded signs.
  static void mixerQuad() {
    m1_cmd = thro_des - pitch_PID + roll_PID + ((float)M1_SPIN * yaw_PID);
    m2_cmd = thro_des - pitch_PID - roll_PID + ((float)M2_SPIN * yaw_PID);
    m3_cmd = thro_des + pitch_PID - roll_PID + ((float)M3_SPIN * yaw_PID);
    m4_cmd = thro_des + pitch_PID + roll_PID + ((float)M4_SPIN * yaw_PID);

    m1_cmd = constrain(m1_cmd, 0.0f, 1.0f);
    m2_cmd = constrain(m2_cmd, 0.0f, 1.0f);
    m4_cmd = constrain(m4_cmd, 0.0f, 1.0f);
    m3_cmd = constrain(m3_cmd, 0.0f, 1.0f);
  }

  static void scaleToDuty() {
    m1_duty = (int)lroundf(m1_cmd * 255.0f);
    m2_duty = (int)lroundf(m2_cmd * 255.0f);
    m4_duty = (int)lroundf(m4_cmd * 255.0f);
    m3_duty = (int)lroundf(m3_cmd * 255.0f);

    // Minimum duty to overcome static friction on weak brushed motors.
    if (MIN_DUTY_WHEN_RUNNING > 0 && thro_des > 0.05f) {
      if (m1_duty>0 && m1_duty<MIN_DUTY_WHEN_RUNNING) m1_duty = MIN_DUTY_WHEN_RUNNING;
      if (m2_duty>0 && m2_duty<MIN_DUTY_WHEN_RUNNING) m2_duty = MIN_DUTY_WHEN_RUNNING;
      if (m4_duty>0 && m4_duty<MIN_DUTY_WHEN_RUNNING) m4_duty = MIN_DUTY_WHEN_RUNNING;
      if (m3_duty>0 && m3_duty<MIN_DUTY_WHEN_RUNNING) m3_duty = MIN_DUTY_WHEN_RUNNING;
    }

    m1_duty = constrain(m1_duty, 0, 255);
    m2_duty = constrain(m2_duty, 0, 255);
    m4_duty = constrain(m4_duty, 0, 255);
    m3_duty = constrain(m3_duty, 0, 255);
  }

  // Final safety gate: STBY low + PWM 0 on every disarmed or cut cycle.
  static void throttleCutAndCommand() {
    const bool cut = (channel_pwm[4] > 1500) || (!armedFly);

    if (cut) {
      armedFly = false;
      digitalWrite(STBY_PIN, LOW);
      ledcWrite(M1_PIN, 0);
      ledcWrite(M2_PIN, 0);
      ledcWrite(M4_PIN, 0);
      ledcWrite(M3_PIN, 0);
      return;
    }

    digitalWrite(STBY_PIN, HIGH);
    ledcWrite(M1_PIN, m1_duty);
    ledcWrite(M2_PIN, m2_duty);
    ledcWrite(M4_PIN, m4_duty);
    ledcWrite(M3_PIN, m3_duty);
  }

  // Busy-wait to the next scheduled tick. Keeps PID dt stable across WiFi
  // jitter. If the loop overruns we simply skip the wait.
  static void loopRate(int hz) {
    static uint32_t tNext = 0;
    if (tNext == 0) tNext = micros();

    uint32_t period = 1000000UL / (uint32_t)hz;
    while ((int32_t)(micros() - tNext) < 0) { }
    tNext += period;
  }


  // ========================= TELEMETRY =========================
  static uint32_t lastTelTxMs = 0;
  static uint8_t  telSeq = 0;

  static void sendTelemetry() {
    if (millis() - lastTelTxMs < 50) return;  // 20 Hz
    lastTelTxMs = millis();

    ControlPacket p;
    uint32_t age = 0;
    bool ok = getLatestPacket(&p, sizeof(p), age);

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
    const float yawRateBody = bodyYawRateDps();
    static uint32_t lastTelPrintMs = 0;
    if (millis() - lastTelPrintMs >= 50) {
      lastTelPrintMs = millis();
      Serial.printf(
        "%lu -> TEL age=%lums ok=%d seq=%u | armedCmd=%d armedFly=%d STBY=%d | thr=%d yaw=%d pitch=%d roll=%d | IMU r=%.2f p=%.2f yRate=%.1f | PID r=%.3f p=%.3f y=%.3f | duty %3d %3d %3d %3d acc|=%.3f g aRej=%.1f%%\n",
        (unsigned long)millis(),
        (unsigned long)age,
        ok ? 1 : 0,
        (unsigned int)t.seq,
        (int)t.armed_cmd, (int)t.armedFly, (int)t.stby,
        (int)t.thr, (int)t.yaw, (int)t.pitch, (int)t.roll,
        roll_IMU, pitch_IMU, yawRateBody,
        roll_PID, pitch_PID, yaw_PID,
        (int)constrain(m1_duty, 0, 255),
        (int)constrain(m2_duty, 0, 255),
        (int)constrain(m3_duty, 0, 255),
        (int)constrain(m4_duty, 0, 255),
        accMag_g, accRejectPct
      );
    }
  #endif
  }


  void calculate_IMU_error() {
    forceOutputsSafeAndResetControllers();
    (void)calibrateImuStill();
  }

  void calibrateAttitude() {
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
  // Bypasses stabilization to drive ONE motor at a time by stick position.
  // Lets you watch accMag_g per motor to find the dominant vibration source.
  //
  // Stick-to-motor mapping:
  //   Throttle near full + other sticks centered -> Motor 1
  //   Yaw > others  -> Motor 2
  //   Roll > others -> Motor 3
  //   Pitch > others -> Motor 4
  static void motorIsolationTest() {
    const float thr_norm  = constrain((channel_pwm[0] - 1000.0f)/1000.0f, 0.0f, 1.0f);
    const float roll_norm = constrain((channel_pwm[1] - 1500.0f)/500.0f, -1.0f, 1.0f);
    const float pit_norm  = constrain((channel_pwm[2] - 1500.0f)/500.0f, -1.0f, 1.0f);
    const float yaw_norm  = constrain((channel_pwm[3] - 1500.0f)/500.0f, -1.0f, 1.0f);

    const float aRoll = fabsf(roll_norm);
    const float aPit  = fabsf(pit_norm);
    const float aYaw  = fabsf(yaw_norm);

    const float DB = 0.12f;          // deadband against stick noise
    const int TEST_MAX_DUTY = 200;   // safety cap during isolation tests

    auto map01ToDuty = [&](float x)->int {
      x = constrain(x, 0.0f, 1.0f);
      int d = (int)lroundf(x * (float)TEST_MAX_DUTY);
      return constrain(d, 0, TEST_MAX_DUTY);
    };

    m1_duty = m2_duty = m3_duty = m4_duty = 0;
    int which = 0;

    const bool otherCentered = (aRoll < DB && aPit < DB && aYaw < DB);

    if (thr_norm > 0.95f && otherCentered) {
      which = 1;
      m1_duty = map01ToDuty(thr_norm);
    } else {
      float best = 0.0f;
      if (aYaw  > DB && aYaw  >= best) { best = aYaw;  which = 2; }
      if (aRoll > DB && aRoll >  best) { best = aRoll; which = 3; }
      if (aPit  > DB && aPit  >  best) { best = aPit;  which = 4; }

      if (which == 2) m2_duty = map01ToDuty(aYaw);
      if (which == 3) m3_duty = map01ToDuty(aRoll);
      if (which == 4) m4_duty = map01ToDuty(aPit);
    }

    const int TEST_MIN_DUTY = 0;  // raise if a motor won't start (e.g. 30-60)
    if (TEST_MIN_DUTY > 0) {
      if (m1_duty > 0 && m1_duty < TEST_MIN_DUTY) m1_duty = TEST_MIN_DUTY;
      if (m2_duty > 0 && m2_duty < TEST_MIN_DUTY) m2_duty = TEST_MIN_DUTY;
      if (m3_duty > 0 && m3_duty < TEST_MIN_DUTY) m3_duty = TEST_MIN_DUTY;
      if (m4_duty > 0 && m4_duty < TEST_MIN_DUTY) m4_duty = TEST_MIN_DUTY;
    }

    m1_cmd = (float)m1_duty / 255.0f;
    m2_cmd = (float)m2_duty / 255.0f;
    m3_cmd = (float)m3_duty / 255.0f;
    m4_cmd = (float)m4_duty / 255.0f;

    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 100) {
      lastPrint = millis();
      Serial.printf("[MOTOR TEST] which=%d duty=%d,%d,%d,%d acc|=%.3f g gyro=%.1f,%.1f,%.1f\n",
        which, m1_duty, m2_duty, m3_duty, m4_duty, accMag_g, GyroX, GyroY, GyroZ);
    }
  }

  // All motors driven at the same throttle-derived duty. Confirms the frame
  // lifts straight up without PID corrections masking a thrust/CG imbalance.
  static void allMotorsSameThrust() {
    const float thr_norm = constrain((channel_pwm[0] - 1000.0f) / 1000.0f, 0.0f, 1.0f);
    int thrust_duty = (int)lroundf(thr_norm * 255.0f);

    const int TEST_MIN_DUTY = 0;
    if (TEST_MIN_DUTY > 0 && thrust_duty > 0 && thrust_duty < TEST_MIN_DUTY) {
      thrust_duty = TEST_MIN_DUTY;
    }

    m1_duty = m2_duty = m3_duty = m4_duty = thrust_duty;

    m1_cmd = m2_cmd = m3_cmd = m4_cmd = (float)thrust_duty / 255.0f;

    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 100) {
      lastPrint = millis();
      Serial.printf(
        "[ALL MOTORS SAME] thr=%.3f duty=%d acc|=%.3f g gyro=%.1f,%.1f,%.1f\n",
        thr_norm, thrust_duty, accMag_g, GyroX, GyroY, GyroZ
      );
    }
  }


  // ========================= SETUP & LOOP =========================
  void setup() {
    Serial.begin(115200);
    delay(300);

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);

    pinMode(STBY_PIN, OUTPUT);
    digitalWrite(STBY_PIN, LOW);

    ledcAttach(M1_PIN, PWM_FREQ, PWM_RES);
    ledcAttach(M2_PIN, PWM_FREQ, PWM_RES);
    ledcAttach(M4_PIN, PWM_FREQ, PWM_RES);
    ledcAttach(M3_PIN, PWM_FREQ, PWM_RES);
    ledcWrite(M1_PIN, 0);
    ledcWrite(M2_PIN, 0);
    ledcWrite(M4_PIN, 0);
    ledcWrite(M3_PIN, 0);

    if (!mpuInit()) {
      Serial.println("MPU6050 not found at 0x68. Check SDA/SCL/3V3/GND.");
      while(true) { delay(1000); }
    }
    Serial.printf(
      "Axis remap: roll=-GyroY pitch=GyroX yaw=-GyroZ | spins M1=%s M2=%s M3=%s M4=%s | yawPidLimit=%.2f\n",
      motorSpinName(M1_SPIN), motorSpinName(M2_SPIN),
      motorSpinName(M3_SPIN), motorSpinName(M4_SPIN),
      yaw_pid_limit
    );

    WiFi.mode(WIFI_STA);
    Serial.print("Drone MAC: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
      Serial.println("ESP-NOW init FAILED");
      while(true) delay(1000);
    }
    esp_now_register_recv_cb(onEspNowRecv);

    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, CONTROLLER_MAC, 6);
    peer.channel = 0;
    peer.encrypt = false;

    if (esp_now_add_peer(&peer) != ESP_OK) {
      Serial.println("ESP-NOW: failed to add controller peer");
    }

    // Blink STBY to indicate successful startup.
    for (int i = 0; i < 3; i++) {
      digitalWrite(STBY_PIN, HIGH);
      delay(200);
      digitalWrite(STBY_PIN, LOW);
      delay(200);
    }

    // Keep the drone still during the startup calibration window.
    delay(3000);

  #if PRINT_TEL_ON_USB
    Serial.println("Startup calibration: keep the drone still.");
  #endif
    calculate_IMU_error();
    calibrateAttitude();

    tPrev = micros();
    Serial.println("Ready. PROPS OFF. Startup calibration finished. You can still recalibrate with the joystick gesture.");
  }

  void loop() {
    uint32_t tNow = micros();
    dt = (tNow - tPrev) / 1000000.0f;
    tPrev = tNow;

    getCommandsFromEspNow();
    getIMUdata();

    // Disarmed: use accel pose directly so bench tests snap back to level.
    // Armed: full gyro+accel fusion.
    if (!armedFly) {
      seedAttitudeFromCurrentAccel();
    } else {
      float gxBody, gyBody, gzBody;
      float axBody, ayBody, azBody;
      getBodyGyroDps(gxBody, gyBody, gzBody);
      getBodyAccelG(axBody, ayBody, azBody);
      Madgwick6DOF(gxBody, gyBody, gzBody, axBody, ayBody, azBody, dt);
    }
    applyLevelTrimToAttitude();
    maybeApplyLevelCalibration();

  #if MOTOR_ISOLATION_MODE
    motorIsolationTest();
    throttleCutAndCommand();
    sendTelemetry();
    loopRate(LOOP_HZ);
    return;
  #endif

  #if ALL_MOTORS_SAME_THRUST_MODE
    allMotorsSameThrust();
    throttleCutAndCommand();
    sendTelemetry();
    loopRate(LOOP_HZ);
    return;
  #endif

    getDesState();
    controlANGLE();
    mixerQuad();
    scaleToDuty();
    throttleCutAndCommand();
    sendTelemetry();

    loopRate(LOOP_HZ);
  }
