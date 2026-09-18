/* ======================= gy85_imu.cpp =======================
   Implementation of the GY-85 IMU module - see gy85_imu.h.
   =========================================================== */

#include "gy85_imu.h"

#include <Preferences.h>
#include <Wire.h>
#include <esp_attr.h>

// Degrees <-> radians conversion factors (pi/180 and 180/pi). These are
// mathematical constants - not settings - so they are fixed here; constexpr
// keeps them typed and visible to the compiler/debugger (call sites use the
// same names).
constexpr float DEG2RAD_F = 3.14159265358979323846f / 180.0f;
constexpr float RAD2DEG_F = 180.0f / 3.14159265358979323846f;

// ADXL345 at +/-2g full resolution -> 256 LSB/g
#define GY85_ACCEL_LSB_PER_G 256.0f
// ITG3205 at +/-2000 dps -> 14.375 LSB per deg/s
#define GY85_GYRO_LSB_PER_DPS 14.375f

/* ======================= driver state ======================= */

static uint8_t adxlAddr = 0x53;  // ADXL345; some boards strap it to 0x1D
static uint8_t itgAddr = 0x68;   // ITG3205; some boards strap it to 0x69

// Accel / gyro zero offsets (raw counts), measured at startup.
static int16_t accelOffset[3] = {0, 0, 0};
static int16_t gyroOffset[3] = {0, 0, 0};

// The calibration survives deep sleep: it is kept in RTC memory and reused
// after a motion wake-up, so the module may already be moving at wake.
RTC_DATA_ATTR static bool calStored = false;
RTC_DATA_ATTR static int16_t calAccelOffset[3] = {0, 0, 0};
RTC_DATA_ATTR static int16_t calGyroOffset[3] = {0, 0, 0};

// Magnetometer hard-iron offsets (raw counts) - see gy85_imu.h.
static int16_t magOffset[3] = {
    GY85_MAG_OFFSET_X,
    GY85_MAG_OFFSET_Y,
    GY85_MAG_OFFSET_Z,
};

// Raw magnetometer values from the last gy85Update() (exported).
int16_t gy85MagRaw[3] = {0, 0, 0};

// Fused angles (degrees), updated by gy85Update() (exported).
float gy85Roll = 0.0f, gy85Pitch = 0.0f, gy85Yaw = 0.0f;

// True while the module is being moved (exported; used by the sleep logic).
bool gy85Moving = true;

// True while a re-calibration window is running (exported; sleep gate).
bool gy85RecalActive = false;

// Manual calibration request / reply (exported; see gy85_imu.h).
volatile bool gy85CalRequest = false;
volatile uint8_t gy85CalReply = 0;

// True while the module lies flat (Z carries +1 g) and does not move.
static bool gy85FlatStill = false;

// Raw accel / gyro values from the last gy85Update() (for re-calibration).
static int16_t gy85AccelRaw[3] = {0, 0, 0};
static int16_t gy85GyroRaw[3] = {0, 0, 0};

// Automatic re-calibration state - see gy85RecalTick().
static bool recalDone = false;  // one re-calibration per "still session"
static unsigned long recalFlatSince = 0;
static unsigned long recalStartMs = 0;
static int32_t recalSumAccel[3] = {0, 0, 0};
static int32_t recalSumGyro[3] = {0, 0, 0};
static uint32_t recalCount = 0;
static bool recalManual = false;  // current window was requested manually

/* One-dimensional Kalman filter fusing a gyro rate with a reference angle
   (classic TKJ-electronics structure, like the esp-idf-gy85 demo). */
struct Gy85Kalman {
  float angle = 0.0f;  // fused angle (deg)
  float bias = 0.0f;   // gyro bias estimate (deg/s)
  float P[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
  float Q_angle = 0.001f;
  float Q_bias = 0.003f;
  float R_measure = 0.03f;

  void setAngle(float newAngle) { angle = newAngle; }

  float getAngle(float newAngle, float newRate, float dt) {
    // Predict
    float rate = newRate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Update
    float S = P[0][0] + R_measure;
    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;
    float y = newAngle - angle;
    angle += K0 * y;
    bias += K1 * y;

    float P00 = P[0][0], P01 = P[0][1];
    P[0][0] -= K0 * P00;
    P[0][1] -= K0 * P01;
    P[1][0] -= K1 * P00;
    P[1][1] -= K1 * P01;

    return angle;
  }
};

static Gy85Kalman gy85KalmanX, gy85KalmanY, gy85KalmanZ;

/* ======================= low-level I2C ======================= */

static uint8_t gy85Read(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom(addr, len) != len) return 0;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return 1;
}

static uint8_t gy85Write(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

/* ======================= public API ======================= */

bool gy85Init(void) {
  Wire.begin(GY85_PIN_SDA, GY85_PIN_SCL, 100000);

  /* --- ADXL345 accelerometer --- */
  uint8_t id = 0;
  if (!gy85Read(0x53, 0x00, &id, 1) || id != 0xE5) {
    adxlAddr = 0x1D;
    if (!gy85Read(adxlAddr, 0x00, &id, 1) || id != 0xE5) return false;
  }
  gy85Write(adxlAddr, 0x31, 0x08);  // DATA_FORMAT: full resolution, +/-2g
  gy85Write(adxlAddr, 0x2C, 0x0A);  // BW_RATE: 100 Hz output rate
  gy85Write(adxlAddr, 0x2D, 0x08);  // POWER_CTL: measurement mode
  // After a deep-sleep wake the activity interrupt is still configured -
  // disable it and clear anything pending before measuring again.
  gy85Write(adxlAddr, 0x2E, 0x00);  // INT_ENABLE: all interrupts off
  uint8_t intSrc;
  gy85Read(adxlAddr, 0x30, &intSrc, 1);  // INT_SOURCE: clear pending

  /* --- ITG3205 gyroscope --- */
  if (!gy85Read(0x68, 0x00, &id, 1) || id != 0x68) {
    itgAddr = 0x69;
    if (!gy85Read(itgAddr, 0x00, &id, 1) || id != 0x68) return false;
  }
  gy85Write(itgAddr, 0x3E, 0x80);  // PWR_MGM: device reset
  delay(50);
  gy85Write(itgAddr, 0x3E, 0x01);  // PWR_MGM: clock from X-gyro PLL
  gy85Write(itgAddr, 0x16, 0x1B);  // DLPF_FS: +/-2000 dps, 42 Hz low pass
  gy85Write(itgAddr, 0x15, 0x09);  // SMPLRT_DIV: 1 kHz / 10 = 100 Hz

  /* --- QMC5883L magnetometer --- */
  gy85Write(0x0D, 0x0B, 0x01);  // SET/RESET period
  gy85Write(0x0D, 0x09, 0x05);  // CTRL1: 512 OSR, +/-2 G, 50 Hz, continuous

  /* --- accel + gyro zero-offset calibration --- */
  if (calStored) {
    // Deep-sleep wake: reuse the calibration from RTC memory; the module may
    // already be moving, so a fresh calibration is not possible.
    memcpy(accelOffset, calAccelOffset, sizeof(accelOffset));
    memcpy(gyroOffset, calGyroOffset, sizeof(gyroOffset));
    Serial.println("Reusing accel + gyro calibration kept in RTC memory");
  } else {
    // Wait (up to ~5 s) for the module to lie flat and still, so that a
    // fresh calibration is meaningful.
    bool still = false;
    for (int t = 0; t < 50 && !still; t++) {
      uint8_t w[6];
      int16_t wax = 0, way = 0, waz = 0, wgx = 0, wgy = 0, wgz = 0;
      if (gy85Read(adxlAddr, 0x32, w, 6)) {
        wax = (int16_t)(w[1] << 8 | w[0]);
        way = (int16_t)(w[3] << 8 | w[2]);
        waz = (int16_t)(w[5] << 8 | w[4]);
      }
      if (gy85Read(itgAddr, 0x1D, w, 6)) {
        wgx = (int16_t)(w[0] << 8 | w[1]);
        wgy = (int16_t)(w[2] << 8 | w[3]);
        wgz = (int16_t)(w[4] << 8 | w[5]);
      }
      float gMag = sqrtf(wgx * wgx + wgy * wgy + wgz * wgz);
      still = fabsf(wax) < 40.0f && fabsf(way) < 40.0f && waz > 216 &&
              waz < 296 && gMag < 100.0f;
      if (!still) delay(100);
    }

    // Calibration kept in flash (NVS) - it survives resets and power cycles.
    Preferences prefs;
    bool nvsValid = false;
    int16_t nvsAccel[3] = {0, 0, 0};
    int16_t nvsGyro[3] = {0, 0, 0};
    if (prefs.begin(GY85_NVS_NAMESPACE, true)) {
      nvsValid = prefs.getBool("cal", false);
      if (nvsValid) {
        nvsAccel[0] = prefs.getShort("ax", 0);
        nvsAccel[1] = prefs.getShort("ay", 0);
        nvsAccel[2] = prefs.getShort("az", 0);
        nvsGyro[0] = prefs.getShort("gx", 0);
        nvsGyro[1] = prefs.getShort("gy", 0);
        nvsGyro[2] = prefs.getShort("gz", 0);
      }
      prefs.end();
    }

    if (!still && nvsValid) {
      // Not a good moment for calibrating - reuse the stored calibration.
      memcpy(accelOffset, nvsAccel, sizeof(accelOffset));
      memcpy(gyroOffset, nvsGyro, sizeof(gyroOffset));
      Serial.println(
          "Module not flat and still - reusing calibration from flash (NVS)");
    } else {
      Serial.println(
          "Calibrating accel + gyro - keep the module flat and still...");
      if (!still) {
        Serial.println(
            "Warning: the module seems to be moving - the calibration may be "
            "inaccurate");
      }

      const int samples = 200;
      int32_t aSum[3] = {0, 0, 0};
      int32_t gSum[3] = {0, 0, 0};
      for (int i = 0; i < samples; i++) {
        uint8_t raw[6];
        if (gy85Read(adxlAddr, 0x32, raw, 6)) {
          aSum[0] += (int16_t)(raw[1] << 8 | raw[0]);
          aSum[1] += (int16_t)(raw[3] << 8 | raw[2]);
          aSum[2] += (int16_t)(raw[5] << 8 | raw[4]);
        }
        if (gy85Read(itgAddr, 0x1D, raw, 6)) {
          gSum[0] += (int16_t)(raw[0] << 8 | raw[1]);
          gSum[1] += (int16_t)(raw[2] << 8 | raw[3]);
          gSum[2] += (int16_t)(raw[4] << 8 | raw[5]);
        }
        delay(10);
      }
      accelOffset[0] = aSum[0] / samples;
      accelOffset[1] = aSum[1] / samples;
      accelOffset[2] = aSum[2] / samples - 256;  // flat module reads +1g on Z
      gyroOffset[0] = gSum[0] / samples;
      gyroOffset[1] = gSum[1] / samples;
      gyroOffset[2] = gSum[2] / samples;

      if (still) {
        // Good conditions - persist the calibration so that it can survive
        // resets and power cycles.
        recalDone = true;  // do not re-calibrate again right after boot
        if (prefs.begin(GY85_NVS_NAMESPACE, false)) {
          prefs.putShort("ax", accelOffset[0]);
          prefs.putShort("ay", accelOffset[1]);
          prefs.putShort("az", accelOffset[2]);
          prefs.putShort("gx", gyroOffset[0]);
          prefs.putShort("gy", gyroOffset[1]);
          prefs.putShort("gz", gyroOffset[2]);
          prefs.putBool("cal", true);
          prefs.end();
          Serial.println("Calibration stored in flash (survives resets)");
        }
      }
    }

    Serial.printf("Offsets: accel=[%d,%d,%d] gyro=[%d,%d,%d]\r\n",
                  accelOffset[0], accelOffset[1], accelOffset[2],
                  gyroOffset[0], gyroOffset[1], gyroOffset[2]);

    memcpy(calAccelOffset, accelOffset, sizeof(calAccelOffset));
    memcpy(calGyroOffset, gyroOffset, sizeof(calGyroOffset));
    calStored = true;
  }

  Serial.printf("GY-85 ready: ADXL345 @0x%02X, ITG3205 @0x%02X\r\n", adxlAddr,
                itgAddr);
  return true;
}

void gy85Update(float dt) {
  uint8_t raw[6];
  int16_t ax16, ay16, az16, gx16, gy16, gz16;

  if (!gy85Read(adxlAddr, 0x32, raw, 6)) return;  // DATAX0..DATAZ1
  ax16 = (int16_t)(raw[1] << 8 | raw[0]);
  ay16 = (int16_t)(raw[3] << 8 | raw[2]);
  az16 = (int16_t)(raw[5] << 8 | raw[4]);

  if (!gy85Read(itgAddr, 0x1D, raw, 6)) return;  // GYRO_XOUT_H..GYRO_ZOUT_L
  gx16 = (int16_t)(raw[0] << 8 | raw[1]);
  gy16 = (int16_t)(raw[2] << 8 | raw[3]);
  gz16 = (int16_t)(raw[4] << 8 | raw[5]);

  gy85AccelRaw[0] = ax16;
  gy85AccelRaw[1] = ay16;
  gy85AccelRaw[2] = az16;
  gy85GyroRaw[0] = gx16;
  gy85GyroRaw[1] = gy16;
  gy85GyroRaw[2] = gz16;

  // Accelerometer -> g
  float ax = (ax16 - accelOffset[0]) / GY85_ACCEL_LSB_PER_G;
  float ay = (ay16 - accelOffset[1]) / GY85_ACCEL_LSB_PER_G;
  float az = (az16 - accelOffset[2]) / GY85_ACCEL_LSB_PER_G;

  // Gyro -> deg/s
  float gx = (gx16 - gyroOffset[0]) / GY85_GYRO_LSB_PER_DPS;
  float gy = (gy16 - gyroOffset[1]) / GY85_GYRO_LSB_PER_DPS;
  float gz = (gz16 - gyroOffset[2]) / GY85_GYRO_LSB_PER_DPS;

  // Movement detection for the automatic deep sleep
  float accelMag = sqrtf(ax * ax + ay * ay + az * az);
  float gyroMag = sqrtf(gx * gx + gy * gy + gz * gz);
  gy85Moving = (fabsf(accelMag - 1.0f) > GY85_MOTION_ACCEL_G) ||
               (gyroMag > GY85_MOTION_GYRO_DPS);

  // Flat (Z up) and still - same criteria as the calibration at boot.
  gy85FlatStill = !gy85Moving && fabsf(ax) < 0.15f && fabsf(ay) < 0.15f &&
                  az > 0.85f && az < 1.15f;

  /* --- roll & pitch from the accelerometer, fused with the gyro --- */
  float roll = atan2f(ay, az) * RAD2DEG_F;
  float pitch = atanf(-ax / sqrtf(ay * ay + az * az)) * RAD2DEG_F;

  if ((roll < -90 && gy85KalmanX.angle > 90) ||
      (roll > 90 && gy85KalmanX.angle < -90)) {
    gy85KalmanX.setAngle(roll);
  } else {
    roll = gy85KalmanX.getAngle(roll, gx, dt);
  }
  // pitch is restricted to +/-90 deg; invert the rate past that point
  float gyroYrate = gy;
  if (fabsf(roll) > 90) gyroYrate = -gyroYrate;
  pitch = gy85KalmanY.getAngle(pitch, gyroYrate, dt);

  /* --- yaw: tilt-compensated compass (Freescale AN4248), fused with
         the gyro Z rate --- */
  if (gy85Read(0x0D, 0x00, raw, 6)) {  // QMC5883L DATA X/Y/Z
    gy85MagRaw[0] = (int16_t)(raw[1] << 8 | raw[0]);
    gy85MagRaw[1] = (int16_t)(raw[3] << 8 | raw[2]);
    gy85MagRaw[2] = (int16_t)(raw[5] << 8 | raw[4]);
  }
  float magX = (gy85MagRaw[0] - magOffset[0]) * (float)GY85_MAG_SIGN_X;
  float magY = (gy85MagRaw[1] - magOffset[1]) * (float)GY85_MAG_SIGN_Y;
  float magZ = (gy85MagRaw[2] - magOffset[2]) * (float)GY85_MAG_SIGN_Z;

  float rollRad = roll / RAD2DEG_F;
  float pitchRad = pitch / RAD2DEG_F;
  float Bfy = magY * cosf(rollRad) - magZ * sinf(rollRad);
  float Bfx = magX * cosf(pitchRad) + magY * sinf(pitchRad) * sinf(rollRad) +
              magZ * sinf(pitchRad) * cosf(rollRad);
  float yaw = atan2f(-Bfy, Bfx) * RAD2DEG_F;

  if ((yaw < -90 && gy85KalmanZ.angle > 90) ||
      (yaw > 90 && gy85KalmanZ.angle < -90)) {
    gy85KalmanZ.setAngle(yaw);
  } else {
    yaw = gy85KalmanZ.getAngle(yaw, gz, dt);
  }

  gy85Roll = roll;
  gy85Pitch = pitch;
  gy85Yaw = yaw;
}

/* Manual calibration (web page "Calibrate" button): start a short sampling
   window right away if the module lies flat and still. Returns true when the
   window was started; the status reply is sent by the sketch. */
bool gy85StartManualRecal(void) {
  if (gy85Moving || !gy85FlatStill) return false;
  gy85RecalActive = true;
  recalManual = true;
  recalStartMs = millis();
  recalCount = 0;
  recalSumAccel[0] = recalSumAccel[1] = recalSumAccel[2] = 0;
  recalSumGyro[0] = recalSumGyro[1] = recalSumGyro[2] = 0;
  Serial.printf("Manual re-calibration - keep it still for %d s...\r\n",
                GY85_RECAL_MANUAL_MS / 1000);
  return true;
}

/* Refresh the zero offsets automatically: after the module has been lying
   flat and still for GY85_RECAL_STILL_MS, the raw accel/gyro data is
   averaged for GY85_RECAL_SAMPLE_MS and the new offsets are applied and
   stored (RTC + NVS). One re-calibration is done per "still session";
   movement re-arms it. Returns true when a new calibration was applied. */
bool gy85RecalTick(void) {
  unsigned long now = millis();

  if (gy85Moving) {
    recalDone = false;  // movement arms the next re-calibration
  }

  if (!gy85FlatStill) {
    if (gy85RecalActive) {
      gy85RecalActive = false;
      recalCount = 0;
      if (recalManual) {
        recalManual = false;
        gy85CalReply = 4;  // aborted
      }
      Serial.println("Re-calibration aborted - the module moved");
    }
    recalFlatSince = now;
    return false;
  }

  if (!gy85RecalActive) {
    if (GY85_RECAL_STILL_MS <= 0 || recalDone) return false;
    if (now - recalFlatSince >= (unsigned long)GY85_RECAL_STILL_MS) {
      gy85RecalActive = true;
      recalStartMs = now;
      recalCount = 0;
      recalSumAccel[0] = recalSumAccel[1] = recalSumAccel[2] = 0;
      recalSumGyro[0] = recalSumGyro[1] = recalSumGyro[2] = 0;
      Serial.printf(
          "Flat and still - re-calibrating zero offsets (keep it still for "
          "%d s)...\r\n",
          GY85_RECAL_SAMPLE_MS / 1000);
    }
    return false;
  }

  for (int i = 0; i < 3; i++) {
    recalSumAccel[i] += gy85AccelRaw[i];
    recalSumGyro[i] += gy85GyroRaw[i];
  }
  recalCount++;

  unsigned long windowMs = recalManual ? (unsigned long)GY85_RECAL_MANUAL_MS
                                       : (unsigned long)GY85_RECAL_SAMPLE_MS;
  if (now - recalStartMs < windowMs) return false;

  /* Window complete - apply the averaged offsets. */
  if (recalCount > 0) {
    accelOffset[0] = recalSumAccel[0] / (int32_t)recalCount;
    accelOffset[1] = recalSumAccel[1] / (int32_t)recalCount;
    accelOffset[2] = recalSumAccel[2] / (int32_t)recalCount - 256;
    gyroOffset[0] = recalSumGyro[0] / (int32_t)recalCount;
    gyroOffset[1] = recalSumGyro[1] / (int32_t)recalCount;
    gyroOffset[2] = recalSumGyro[2] / (int32_t)recalCount;

    memcpy(calAccelOffset, accelOffset, sizeof(calAccelOffset));
    memcpy(calGyroOffset, gyroOffset, sizeof(calGyroOffset));
    calStored = true;

    Preferences prefs;
    if (prefs.begin(GY85_NVS_NAMESPACE, false)) {
      prefs.putShort("ax", accelOffset[0]);
      prefs.putShort("ay", accelOffset[1]);
      prefs.putShort("az", accelOffset[2]);
      prefs.putShort("gx", gyroOffset[0]);
      prefs.putShort("gy", gyroOffset[1]);
      prefs.putShort("gz", gyroOffset[2]);
      prefs.putBool("cal", true);
      prefs.end();
    }

    Serial.printf(
        "Re-calibrated from %u samples: accel=[%d,%d,%d] gyro=[%d,%d,%d]\r\n",
        (unsigned)recalCount, accelOffset[0], accelOffset[1], accelOffset[2],
        gyroOffset[0], gyroOffset[1], gyroOffset[2]);
  }

  if (recalManual) {
    recalManual = false;
    gy85CalReply = 2;  // done
  }
  gy85RecalActive = false;
  recalDone = true;
  recalFlatSince = now;
  return true;
}

// Euler angles (degrees) -> quaternion (w-x-y-z) for the 3D cube
void gy85EulerToQuat(float rollDeg, float pitchDeg, float yawDeg, float *qw,
                     float *qx, float *qy, float *qz) {
  float hr = rollDeg * DEG2RAD_F / 2.0f;
  float hp = pitchDeg * DEG2RAD_F / 2.0f;
  float hy = yawDeg * DEG2RAD_F / 2.0f;
  float cr = cosf(hr), sr = sinf(hr);
  float cp = cosf(hp), sp = sinf(hp);
  float cy = cosf(hy), sy = sinf(hy);

  *qw = cr * cp * cy + sr * sp * sy;
  *qx = sr * cp * cy - cr * sp * sy;
  *qy = cr * sp * cy + sr * cp * sy;
  *qz = cr * cp * sy - sr * sp * cy;
}

/* Prepare the ADXL345 activity interrupt (AC-coupled, mapped to INT1) and
   the wake pin, so that motion wakes the board from deep sleep. The other
   sensors are paused as far as they allow it. */
void gy85PrepareWakeOnMotion(void) {
  /* Pause the gyro and the magnetometer; the ADXL345 must keep running. */
  gy85Write(itgAddr, 0x3E, 0x40);  // PWR_MGM: sleep
  gy85Write(0x0D, 0x09, 0x00);     // QMC5883L CTRL1: standby

  gy85Write(adxlAddr, 0x2C, 0x17);  // BW_RATE: low power, 12.5 Hz
  gy85Write(adxlAddr, 0x24,
            (uint8_t)(GY85_SLEEP_WAKE_ACT_G / 0.0625f));  // THRESH_ACT
  gy85Write(adxlAddr, 0x27, 0xF0);  // ACT_INACT_CTL: activity AC, X+Y+Z
  gy85Write(adxlAddr, 0x2F, 0x00);  // INT_MAP: activity -> INT1
  gy85Write(adxlAddr, 0x2E, 0x10);  // INT_ENABLE: activity only
  uint8_t intSrc;
  gy85Read(adxlAddr, 0x30, &intSrc, 1);  // clear any pending interrupt

  pinMode(GY85_PIN_WAKE, INPUT);
}
