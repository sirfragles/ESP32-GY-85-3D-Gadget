/* ======================= gy85_imu.h =======================
   GY-85 IMU module for the ESP32-GY-85-3D-Gadget sketch.

   Driver for the GY-85 board (ADXL345 accelerometer + ITG3205 gyroscope +
   QMC5883L magnetometer) with a 9-DoF sensor fusion:
     - Kalman-filtered roll/pitch (accelerometer + gyro rate)
     - tilt-compensated compass yaw (Freescale AN4248 + gyro Z rate) - the
       same scheme as the nopnop2002/esp-idf-gy85 reference demo
     - zero-offset calibration: at boot, automatically while idle, and on
       request from the web page; stored in RTC memory and flash (NVS)
     - motion detection (for the automatic deep sleep) and ADXL345
       activity-interrupt setup for the deep-sleep wake-up

   All settings below can also be overridden with #define in the sketch,
   before including this header.
   =========================================================== */

#pragma once
#include <Arduino.h>

/* ------------------------------- pins ------------------------------- */
#ifndef GY85_PIN_SDA
#define GY85_PIN_SDA 6  // XIAO D4 (GPIO6)
#endif
#ifndef GY85_PIN_SCL
#define GY85_PIN_SCL 7  // XIAO D5 (GPIO7)
#endif
#ifndef GY85_PIN_WAKE
#define GY85_PIN_WAKE 5  // XIAO D3 (GPIO5) - ADXL345 INT1 ("A_INT1")
#endif

/* ---------------------- magnetometer settings ----------------------- */
// Hard-iron offsets (raw counts, 0 = uncalibrated). To find them: rotate
// the module through all orientations and note the min/max of each axis
// from the mag=[...] values on the serial monitor; the offset is
// (min + max) / 2.
#ifndef GY85_MAG_OFFSET_X
#define GY85_MAG_OFFSET_X 0
#endif
#ifndef GY85_MAG_OFFSET_Y
#define GY85_MAG_OFFSET_Y 0
#endif
#ifndef GY85_MAG_OFFSET_Z
#define GY85_MAG_OFFSET_Z 0
#endif

// Some GY-85 boards mount the magnetometer rotated - flip these if the yaw
// moves the wrong way or is off by ~180 deg.
#ifndef GY85_MAG_SIGN_X
#define GY85_MAG_SIGN_X (-1)
#endif
#ifndef GY85_MAG_SIGN_Y
#define GY85_MAG_SIGN_Y (1)
#endif
#ifndef GY85_MAG_SIGN_Z
#define GY85_MAG_SIGN_Z (-1)
#endif

/* --------------------- motion / sleep thresholds -------------------- */
// What counts as "no movement" while awake: acceleration magnitude away
// from 1 g [g] and gyro rate [deg/s].
#ifndef GY85_MOTION_ACCEL_G
#define GY85_MOTION_ACCEL_G 0.05f
#endif
#ifndef GY85_MOTION_GYRO_DPS
#define GY85_MOTION_GYRO_DPS 3.0f
#endif

// ADXL345 activity threshold used for the wake-up interrupt, in g
// (register resolution is 62.5 mg/LSB).
#ifndef GY85_SLEEP_WAKE_ACT_G
#define GY85_SLEEP_WAKE_ACT_G 0.5f
#endif

/* --------------------- automatic re-calibration --------------------- */
// When the module has been lying flat and still for GY85_RECAL_STILL_MS,
// the zero offsets are measured again and stored. The raw data is averaged
// for GY85_RECAL_SAMPLE_MS before the offsets are refreshed. 0 disables
// the automatic re-calibration.
#ifndef GY85_RECAL_STILL_MS
#define GY85_RECAL_STILL_MS 30000
#endif
#ifndef GY85_RECAL_SAMPLE_MS
#define GY85_RECAL_SAMPLE_MS 30000
#endif

// Sampling window for a manual calibration requested from the web page
// ("Calibrate" button).
#ifndef GY85_RECAL_MANUAL_MS
#define GY85_RECAL_MANUAL_MS 5000
#endif

/* --------------------------- NVS storage ---------------------------- */
// Namespace where a validated zero-offset calibration is persisted so that
// it survives resets and power cycles.
#ifndef GY85_NVS_NAMESPACE
#define GY85_NVS_NAMESPACE "gy85"
#endif

/* ------------------------------ API --------------------------------- */
// I2C + sensors + zero-offset calibration (call from a task).
bool gy85Init(void);

// Read the sensors and update the fused angles (call periodically).
void gy85Update(float dt);

// Automatic re-calibration tick - call every loop iteration; returns true
// when a new calibration was applied.
bool gy85RecalTick(void);

// Manual calibration request (accepted only when flat and still) - returns
// true when the sampling window was started.
bool gy85StartManualRecal(void);

// Euler angles (degrees) -> quaternion (w-x-y-z) for the 3D cube
void gy85EulerToQuat(float rollDeg, float pitchDeg, float yawDeg, float *qw,
                     float *qx, float *qy, float *qz);

// Prepare the ADXL345 activity interrupt + wake pin. Call just before
// entering deep sleep; the pin then wakes the board on motion.
void gy85PrepareWakeOnMotion(void);

/* ------------------------------ state ------------------------------- */
extern float gy85Roll, gy85Pitch, gy85Yaw;  // fused angles (deg)
extern int16_t gy85MagRaw[3];                // last raw magnetometer values
extern bool gy85Moving;       // module is being moved (sleep logic)
extern bool gy85RecalActive;  // a re-calibration window is running
extern volatile bool gy85CalRequest;  // set to request a manual calibration
extern volatile uint8_t gy85CalReply; // 0 = none, 1 = started, 2 = done,
                                      // 3 = refused, 4 = aborted
