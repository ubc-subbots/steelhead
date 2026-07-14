// This file has been modified from the official adafruit demo files and simply publishes yaw, pitch, roll, and acceleration in x, y, z direction to serial output
// Meant to be connected to the sbc via a teensy through serial connection.

#include <Arduino.h>

// This demo explores two reports (SH2_ARVR_STABILIZED_RV and SH2_GYRO_INTEGRATED_RV) both can be used to give 
// quartenion and euler (yaw, pitch roll) angles.  Toggle the FAST_MODE define to see other report.  
// Note sensorValue.status gives calibration accuracy (which improves over time)

#include <Adafruit_BNO08x.h>

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9

//if fastmode is on, uses SH2_GYRO_INTEGRATED_RV at 1000 Hz 
//otherwise uses SH2_ARVR_STABILIZED_RV at 250 Hz with more accuracy
//#define FAST_MODE // uncomment this to enable fast mode. 

// For SPI mode, we also need a RESET 
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

struct quat_t {
  float w;
  float x;
  float y;
  float z;
};

quat_t multiplyQuat(quat_t a, quat_t b) {
  quat_t r;
  r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
  r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
  r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
  r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
  return r;
}

// This is the constant rotation that is applied to calculations for if the imu is not mounted in the correct way onto the robot (based on the markings on the bno)
// CHANGE ME AND ACCLERATION IF MOUNTED ORIENTATION CHANGES
quat_t q_mount = {0, 1, 0, 0};

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

#ifdef FAST_MODE
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  sh2_SensorId_t reportType = SH2_GAME_ROTATION_VECTOR;
  long reportIntervalUs = 5000;
#endif
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
  if (! bno08x.enableReport(SH2_LINEAR_ACCELERATION, report_interval)) {
    Serial.println("Could not enable accelerometer");
  }
}

void setup(void) {

  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");


  setReports(reportType, reportIntervalUs);

  Serial.println("Reading events");
  delay(100);
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    float roll_arg = 2.0 * (qj * qk + qr * qi) / (sqi + sqj + sqk + sqr);
    if (roll_arg > 1.0) roll_arg = 1.0;
    if (roll_arg < -1.0) roll_arg = -1.0;

    // roll is the asin (middle) axis -> clamped to +/-90 deg; this is our "axis of error"
    // that the controller keeps near level, so it never approaches gimbal lock.
    // yaw and pitch use atan2 -> full +/-180 deg range, so they track 360 rotation unclamped.
    ypr->yaw   = atan2(2.0 * (qr * qk - qi * qj), sqr - sqi + sqj - sqk);
    ypr->roll  = asin(roll_arg);
    ypr->pitch = atan2(2.0 * (qr * qj - qi * qk), sqr - sqi - sqj + sqk);

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void loop() {

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }

  float ax = 0, ay = 0, az = 0; 
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {

      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        break;

      case SH2_GYRO_INTEGRATED_RV:
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;

      case SH2_GAME_ROTATION_VECTOR: {
        quat_t q_sensor = {
          sensorValue.un.gameRotationVector.real,
          sensorValue.un.gameRotationVector.i,
          sensorValue.un.gameRotationVector.j,
          sensorValue.un.gameRotationVector.k
        };
        quat_t q_body = multiplyQuat(q_sensor, q_mount);
        quaternionToEuler(q_body.w, q_body.x, q_body.y, q_body.z, &ypr, true);
        break;
      }
      case SH2_LINEAR_ACCELERATION: {
        float sx = sensorValue.un.accelerometer.x;
        float sy = sensorValue.un.accelerometer.y;
        float sz = sensorValue.un.accelerometer.z;

        // CHANGE ME IF ORIENTATION CHANGES
        // 180deg roll about local X: x unchanged, y and z flip sign.
        ax = sx;
        ay = -sy;
        az = -sz;
        break;
      }
    }

    float yaw_deg = ypr.yaw - 90.0;
    if (yaw_deg > 180.0) yaw_deg -= 360.0;
    if (yaw_deg < -180.0) yaw_deg += 360.0;
    float pitch_deg = ypr.pitch;
    float roll_deg  = ypr.roll;
    float status_f = (float) sensorValue.status; 

    Serial.print(status_f);
    Serial.print(",");
    Serial.print(yaw_deg);
    Serial.print(",");
    Serial.print(pitch_deg);
    Serial.print(",");
    Serial.print(roll_deg);
    Serial.print(",");
    Serial.print(ax);
    Serial.print(",");
    Serial.print(ay);
    Serial.print(",");
    Serial.println(az);
  }
}