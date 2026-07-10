/*
for debugging on another laptop on saturday
added bno publisher to CMAKELISTS.txt
plug in imu
make sure .ino file is uploaded / serial monitor is showing imu is giving that output
joel@joel-B450-AORUS-ELITE:~/Documents/code/steelhead/src/steelhead_controls/steelhead_controls$ python3 bno085_imu_publisher.py
*/

// This file has been modified from the official adafruit demo files and publishes the raw
// orientation quaternion and acceleration in x, y, z direction to serial output.
//
// Line format (must match bno085_imu_publisher.py):
//   status,qx,qy,qz,qw,ax,ay,az
// The quaternion is sent in the BNO085's native i,j,k,real order, which maps
// directly onto the x,y,z,w fields of a ROS geometry_msgs/Quaternion.

#include <Arduino.h>

// This demo explores two reports (SH2_ARVR_STABILIZED_RV and SH2_GYRO_INTEGRATED_RV) both can be used to give
// quartenion and euler (yaw, pitch roll) angles.  Toggle the FAST_MODE define to see other report.
// Note sensorValue.status gives calibration accuracy (which improves over time)

#include <Adafruit_BNO08x.h>

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9

//if fastmode is on, uses SH2_GYRO_INTEGRATED_RV at 1000 Hz
//otherwise uses SH2_ARVR_STABILIZED_RV at 250 Hz with more accuracy
//#define FAST_MODE // uncomment this to enable fast mode.

// For SPI mode, we also need a RESET
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1

// Latest orientation quaternion in i,j,k,real (= x,y,z,w) order
float qx = 0, qy = 0, qz = 0, qw = 1;

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

void loop() {

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }

  float ax = 0, ay = 0, az = 0;

  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {

      case SH2_ARVR_STABILIZED_RV:
        qx = sensorValue.un.arvrStabilizedRV.i;
        qy = sensorValue.un.arvrStabilizedRV.j;
        qz = sensorValue.un.arvrStabilizedRV.k;
        qw = sensorValue.un.arvrStabilizedRV.real;
        break;

      case SH2_GYRO_INTEGRATED_RV:
        qx = sensorValue.un.gyroIntegratedRV.i;
        qy = sensorValue.un.gyroIntegratedRV.j;
        qz = sensorValue.un.gyroIntegratedRV.k;
        qw = sensorValue.un.gyroIntegratedRV.real;
        break;

      case SH2_GAME_ROTATION_VECTOR:
        qx = sensorValue.un.gameRotationVector.i;
        qy = sensorValue.un.gameRotationVector.j;
        qz = sensorValue.un.gameRotationVector.k;
        qw = sensorValue.un.gameRotationVector.real;
        break;

      case SH2_LINEAR_ACCELERATION:
        ax = sensorValue.un.accelerometer.x;
        ay = sensorValue.un.accelerometer.y;
        az = sensorValue.un.accelerometer.z;
        break;

    }

    float status_f = (float) sensorValue.status;

    // Quaternion components live in [-1, 1]; the default 2 decimal places
    // would quantize orientation to ~1 degree, so print 6
    Serial.print(status_f);
    Serial.print(",");
    Serial.print(qx, 6);
    Serial.print(",");
    Serial.print(qy, 6);
    Serial.print(",");
    Serial.print(qz, 6);
    Serial.print(",");
    Serial.print(qw, 6);
    Serial.print(",");
    Serial.print(ax);
    Serial.print(",");
    Serial.print(ay);
    Serial.print(",");
    Serial.println(az);
  }
}
