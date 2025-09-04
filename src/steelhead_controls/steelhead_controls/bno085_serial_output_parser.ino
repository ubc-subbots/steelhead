/*
for debugging on another laptop on saturday
added bno publisher to CMAKELISTS.txt
plug in imu
make sure .ino file is uploaded / serial monitor is showing imu is giving that output
joel@joel-B450-AORUS-ELITE:~/Documents/code/steelhead/src/steelhead_controls/steelhead_controls$ python3 bno085_imu_publisher.py
*/

// This file has been modified from the official adafruit demo files and simply publishes yaw, pitch, roll, and acceleration in x, y, z direction to serial output

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

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
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

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

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
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
      case SH2_LINEAR_ACCELERATION:
        // Serial.print("\t\t\tAccelerometer - x: ");
        // Serial.print(sensorValue.un.accelerometer.x);
        // Serial.print(" y: ");
        // Serial.print(sensorValue.un.accelerometer.y);
        // Serial.print(" z: ");
        // Serial.println(sensorValue.un.accelerometer.z);
        // Serial.println("\n");
        ax = sensorValue.un.accelerometer.x;
        ay = sensorValue.un.accelerometer.y;
        az = sensorValue.un.accelerometer.z;
        break;
    }
    // static long last = 0;
    // long now = micros();
    // Serial.print(now - last);             Serial.print("\t"); // time elapsed since last reading
    // last = now; 
    // Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
    // Serial.print(ypr.yaw);                Serial.print("\t"); //yaw
    // Serial.print(ypr.pitch);              Serial.print("\t"); //pitch
    // Serial.println(ypr.roll); //roll

    // Print everything on ONE line, comma-separated:
    // Format: status,yaw,pitch,roll,ax,ay,az
    float yaw_deg   = ypr.yaw;
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