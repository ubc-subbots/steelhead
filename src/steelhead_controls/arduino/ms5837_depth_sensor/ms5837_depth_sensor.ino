#include <Wire.h>
#include "MS5837.h"


MS5837 sensor;


void setup() {
  Serial.begin(115200);
  Wire.begin();


  sensor.setModel(MS5837::MS5837_02BA);  // # Change to MS5837::MS5837_02BA if using Bar02
  sensor.init();
  sensor.setFluidDensity(997); // kg/m^3 (997 freshwater, 1029 seawater)


  Serial.println("Starting");
}


void loop() {
  sensor.read();
  Serial.print(sensor.depth());
  Serial.print(",");
  Serial.print(sensor.pressure());
  Serial.print(",");
  Serial.println(sensor.temperature());
  delay(50);
}