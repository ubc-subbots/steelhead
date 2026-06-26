#include <Wire.h>
#include "MS5837.h"

#define I2C0_SDA 28
#define I2C0_SCL 29
#define SENSOR_MODEL MS5837::MS5837_02BA
#define FLUID_DENSITY 997

MS5837 sensor;

void setup() {
  Serial1.begin(115200);
  Wire.setSDA(I2C0_SDA);
  Wire.setSCL(I2C0_SCL);
  Wire.begin();
  Serial1.println("MS5837 depth sensor init...");
  while (!sensor.init()) {
    Serial1.println("MS5837 init failed! Check wiring (SDA/SCL/3.3V/GND), 0x76.");
    delay(2000);
  }
  sensor.setModel(SENSOR_MODEL);
  sensor.setFluidDensity(FLUID_DENSITY);
  Serial1.println("MS5837 found!");
}

void loop() {
  sensor.read();
  Serial1.print(sensor.depth(), 3);
  Serial1.print(",");
  Serial1.print(sensor.pressure(), 1);
  Serial1.print(",");
  Serial1.println(sensor.temperature(), 2);
  delay(50);
}