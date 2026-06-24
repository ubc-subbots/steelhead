/*
 * MS5837 (Blue Robotics Bar02 / Bar30) depth sensor reader for the Radxa X4's
 * ONBOARD RP2040 (the 40-pin GPIO header).
 *
 * This mirrors the BNO085 setup (../bno085_serial_output_parser): the MCU reads
 * the sensor over I2C and streams one CSV line per sample over USB serial. The
 * host node (steelhead_controls/depth_sensor_serial_publisher.py) parses it and
 * publishes steelhead_interfaces/PressureSensor on drivers/depth_sensor.
 *
 * Output line:  depth,pressure,temperature\n
 *   depth        meters (computed on-chip, see fluid density below)
 *   pressure     mbar
 *   temperature  degrees C
 *
 * Toolchain: Arduino IDE with the "Raspberry Pi Pico/RP2040" core
 *   (earlephilhower). Select board "Raspberry Pi Pico". Put the onboard RP2040
 *   in BOOTSEL (button by the header) so it mounts as RPI-RP2, then Upload.
 *
 * Library: "BlueRobotics MS5837 Library" (install via Library Manager).
 */

#include <Wire.h>
#include "MS5837.h"

#define I2C0_SDA 28
#define I2C0_SCL 29
\
#define SENSOR_MODEL MS5837::MS5837_02BA // MS5837_30BA 

// 997 kg/m^3 freshwater (pool), 1029 for saltwater.
#define FLUID_DENSITY 997

MS5837 sensor;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);  // wait for USB serial

  Wire.setSDA(I2C0_SDA);
  Wire.setSCL(I2C0_SCL);
  Wire.begin();

  Serial.println("MS5837 depth sensor init...");
  while (!sensor.init()) {
    Serial.println("MS5837 init failed! Check wiring (SDA/SCL/3.3V/GND), 0x76.");
    delay(2000);
  }

  sensor.setModel(SENSOR_MODEL);
  sensor.setFluidDensity(FLUID_DENSITY);
  Serial.println("MS5837 found!");
}

void loop() {
  sensor.read();

  Serial.print(sensor.depth(), 3);        // meters
  Serial.print(",");
  Serial.print(sensor.pressure(), 1);     // mbar
  Serial.print(",");
  Serial.println(sensor.temperature(), 2); // deg C
}
