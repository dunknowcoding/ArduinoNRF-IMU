/*
  GY9250_SPI - Read a GY-9250 over SPI instead of I2C.

  SPI is the fastest way to stream the IMU: the MPU-9250 tolerates ~1 MHz for
  configuration but up to ~20 MHz for reading sensor data, so we configure at a
  safe clock and then raise it just for the data bursts. (On SPI the AK8963
  magnetometer is not read - it lives on the chip's auxiliary I2C bus - so this
  is a 6-axis stream; use I2C if you need the compass.)

  Wiring (IMU silk -> ProMicro nRF52840 silk):
    SCLK -> SCK (D2)     SDI -> MOSI (D4)     SDO -> MISO (D3)
    NCS  -> CS  (D5/SS)  VCC -> 3V3           GND -> GND

  Open Serial Monitor at 115200 baud.
*/
#include <GY9250.h>
#include <SPI.h>

GY9250 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  // begin*() configures the chip at a safe 1 MHz internally.
  if (!imu.beginSPI(SPI, SS)) {   // SS = the CS pin (silk D5)
    Serial.println("GY-9250 not found on SPI - check SCLK/SDI/SDO/NCS wiring.");
    while (true) {
      delay(1000);
    }
  }
  Serial.print("GY-9250 on SPI. WHO_AM_I = 0x");
  Serial.println(imu.whoAmI(), HEX);

  imu.calibrateGyro();            // hold still (still at the safe 1 MHz)

  // Now go fast for data: 8 MHz is the ArduinoNRF SPI ceiling and reads cleanly.
  // (Raise this ONLY for reading; drop back to <=1 MHz before changing config.)
  imu.setBusClockHz(8000000);
  Serial.print("Streaming at ");
  Serial.print(imu.busClockHz() / 1000000);
  Serial.println(" MHz.");
}

void loop() {
  imu.update();
  Vec3 a = imu.accelG();
  Vec3 g = imu.gyroDps();

  Serial.print("A[g] ");
  Serial.print(a.x, 2); Serial.print(", ");
  Serial.print(a.y, 2); Serial.print(", ");
  Serial.print(a.z, 2);
  Serial.print("  G[dps] ");
  Serial.print(g.x, 1); Serial.print(", ");
  Serial.print(g.y, 1); Serial.print(", ");
  Serial.print(g.z, 1);
  Serial.print("  T[C] ");
  Serial.println(imu.temperatureC(), 1);

  delay(50);
}
