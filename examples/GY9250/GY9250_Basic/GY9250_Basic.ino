/*
  GY9250_Basic - The simplest way to read a GY-9250 (9-DOF MPU-9250 board).

  Prints acceleration (g), rotation rate (deg/s), magnetic field (uT) and
  temperature (C) a few times per second. GY9250 is the same device as the
  MPU9250 driver under the breakout's name - use whichever you prefer.

  Wiring (ArduinoNRF ProMicro nRF52840 default I2C):
    SDA -> SDA (silk D6)    SCL -> SCL (silk D7)    VCC -> 3V3    GND -> GND

  Open Serial Monitor at 115200 baud.
*/
#include <GY9250.h>

GY9250 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  // Start the sensor on the default I2C bus with sensible defaults.
  if (!imu.begin()) {
    Serial.println("GY-9250 not found - check wiring and power.");
    while (true) {
      delay(1000);
    }
  }

  Serial.print("GY-9250 ready. WHO_AM_I = 0x");
  Serial.println(imu.whoAmI(), HEX);
  Serial.print("Magnetometer: ");
  Serial.println(imu.hasMagnetometer() ? "yes" : "no");

  // Remove gyro drift: hold the board completely still for about one second.
  Serial.println("Calibrating gyro - keep the board still...");
  imu.calibrateGyro();
  Serial.println("Done.");
}

void loop() {
  imu.update();  // grab one fresh sample

  Vec3 a = imu.accelG();
  Vec3 g = imu.gyroDps();
  Vec3 m = imu.magUT();

  Serial.print("A[g] ");
  Serial.print(a.x, 2); Serial.print(", ");
  Serial.print(a.y, 2); Serial.print(", ");
  Serial.print(a.z, 2);

  Serial.print("  G[dps] ");
  Serial.print(g.x, 1); Serial.print(", ");
  Serial.print(g.y, 1); Serial.print(", ");
  Serial.print(g.z, 1);

  Serial.print("  M[uT] ");
  Serial.print(m.x, 1); Serial.print(", ");
  Serial.print(m.y, 1); Serial.print(", ");
  Serial.print(m.z, 1);

  Serial.print("  T[C] ");
  Serial.println(imu.temperatureC(), 1);

  delay(100);
}
