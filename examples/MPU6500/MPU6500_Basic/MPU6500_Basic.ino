/*
  MPU6500_Basic - Read a 6-axis MPU-6500 (accelerometer + gyroscope).

  Use this driver for a bare MPU-6500, or for any board whose "MPU-9250" turns
  out to be a 6-axis MPU-6500 (WHO_AM_I 0x70, no magnetometer) - common on GY-91
  clones. For a board with a real magnetometer use <MPU9250.h>; for the whole
  GY-91 (IMU + barometer) use <GY91.h>.

  Wiring (ArduinoNRF ProMicro nRF52840 default I2C):
    SDA -> SDA (silk D6)    SCL -> SCL (silk D7)    VCC -> 3V3    GND -> GND

  Open Serial Monitor at 115200 baud.
*/
#include <MPU6500.h>

MPU6500 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!imu.begin()) {
    Serial.println("MPU-6500 not found - check wiring and power.");
    while (true) {
      delay(1000);
    }
  }

  Serial.print("MPU-6500 ready. WHO_AM_I = 0x");
  Serial.println(imu.whoAmI(), HEX);

  // Remove gyro drift: hold the board completely still for about a second.
  Serial.println("Calibrating gyro - keep the board still...");
  imu.calibrateGyro();
  Serial.println("Done.");
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

  delay(100);
}
