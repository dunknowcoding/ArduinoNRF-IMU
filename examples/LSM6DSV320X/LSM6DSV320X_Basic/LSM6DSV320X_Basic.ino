/* LSM6DSV320X_Basic - Minimal I2C accelerometer and gyroscope example. */
#include <LSM6DSV320X.h>

LSM6DSV320X imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!imu.begin()) {
    Serial.println("LSM6DSV320X not found.");
    while (true) {}
  }
}

void loop() {
  if (!imu.update()) return;
  const nimu::Vec3 a = imu.accelG();
  const nimu::Vec3 g = imu.gyroDps();
  Serial.print(a.x); Serial.print(',');
  Serial.print(a.y); Serial.print(',');
  Serial.print(a.z); Serial.print(" g  ");
  Serial.print(g.x); Serial.print(',');
  Serial.print(g.y); Serial.print(',');
  Serial.println(g.z);
  delay(20);
}
