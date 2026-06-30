/* LSM6DSV_Basic - Minimal accelerometer and gyroscope example. */
#include <LSM6DSV.h>

LSM6DSV imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!imu.begin()) {
    Serial.println("LSM6DSV not found.");
    while (true) {}
  }
}

void loop() {
  if (imu.update()) {
    const IMUData& d = imu.data();
    Serial.print(d.accel.x);
    Serial.print(',');
    Serial.println(d.gyro.x);
  }
  delay(10);
}
