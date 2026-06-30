/* LSM6DS3TRC_Basic - Minimal I2C example. */
#include <LSM6DS3TRC.h>

LSM6DS3TRC imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!imu.begin()) {
    Serial.println("LSM6DS3TR-C not found.");
    while (true) {}
  }
}

void loop() {
  if (imu.update()) Serial.println(imu.data().accel.x);
  delay(10);
}
