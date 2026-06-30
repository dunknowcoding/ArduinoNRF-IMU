/* GY601N1_Basic - Auto-detect the populated IMU over I2C. */
#include <GY601N1.h>

GY601N1 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!imu.begin()) {
    Serial.println("GY-601N1 IMU not found.");
    while (true) {}
  }
}

void loop() {
  if (imu.update()) {
    Serial.println(imu.accelG().x);
  }
  delay(10);
}
