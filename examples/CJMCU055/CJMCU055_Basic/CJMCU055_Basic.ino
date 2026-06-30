/* CJMCU055_Basic - Minimal I2C example for the CJMCU-055 board. */
#include <CJMCU055.h>

CJMCU055 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!imu.begin()) {
    Serial.println("CJMCU-055 not found.");
    while (true) {}
  }
}

void loop() {
  if (imu.update()) {
    const IMUData& d = imu.data();
    Serial.print(d.accel.x);
    Serial.print(',');
    Serial.print(d.gyro.x);
    Serial.print(',');
    Serial.println(d.mag.x);
  }
  delay(10);
}
