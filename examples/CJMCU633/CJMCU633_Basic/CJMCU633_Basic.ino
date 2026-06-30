/* CJMCU633_Basic - Minimal I2C example. */
#include <CJMCU633.h>

CJMCU633 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!imu.begin()) {
    Serial.println("CJMCU-633 not found.");
    while (true) {}
  }
}

void loop() {
  if (imu.update()) Serial.println(imu.data().accel.x);
  delay(10);
}
