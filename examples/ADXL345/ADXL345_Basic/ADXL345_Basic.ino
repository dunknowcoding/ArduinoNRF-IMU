/*
  ADXL345_Basic - Minimal read loop for an ADXL345 accelerometer.
*/
#include <ADXL345.h>

ADXL345 accel;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!accel.begin()) {
    Serial.println("ADXL345 not found - check wiring, power and SDO.");
    while (true) {
      delay(1000);
    }
  }
}

void loop() {
  accel.update();
  Vec3 a = accel.accelG();

  Serial.print("A[g] ");
  Serial.print(a.x, 2); Serial.print(", ");
  Serial.print(a.y, 2); Serial.print(", ");
  Serial.println(a.z, 2);

  delay(100);
}
