/*
  GY45_Basic - Minimal read loop for a GY-45 MMA8452Q accelerometer module.
*/
#include <GY45.h>

GY45 accel;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!accel.begin()) {
    Serial.println("GY-45/MMA8452Q not found - check wiring, power and SA0.");
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
