/*
  MMA8452Q_Basic - Minimal read loop for an MMA8452Q accelerometer.
*/
#include <MMA8452Q.h>

MMA8452Q accel;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!accel.begin()) {
    Serial.println("MMA8452Q not found - check wiring, power and SA0.");
    while (true) {
      delay(1000);
    }
  }

  Serial.print("MMA8452Q ready. WHO_AM_I = 0x");
  Serial.println(accel.whoAmI(), HEX);
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
