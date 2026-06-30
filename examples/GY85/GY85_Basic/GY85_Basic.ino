/*
  GY85_Basic - Minimal read loop for a GY-85 marketplace module.
*/
#include <GY85.h>

GY85 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!imu.begin()) {
    Serial.println("GY-85 accel/gyro not found - check wiring and power.");
    while (true) {
      delay(1000);
    }
  }
}

void loop() {
  imu.update();

  Vec3 a = imu.accelG();
  Vec3 g = imu.gyroDps();
  Vec3 m = imu.magUT();

  Serial.print("A[g] ");
  Serial.print(a.x, 2); Serial.print(", ");
  Serial.print(a.y, 2); Serial.print(", ");
  Serial.print(a.z, 2);

  Serial.print("  G[dps] ");
  Serial.print(g.x, 1); Serial.print(", ");
  Serial.print(g.y, 1); Serial.print(", ");
  Serial.print(g.z, 1);

  Serial.print("  M[uT] ");
  Serial.print(m.x, 1); Serial.print(", ");
  Serial.print(m.y, 1); Serial.print(", ");
  Serial.println(m.z, 1);

  delay(100);
}
