/*
  LSM6DSOX_Basic - Minimal read loop for an LSM6DSOX 6-axis IMU.
*/
#include <LSM6DSOX.h>

LSM6DSOX imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!imu.begin()) {
    Serial.println("LSM6DSOX not found - check wiring, power and SA0/SDO.");
    while (true) {
      delay(1000);
    }
  }
}

void loop() {
  imu.update();

  Vec3 a = imu.accelG();
  Vec3 g = imu.gyroDps();

  Serial.print("A[g] ");
  Serial.print(a.x, 2); Serial.print(", ");
  Serial.print(a.y, 2); Serial.print(", ");
  Serial.print(a.z, 2);

  Serial.print("  G[dps] ");
  Serial.print(g.x, 1); Serial.print(", ");
  Serial.print(g.y, 1); Serial.print(", ");
  Serial.print(g.z, 1);

  Serial.print("  T[C] ");
  Serial.println(imu.temperatureC(), 1);

  delay(100);
}
