/*
  LSM6DS3_Basic - Minimal read loop for an LSM6DS3 6-axis IMU.
*/
#include <LSM6DS3.h>

LSM6DS3 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!imu.begin()) {
    Serial.println("LSM6DS3 not found - check wiring, power and SA0.");
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
