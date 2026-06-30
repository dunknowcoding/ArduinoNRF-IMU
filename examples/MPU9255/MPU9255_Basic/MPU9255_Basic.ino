/*
  MPU9255_Basic - Minimal read loop for an MPU-9255 9-axis module.
*/
#include <MPU9255.h>

MPU9255 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!imu.begin()) {
    Serial.println("MPU-9255 not found - check wiring, power and AD0.");
    while (true) {
      delay(1000);
    }
  }

  Serial.print("MPU-9255 ready. WHO_AM_I = 0x");
  Serial.println(imu.whoAmI(), HEX);
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

  if (imu.hasMagnetometer()) {
    Serial.print("  M[uT] ");
    Serial.print(m.x, 1); Serial.print(", ");
    Serial.print(m.y, 1); Serial.print(", ");
    Serial.print(m.z, 1);
  }

  Serial.print("  T[C] ");
  Serial.println(imu.temperatureC(), 1);

  delay(100);
}
