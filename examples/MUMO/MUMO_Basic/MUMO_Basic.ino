/* MUMO_Basic - Minimal ICM-45686 + QMC6309 example. */
#include <MUMO.h>

MUMO imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  if (!imu.begin()) {
    Serial.println("MUMO not found.");
    while (true) delay(1000);
  }
}

void loop() {
  if (!imu.update()) return;
  Vec3 a = imu.accelG();
  Vec3 g = imu.gyroDps();
  Vec3 m = imu.magUT();
  Serial.print(a.x, 3); Serial.print(','); Serial.print(a.y, 3);
  Serial.print(','); Serial.print(a.z, 3); Serial.print(" | ");
  Serial.print(g.x, 2); Serial.print(','); Serial.print(g.y, 2);
  Serial.print(','); Serial.print(g.z, 2); Serial.print(" | ");
  Serial.print(m.x, 2); Serial.print(','); Serial.print(m.y, 2);
  Serial.print(','); Serial.println(m.z, 2);
  delay(20);
}
