/* LSM303_L3GD20_Basic - Minimal 9DOF clone-board example. */
#include <LSM303_L3GD20.h>

LSM303_L3GD20 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  if (!imu.begin()) {
    Serial.println("LSM303DLHC + L3GD20 board not found.");
    while (true) delay(1000);
  }
}

void loop() {
  if (!imu.update()) return;
  Vec3 a = imu.accelG(); Vec3 g = imu.gyroDps(); Vec3 m = imu.magUT();
  Serial.print(a.x, 2); Serial.print(','); Serial.print(a.y, 2);
  Serial.print(','); Serial.print(a.z, 2); Serial.print(" | ");
  Serial.print(g.x, 1); Serial.print(','); Serial.print(g.y, 1);
  Serial.print(','); Serial.print(g.z, 1); Serial.print(" | ");
  Serial.print(m.x, 1); Serial.print(','); Serial.print(m.y, 1);
  Serial.print(','); Serial.println(m.z, 1);
}
