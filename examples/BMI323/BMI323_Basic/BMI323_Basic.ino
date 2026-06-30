/* BMI323_Basic - Minimal I2C read loop for a Bosch BMI323. */
#include <BMI323.h>

BMI323 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  if (!imu.begin()) {
    Serial.println("BMI323 not found - check power, SDA/SCL, CSB and SDO.");
    while (true) delay(1000);
  }
}

void loop() {
  if (!imu.update()) return;
  Vec3 a = imu.accelG();
  Vec3 g = imu.gyroDps();
  Serial.print("A[g] ");
  Serial.print(a.x, 2); Serial.print(", ");
  Serial.print(a.y, 2); Serial.print(", ");
  Serial.print(a.z, 2);
  Serial.print("  G[dps] ");
  Serial.print(g.x, 1); Serial.print(", ");
  Serial.print(g.y, 1); Serial.print(", ");
  Serial.println(g.z, 1);
  delay(100);
}
