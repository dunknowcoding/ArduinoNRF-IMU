/* GYBNO085_Basic - Minimal I2C loop for the ten-pad purple GY board. */
#include <GYBNO085.h>

GYBNO085 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  if (!imu.begin()) {
    Serial.println("GY-BNO085 not found - PS1/PS0 must both be LOW for I2C.");
    while (true) delay(1000);
  }
}

void loop() {
  if (!imu.update()) return;
  Vec3 a = imu.accelG();
  Vec3 g = imu.gyroDps();
  Serial.print("A[g] ");
  Serial.print(a.x, 2); Serial.print(',');
  Serial.print(a.y, 2); Serial.print(','); Serial.print(a.z, 2);
  Serial.print(" G[dps] ");
  Serial.print(g.x, 1); Serial.print(',');
  Serial.print(g.y, 1); Serial.print(','); Serial.println(g.z, 1);
}
