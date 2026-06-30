/* ICM45686_Basic - Minimal accelerometer and gyroscope example. */
#include <ICM45686.h>

ICM45686 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  if (!imu.begin()) {
    Serial.println("ICM-45686 not found.");
    while (true) delay(1000);
  }
}

void loop() {
  if (!imu.update()) return;
  Vec3 a = imu.accelG();
  Vec3 g = imu.gyroDps();
  Serial.print(a.x, 3); Serial.print(',');
  Serial.print(a.y, 3); Serial.print(',');
  Serial.print(a.z, 3); Serial.print(" | ");
  Serial.print(g.x, 2); Serial.print(',');
  Serial.print(g.y, 2); Serial.print(',');
  Serial.println(g.z, 2);
  delay(20);
}
