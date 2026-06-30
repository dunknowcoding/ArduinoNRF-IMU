/* HMC5883L_Advanced - DRDY status, lock detection and gain. */
#include <HMC5883L.h>

HMC5883L compass;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  if (!compass.begin()) {
    Serial.println("HMC5883L not found.");
    while (true) delay(1000);
  }
  compass.setGainGauss(1.9f);
}

void loop() {
  if (!compass.dataReady() || compass.dataLocked()) return;
  if (!compass.update()) return;
  Vec3 m = compass.magUT();
  Serial.print(m.x, 2); Serial.print(',');
  Serial.print(m.y, 2); Serial.print(','); Serial.println(m.z, 2);
}
