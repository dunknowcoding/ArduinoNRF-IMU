/* QMC5883L_Advanced - DRDY and overflow-aware reads. */
#include <QMC5883L.h>

QMC5883L compass;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  if (!compass.begin()) {
    Serial.println("QMC5883L not found.");
    while (true) delay(1000);
  }
}

void loop() {
  if (!compass.dataReady() || compass.dataOverflow()) return;
  if (!compass.update()) return;
  Vec3 m = compass.magUT();
  Serial.print(m.x, 2); Serial.print(',');
  Serial.print(m.y, 2); Serial.print(','); Serial.println(m.z, 2);
}
