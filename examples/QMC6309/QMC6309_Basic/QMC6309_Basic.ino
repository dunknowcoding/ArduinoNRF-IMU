/* QMC6309_Basic - Minimal direct-I2C magnetometer example. */
#include <QMC6309.h>

QMC6309 compass;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  if (!compass.begin()) {
    Serial.println("QMC6309 not found.");
    while (true) delay(1000);
  }
}

void loop() {
  if (!compass.update()) return;
  Vec3 m = compass.magUT();
  Serial.print(m.x, 2); Serial.print(',');
  Serial.print(m.y, 2); Serial.print(',');
  Serial.println(m.z, 2);
  delay(20);
}
