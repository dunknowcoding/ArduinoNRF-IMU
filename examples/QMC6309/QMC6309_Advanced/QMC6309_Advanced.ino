/* QMC6309_Advanced - Identity, reset and data-ready polling. */
#include <QMC6309.h>

QMC6309 compass;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  if (!compass.begin(0x3E, Wire)) {
    Serial.println("QMC6309 not found.");
    while (true) delay(1000);
  }
  Serial.print("WHO_AM_I=0x");
  Serial.println(compass.whoAmI(), HEX);
}

void loop() {
  if (!compass.dataReady() || !compass.update()) return;
  Vec3 m = compass.magUT();
  Serial.print("mag uT ");
  Serial.print(m.x, 2); Serial.print(',');
  Serial.print(m.y, 2); Serial.print(',');
  Serial.println(m.z, 2);
}
