/*
  GY273_Basic - Minimal read loop for a GY-273 compass module.
*/
#include <GY273.h>

GY273 compass;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!compass.begin()) {
    Serial.println("GY-273 compass not found - check wiring and chip variant.");
    while (true) {
      delay(1000);
    }
  }

  Serial.print("Compass: ");
  Serial.println(compass.compassName());
}

void loop() {
  compass.update();
  Vec3 m = compass.magUT();

  Serial.print("M[uT] ");
  Serial.print(m.x, 1); Serial.print(", ");
  Serial.print(m.y, 1); Serial.print(", ");
  Serial.println(m.z, 1);

  delay(100);
}
