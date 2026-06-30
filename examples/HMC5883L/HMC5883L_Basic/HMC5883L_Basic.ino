/*
  HMC5883L_Basic - Minimal read loop for an HMC5883L magnetometer.
*/
#include <HMC5883L.h>

HMC5883L mag;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!mag.begin()) {
    Serial.println("HMC5883L not found - check wiring and power.");
    while (true) {
      delay(1000);
    }
  }
}

void loop() {
  mag.update();

  Vec3 m = mag.magUT();
  Serial.print("M[uT] ");
  Serial.print(m.x, 1); Serial.print(", ");
  Serial.print(m.y, 1); Serial.print(", ");
  Serial.println(m.z, 1);

  delay(100);
}
