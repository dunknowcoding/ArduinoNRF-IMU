/*
  L3G4200D_Basic - Minimal read loop for an L3G4200D gyroscope.
*/
#include <L3G4200D.h>

L3G4200D gyro;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!gyro.begin()) {
    Serial.println("L3G4200D not found - check wiring, power, CS and SDO.");
    while (true) {
      delay(1000);
    }
  }
}

void loop() {
  gyro.update();
  Vec3 g = gyro.gyroDps();

  Serial.print("G[dps] ");
  Serial.print(g.x, 1); Serial.print(", ");
  Serial.print(g.y, 1); Serial.print(", ");
  Serial.print(g.z, 1);
  Serial.print("  T[raw] ");
  Serial.println(gyro.temperatureC(), 0);

  delay(100);
}
