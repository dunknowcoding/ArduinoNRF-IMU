/*
  GY80_Basic - Minimal read loop for a GY-80/GY-801 marketplace module.
*/
#include <GY80.h>

GY80 board;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!board.begin()) {
    Serial.println("GY-80 core sensors not found - check wiring and power.");
    while (true) {
      delay(1000);
    }
  }
}

void loop() {
  board.update();

  Vec3 a = board.accelG();
  Vec3 g = board.gyroDps();
  Vec3 m = board.magUT();

  Serial.print("A[g] ");
  Serial.print(a.x, 2); Serial.print(", ");
  Serial.print(a.y, 2); Serial.print(", ");
  Serial.print(a.z, 2);

  Serial.print("  G[dps] ");
  Serial.print(g.x, 1); Serial.print(", ");
  Serial.print(g.y, 1); Serial.print(", ");
  Serial.print(g.z, 1);

  Serial.print("  M[uT] ");
  Serial.print(m.x, 1); Serial.print(", ");
  Serial.print(m.y, 1); Serial.print(", ");
  Serial.print(m.z, 1);

  Serial.print("  P[hPa] ");
  Serial.println(board.pressureHpa(), 1);

  delay(100);
}
