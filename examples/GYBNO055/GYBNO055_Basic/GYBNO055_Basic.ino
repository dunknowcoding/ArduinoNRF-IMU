/*
  GYBNO055_Basic - Minimal read loop for a GY-BNO055 breakout.
*/
#include <GYBNO055.h>

GYBNO055 board;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!board.begin()) {
    Serial.println("GY-BNO055 not found - check wiring, power and ADR.");
    while (true) {
      delay(1000);
    }
  }
}

void loop() {
  board.update();

  Vec3 a = board.accelG();
  Vec3 g = board.gyroDps();

  Serial.print("A[g] ");
  Serial.print(a.x, 2); Serial.print(", ");
  Serial.print(a.y, 2); Serial.print(", ");
  Serial.print(a.z, 2);

  Serial.print("  G[dps] ");
  Serial.print(g.x, 1); Serial.print(", ");
  Serial.print(g.y, 1); Serial.print(", ");
  Serial.println(g.z, 1);

  delay(100);
}
