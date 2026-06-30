/*
  GY521_Basic - Minimal read loop for the common GY-521 MPU-6050 breakout.
*/
#include <GY521.h>

GY521 board;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!board.begin()) {
    Serial.println("GY-521 not found - check wiring, power and AD0.");
    while (true) {
      delay(1000);
    }
  }

  Serial.print("GY-521 ready. WHO_AM_I = 0x");
  Serial.println(board.whoAmI(), HEX);
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
  Serial.print(g.z, 1);

  Serial.print("  T[C] ");
  Serial.println(board.temperatureC(), 1);

  delay(100);
}
