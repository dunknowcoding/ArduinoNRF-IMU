/*
  GY88_Basic - Minimal read loop for a GY-88 10-DOF module.
*/
#include <GY88.h>

GY88 board;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!board.begin()) {
    Serial.print("GY-88 incomplete. IMU:");
    Serial.print(board.imuOk() ? "ok" : "MISSING");
    Serial.print(" BMP085/BMP180:");
    Serial.print(board.baroOk() ? "ok" : "MISSING");
    Serial.print(" compass:");
    Serial.println(board.compassName());
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

  if (board.hasMagnetometer()) {
    Serial.print("  M[uT] ");
    Serial.print(m.x, 1); Serial.print(", ");
    Serial.print(m.y, 1); Serial.print(", ");
    Serial.print(m.z, 1);
  }

  Serial.print("  P[hPa] ");
  Serial.print(board.pressureHpa(), 2);
  Serial.print("  Alt[m] ");
  Serial.println(board.altitudeM(), 1);

  delay(200);
}
