/*
  GY91_Basic - The fastest way to read a GY-91 (10-DOF) board.

  One object gives you the InvenSense IMU (acceleration in g, rotation in deg/s,
  and magnetic field in uT on genuine 9-axis boards) plus the BMP280 barometer
  (pressure in hPa, temperature in C, and altitude in metres).

  Wiring (ArduinoNRF ProMicro nRF52840 default I2C):
    SDA -> SDA (silk D6)    SCL -> SCL (silk D7)    VCC -> 3V3    GND -> GND

  Open Serial Monitor at 115200 baud.
*/
#include <GY91.h>

GY91 board;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!board.begin()) {
    // begin() is true only if BOTH chips answer; show which one is missing.
    Serial.print("GY-91 not fully detected. IMU: ");
    Serial.print(board.imuOk() ? "ok" : "MISSING");
    Serial.print("  BMP280: ");
    Serial.println(board.baroOk() ? "ok" : "MISSING");
  }

  Serial.print("Magnetometer: ");
  Serial.println(board.hasMagnetometer() ? "yes (9-axis)" : "no (6-axis board)");

  // Altitude is relative to sea-level pressure. The default is the standard
  // atmosphere (1013.25 hPa); set your local QNH for an accurate height.
  board.setSeaLevelPressureHpa(1013.25);
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
  Serial.print(board.altitudeM(), 1);
  Serial.print("  T[C] ");
  Serial.println(board.baroTemperatureC(), 1);

  delay(200);
}
