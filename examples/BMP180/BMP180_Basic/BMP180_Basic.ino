/*
  BMP180_Basic - Minimal read loop for a BMP180 barometer.
*/
#include <BMP180.h>

BMP180 baro;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!baro.begin()) {
    Serial.println("BMP180 not found - check wiring and power.");
    while (true) {
      delay(1000);
    }
  }
}

void loop() {
  baro.update();

  Serial.print("P[hPa] ");
  Serial.print(baro.pressureHpa(), 2);
  Serial.print("  Alt[m] ");
  Serial.print(baro.altitudeM(), 1);
  Serial.print("  T[C] ");
  Serial.println(baro.temperatureC(), 1);

  delay(200);
}
