/*
  MS5611_Basic - Minimal read loop for an MS5611 barometer.
*/
#include <MS5611.h>

MS5611 baro;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!baro.begin()) {
    Serial.println("MS5611 not found - check wiring, power and CSB address.");
    while (true) {
      delay(1000);
    }
  }
}

void loop() {
  baro.update();

  Serial.print("P[hPa] ");
  Serial.print(baro.pressureHpa(), 2);
  Serial.print("  T[C] ");
  Serial.print(baro.temperatureC(), 2);
  Serial.print("  Alt[m] ");
  Serial.println(baro.altitudeM(), 1);

  delay(250);
}
