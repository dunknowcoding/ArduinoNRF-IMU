/*
  GY63_Basic - Minimal read loop for a GY-63 MS5611 barometer module.
*/
#include <GY63.h>

GY63 baro;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!baro.begin()) {
    Serial.println("GY-63/MS5611 not found - check wiring, power and CSB address.");
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
