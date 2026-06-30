/* BMP280_Advanced - Sampling and altitude-reference configuration. */
#include <BMP280.h>

BMP280 barometer;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!barometer.begin()) {
    Serial.println("BMP280 not found.");
    while (true) {}
  }

  barometer.setSampling(BMP280::OSRS_X16, BMP280::OSRS_X2,
                        BMP280::FILTER_16, BMP280::MODE_NORMAL);
  // Replace this standard-atmosphere value with the local QNH when available.
  barometer.setSeaLevelPressureHpa(1013.25f);
  // Alternatively, measure at a known height:
  // barometer.calibrateAltitude(knownHeightM);
}

void loop() {
  if (!barometer.update()) return;
  Serial.print(barometer.pressureHpa(), 2);
  Serial.print(" hPa, ");
  Serial.print(barometer.altitudeM(), 1);
  Serial.println(" m");
  delay(500);
}
