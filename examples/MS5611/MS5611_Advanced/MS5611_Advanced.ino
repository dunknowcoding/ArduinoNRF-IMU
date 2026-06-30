/*
  MS5611_Advanced - Oversampling, sea-level pressure, altitude calibration and raw reads.
*/
#include <MS5611.h>

MS5611 baro;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!baro.begin()) {
    Serial.println("MS5611 not found.");
    while (true) {
      delay(1000);
    }
  }

  baro.setOversampling(MS5611::OSR_4096);
  baro.setSeaLevelPressureHpa(1013.25f);

  Serial.println("Streaming. Columns: pressure hPa | temp C | D1 D2");
}

void loop() {
  baro.update();

  uint32_t d1 = 0;
  uint32_t d2 = 0;
  baro.readRaw(d1, d2);

  Serial.print(baro.pressureHpa(), 2); Serial.print("  |  ");
  Serial.print(baro.temperatureC(), 2); Serial.print("  |  ");
  Serial.print(d1); Serial.print(' ');
  Serial.println(d2);

  delay(250);
}
