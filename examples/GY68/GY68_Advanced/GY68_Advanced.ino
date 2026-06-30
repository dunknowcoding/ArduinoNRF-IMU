/*
  GY68_Advanced - Oversampling, sea-level pressure and altitude calibration.
*/
#include <GY68.h>

GY68 baro;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!baro.begin()) {
    Serial.println("GY-68/BMP180 not found.");
    while (true) {
      delay(1000);
    }
  }

  baro.setOversampling(GY68::OSS_HIGH_RES);
  baro.setSeaLevelPressureHpa(1013.25f);
  baro.calibrateAltitude(0.0f);

  Serial.println("Streaming. Columns: pressure hPa | temp C | altitude m");
}

void loop() {
  baro.update();

  Serial.print(baro.pressureHpa(), 2); Serial.print("  |  ");
  Serial.print(baro.temperatureC(), 1); Serial.print("  |  ");
  Serial.println(baro.altitudeM(), 1);

  delay(200);
}
