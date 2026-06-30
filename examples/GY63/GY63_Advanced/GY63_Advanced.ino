/*
  GY63_Advanced - PS, CSB, SDO SPI pins, oversampling, and raw reads.
*/
#include <GY63.h>

GY63 baro;
constexpr uint8_t CSB_PIN = 10;
constexpr uint8_t PS_PIN = 9;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  GY63::selectSPI(PS_PIN);
  if (!baro.beginSPI(SPI, CSB_PIN)) {
    Serial.println("GY-63/MS5611 not found.");
    while (true) {
      delay(1000);
    }
  }

  baro.setOversampling(GY63::OSR_4096);
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
