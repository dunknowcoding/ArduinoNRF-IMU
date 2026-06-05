/*
  BMP280_Basic - Read a Bosch BMP280 pressure + temperature sensor.

  Prints barometric pressure (hPa), temperature (C) and altitude (m). The
  BMP280 is the barometer on a GY-91 board; for the whole GY-91 at once use
  <GY91.h>.

  About "calibration": pressure and temperature are factory-trimmed and need no
  user calibration. Altitude is computed from pressure relative to sea level, so
  for an accurate height you must tell it your local sea-level pressure (QNH).
  Two ways are shown below - pick one.

  Wiring (ArduinoNRF ProMicro nRF52840 default I2C):
    SDA -> SDA (silk D6)    SCL -> SCL (silk D7)    VCC -> 3V3    GND -> GND

  Open Serial Monitor at 115200 baud.
*/
#include <BMP280.h>

BMP280 baro;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!baro.begin()) {  // Wire @ 0x76 (use baro.begin(0x77) if SDO is high)
    Serial.println("BMP280 not found - check wiring and the 0x76/0x77 address.");
    while (true) {
      delay(1000);
    }
  }

  // Option A: type today's local sea-level pressure (from a weather report).
  baro.setSeaLevelPressureHpa(1013.25);

  // Option B (uncomment): stand the board at a known height and back-solve it.
  // baro.calibrateAltitude(0.0f);   // 0 m here -> altitude is height above this
}

void loop() {
  baro.update();

  Serial.print("P ");
  Serial.print(baro.pressureHpa(), 2);
  Serial.print(" hPa   T ");
  Serial.print(baro.temperatureC(), 2);
  Serial.print(" C   Alt ");
  Serial.print(baro.altitudeM(), 1);
  Serial.println(" m");

  delay(500);
}
