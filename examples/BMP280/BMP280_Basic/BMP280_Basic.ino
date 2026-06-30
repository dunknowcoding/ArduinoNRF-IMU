/*
  BMP280_Basic - Read a Bosch BMP280 pressure + temperature sensor.

  Prints barometric pressure (hPa) and temperature (C). The
  BMP280 is the barometer on a GY-91 board; for the whole GY-91 at once use
  <GY91.h>.

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
}

void loop() {
  baro.update();

  Serial.print("P ");
  Serial.print(baro.pressureHpa(), 2);
  Serial.print(" hPa   T ");
  Serial.print(baro.temperatureC(), 2);
  Serial.println(" C");

  delay(500);
}
