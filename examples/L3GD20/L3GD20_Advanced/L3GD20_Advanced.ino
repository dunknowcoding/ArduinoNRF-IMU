/* L3GD20_Advanced - DRDY/INT pads, identity and raw data. */
#include <L3GD20.h>

L3GD20 gyro;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  if (!gyro.begin()) {
    Serial.println("L3GD20/L3GD20H not found.");
    while (true) delay(1000);
  }
  Serial.print("WHO_AM_I=0x"); Serial.println(gyro.whoAmI(), HEX);
  gyro.configureInterruptPins(false, false);
  gyro.setDataReadyInterrupt(true);
  gyro.setThresholdInterrupt(true);
  gyro.setGyroRangeDps(2000);
}

void loop() {
  if (!gyro.dataReady() || !gyro.update()) return;
  L3GD20::RawSample raw;
  gyro.readRaw(raw);
  Serial.print("raw gyro "); Serial.print(raw.gx); Serial.print(',');
  Serial.print(raw.gy); Serial.print(','); Serial.println(raw.gz);
}
