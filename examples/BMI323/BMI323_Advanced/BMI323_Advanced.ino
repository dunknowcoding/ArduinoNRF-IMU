/* BMI323_Advanced - Address, ranges, rate, readiness and raw values. */
#include <BMI323.h>

BMI323 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  // SDO low selects 0x68; CSB must be high for I2C mode.
  if (!imu.beginI2C(Wire, 0x68)) {
    Serial.println("BMI323 not found at 0x68.");
    while (true) delay(1000);
  }
  imu.setAccelRangeG(8);
  imu.setGyroRangeDps(1000);
  imu.setSampleRateHz(200);
  imu.configureInterruptPin(1, true, false);
  imu.routeDataReadyInterrupt(1, true, true);
  Serial.print("Chip ID 0x"); Serial.println(imu.whoAmI(), HEX);
}

void loop() {
  if (!imu.dataReady()) return;
  BMI323::RawSample raw;
  if (!imu.readRaw(raw)) return;
  Serial.print("raw A ");
  Serial.print(raw.ax); Serial.print(',');
  Serial.print(raw.ay); Serial.print(','); Serial.print(raw.az);
  Serial.print(" G ");
  Serial.print(raw.gx); Serial.print(',');
  Serial.print(raw.gy); Serial.print(','); Serial.print(raw.gz);
  Serial.print(" T "); Serial.println(raw.temp);
}
