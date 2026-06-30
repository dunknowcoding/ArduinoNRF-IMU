/* BMI088_Advanced - Addresses, configuration, readiness and raw values. */
#include <BMI088.h>

BMI088 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  // Common modules strap SDO1/SDO2 low: accel=0x18, gyro=0x68.
  if (!imu.beginI2C(Wire, 0x18, 0x68)) {
    Serial.println("BMI088 not found at accel 0x18 + gyro 0x68.");
    while (true) delay(1000);
  }
  imu.setAccelRangeG(12);
  imu.setGyroRangeDps(1000);
  imu.setSampleRateHz(200);
  imu.configureAccelInterruptPin(1, true, false);
  imu.routeAccelDataReadyInterrupt(1);
  imu.configureGyroInterruptPin(3, true, false);
  imu.routeGyroDataReadyInterrupt(3);

  Serial.print("IDs accel=0x"); Serial.print(imu.whoAmI(), HEX);
  Serial.print(" gyro=0x"); Serial.println(imu.gyroWhoAmI(), HEX);
}

void loop() {
  if (!imu.dataReady()) return;
  BMI088::RawSample raw;
  if (!imu.readRaw(raw)) return;
  Serial.print("raw A ");
  Serial.print(raw.ax); Serial.print(',');
  Serial.print(raw.ay); Serial.print(','); Serial.print(raw.az);
  Serial.print(" G ");
  Serial.print(raw.gx); Serial.print(',');
  Serial.print(raw.gy); Serial.print(','); Serial.println(raw.gz);
}
