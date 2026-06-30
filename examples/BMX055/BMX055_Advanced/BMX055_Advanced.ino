/* BMX055_Advanced - Independent addresses, ranges and subdevice access. */
#include <BMX055.h>

BMX055 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  if (!imu.beginI2C(Wire, 0x18, 0x68, 0x10)) {
    Serial.println("BMX055 not found.");
    while (true) delay(1000);
  }
  imu.setAccelRangeG(8);
  imu.setGyroRangeDps(1000);
  imu.setSampleRateHz(200);
}

void loop() {
  if (!imu.update()) return;
  BMX055::RawSample raw;
  imu.readRaw(raw);
  uint8_t accelStatus = 0;
  imu.readAccelRegister(0x0A, accelStatus);
  Serial.print("raw ax="); Serial.print(raw.ax);
  Serial.print(" rhall="); Serial.print(raw.rhall);
  Serial.print(" accel status=0x"); Serial.println(accelStatus, HEX);
  delay(20);
}
