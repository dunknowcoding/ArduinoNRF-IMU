/* GYLSM6DS3_Advanced - INT1/INT2 routing and raw samples. */
#include <GYLSM6DS3.h>

GYLSM6DS3 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  if (!imu.begin()) {
    Serial.println("GY-LSM6DS3 not found.");
    while (true) delay(1000);
  }
  imu.setAccelRangeG(8);
  imu.setGyroRangeDps(1000);
  imu.setSampleRateHz(208);
  imu.configureInterruptPins(false, false);
  imu.setDataReadyInterrupt(1, true, true);
  imu.routeInterrupt(2, nimu::lsm6ds3::INT_BOOT);
}

void loop() {
  if (!imu.dataReady() || !imu.update()) return;
  GYLSM6DS3::RawSample raw;
  imu.readRaw(raw);
  Serial.print("raw accel ");
  Serial.print(raw.ax); Serial.print(',');
  Serial.print(raw.ay); Serial.print(',');
  Serial.println(raw.az);
}
