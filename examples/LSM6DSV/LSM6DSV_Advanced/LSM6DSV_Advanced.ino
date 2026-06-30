/* LSM6DSV_Advanced - Ranges and both interrupt outputs. */
#include <LSM6DSV.h>

LSM6DSV imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!imu.begin()) {
    Serial.println("LSM6DSV not found.");
    while (true) {}
  }
  imu.setAccelRangeG(8);
  imu.setGyroRangeDps(2000);
  imu.setSampleRateHz(240);
  imu.configureInterruptPins(false, false);
  imu.setDataReadyInterrupt(1, true, true);
  imu.setDataReadyInterrupt(2, true, false);
}

void loop() {
  if (imu.dataReady() && imu.update()) {
    const IMUData& d = imu.data();
    Serial.print(d.accel.x);
    Serial.print(',');
    Serial.println(d.gyro.x);
  }
}
