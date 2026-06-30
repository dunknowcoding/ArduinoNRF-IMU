/* MPU6500_Advanced - Configuration, interrupt routing, raw data, and calibration. */
#include <MPU6500.h>

MPU6500 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!imu.begin()) {
    Serial.println("MPU-6500 not found.");
    while (true) {}
  }

  imu.setAccelRangeG(16);
  imu.setGyroRangeDps(2000);
  imu.setLowPassFilterHz(92);
  imu.setSampleRateHz(200);
  imu.configureInterruptPin(false, false, false, true);
  imu.setDataReadyInterrupt(true);
  imu.calibrateGyro(300);
}

void loop() {
  if (!imu.dataReady() || !imu.update()) return;
  MPU6500::RawSample raw;
  if (!imu.readRaw(raw)) return;
  Serial.print(imu.gyroDps().x);
  Serial.print(", raw ax=");
  Serial.println(raw.ax);
}
