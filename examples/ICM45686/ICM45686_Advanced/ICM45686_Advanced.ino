/* ICM45686_Advanced - Ranges, interrupt routing and auxiliary I2C. */
#include <ICM45686.h>

ICM45686 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  if (!imu.begin()) {
    Serial.println("ICM-45686 not found.");
    while (true) delay(1000);
  }

  imu.setAccelRangeG(32);
  imu.setGyroRangeDps(4000);
  imu.setSampleRateHz(200);
  imu.configureInterruptPin(1, false, false);
  imu.routeDataReadyInterrupt(1, true);
}

void loop() {
  if (!imu.dataReady(1) || !imu.update()) return;
  ICM45686::RawSample raw;
  imu.readRaw(raw);
  Serial.print("raw accel x="); Serial.print(raw.ax);
  Serial.print(" temp C="); Serial.println(imu.temperatureC(), 2);
}
