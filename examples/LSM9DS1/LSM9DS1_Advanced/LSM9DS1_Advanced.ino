/*
  LSM9DS1_Advanced - Ranges, sample rate, mag range and raw reads.
*/
#include <LSM9DS1.h>

LSM9DS1 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!imu.begin()) {
    Serial.println("LSM9DS1 not found.");
    while (true) {
      delay(1000);
    }
  }

  imu.setAccelRangeG(8);
  imu.setGyroRangeDps(2000);
  imu.setMagRangeGauss(8);
  imu.setSampleRateHz(238);
  imu.routeDataReadyInterrupt(1, true, true);
  imu.configureMagInterrupt(0x07, 1000, true, false, true);

  Serial.println("Calibrating gyro - keep the board still...");
  imu.calibrateGyro(300);
  Serial.println("Streaming. Columns: ax ay az (g) | mx my mz (uT) | raw ax");
}

void loop() {
  imu.update();

  Vec3 a = imu.accelG();
  Vec3 m = imu.magUT();

  LSM9DS1::RawSample raw;
  imu.readRaw(raw);

  Serial.print(a.x, 2); Serial.print(' ');
  Serial.print(a.y, 2); Serial.print(' ');
  Serial.print(a.z, 2); Serial.print("  |  ");
  Serial.print(m.x, 1); Serial.print(' ');
  Serial.print(m.y, 1); Serial.print(' ');
  Serial.print(m.z, 1); Serial.print("  |  ");
  Serial.println(raw.ax);

  delay(100);
}
