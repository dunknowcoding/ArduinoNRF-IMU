/*
  ICM20948_Advanced - Ranges, sample rate, low-pass filter and raw reads.
*/
#include <ICM20948.h>

ICM20948 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!imu.begin()) {
    Serial.println("ICM-20948 not found.");
    while (true) {
      delay(1000);
    }
  }

  imu.setAccelRangeG(16);
  imu.setGyroRangeDps(2000);
  imu.setSampleRateHz(225);
  imu.setLowPassFilterHz(51);
  imu.configureInterruptPin(false, false);
  imu.setDataReadyInterrupt(true);
  imu.setFsyncInterrupt(true);

  Serial.println("Calibrating gyro - keep the board still...");
  imu.calibrateGyro(300);
  Serial.println("Streaming. Columns: ax ay az (g) | gx gy gz (dps) | raw ax");
}

void loop() {
  if (!imu.dataReady()) {
    return;
  }

  imu.update();

  Vec3 a = imu.accelG();
  Vec3 g = imu.gyroDps();
  Vec3 m = imu.magUT();

  ICM20948::RawSample raw;
  imu.readRaw(raw);

  Serial.print(a.x, 2); Serial.print(' ');
  Serial.print(a.y, 2); Serial.print(' ');
  Serial.print(a.z, 2); Serial.print("  |  ");
  Serial.print(g.x, 1); Serial.print(' ');
  Serial.print(g.y, 1); Serial.print(' ');
  Serial.print(g.z, 1); Serial.print("  |  ");
  Serial.print(m.x, 1); Serial.print(' ');
  Serial.print(m.y, 1); Serial.print(' ');
  Serial.print(m.z, 1); Serial.print("  |  ");
  Serial.println(raw.ax);
}
