/*
  ICM20689_Advanced - Ranges, DLPF, sample rate, data-ready and raw reads.
*/
#include <ICM20689.h>

ICM20689 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!imu.begin()) {
    Serial.println("ICM-20689 not found.");
    while (true) {
      delay(1000);
    }
  }

  imu.setAccelRangeG(16);
  imu.setGyroRangeDps(2000);
  imu.setLowPassFilterHz(92);
  imu.setSampleRateHz(200);
  imu.configureInterruptPin(false, false);
  imu.setDataReadyInterrupt(true);

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

  ICM20689::RawSample raw;
  imu.readRaw(raw);

  Serial.print(a.x, 2); Serial.print(' ');
  Serial.print(a.y, 2); Serial.print(' ');
  Serial.print(a.z, 2); Serial.print("  |  ");
  Serial.print(g.x, 1); Serial.print(' ');
  Serial.print(g.y, 1); Serial.print(' ');
  Serial.print(g.z, 1); Serial.print("  |  ");
  Serial.println(raw.ax);
}
