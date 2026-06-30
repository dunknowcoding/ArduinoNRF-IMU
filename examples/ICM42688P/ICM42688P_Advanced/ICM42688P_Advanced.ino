/*
  ICM42688P_Advanced - Ranges, sample rate, low-pass filter and raw reads.
*/
#include <ICM42688P.h>

ICM42688P imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!imu.begin()) {
    Serial.println("ICM-42688-P not found.");
    while (true) {
      delay(1000);
    }
  }

  imu.setAccelRangeG(16);
  imu.setGyroRangeDps(2000);
  imu.setSampleRateHz(500);
  imu.setLowPassFilterHz(100);
  imu.configureInterruptPin(1, true, false);
  imu.routeDataReadyInterrupt(1);
  imu.setPin9Function(ICM42688P::Pin9Function::Fsync);
  imu.routeFsyncInterrupt(1);

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

  ICM42688P::RawSample raw;
  imu.readRaw(raw);

  Serial.print(a.x, 2); Serial.print(' ');
  Serial.print(a.y, 2); Serial.print(' ');
  Serial.print(a.z, 2); Serial.print("  |  ");
  Serial.print(g.x, 1); Serial.print(' ');
  Serial.print(g.y, 1); Serial.print(' ');
  Serial.print(g.z, 1); Serial.print("  |  ");
  Serial.println(raw.ax);
}
