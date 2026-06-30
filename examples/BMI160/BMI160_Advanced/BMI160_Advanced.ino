/*
  BMI160_Advanced - Ranges, sample rate, data-ready and raw reads.
*/
#include <BMI160.h>

BMI160 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!imu.begin()) {
    Serial.println("BMI160 not found.");
    while (true) {
      delay(1000);
    }
  }

  imu.setAccelRangeG(8);
  imu.setGyroRangeDps(1000);
  imu.setSampleRateHz(200);
  imu.configureInterruptPin(1, true, false);
  imu.routeDataReadyInterrupt(1);

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

  BMI160::RawSample raw;
  imu.readRaw(raw);

  Serial.print(a.x, 2); Serial.print(' ');
  Serial.print(a.y, 2); Serial.print(' ');
  Serial.print(a.z, 2); Serial.print("  |  ");
  Serial.print(g.x, 1); Serial.print(' ');
  Serial.print(g.y, 1); Serial.print(' ');
  Serial.print(g.z, 1); Serial.print("  |  ");
  Serial.println(raw.ax);
}
