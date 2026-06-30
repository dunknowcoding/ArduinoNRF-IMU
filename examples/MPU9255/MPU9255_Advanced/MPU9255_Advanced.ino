/*
  MPU9255_Advanced - Ranges, DLPF, sample rate, magnetometer and raw reads.
*/
#include <MPU9255.h>

MPU9255 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!imu.begin()) {
    Serial.println("MPU-9255 not found.");
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
  imu.setDataReadyInterrupt(true);
  imu.enableMagnetometer(true);

  Serial.println("Calibrating gyro - keep the board still...");
  imu.calibrateGyro(300);

  Serial.print("Magnetometer: ");
  Serial.println(imu.hasMagnetometer() ? "AK8963 ready" : "not detected");
  Serial.println("Streaming. Columns: ax ay az | gx gy gz | mx my mz | raw ax");
}

void loop() {
  if (!imu.dataReady()) {
    return;
  }

  imu.update();

  Vec3 a = imu.accelG();
  Vec3 g = imu.gyroDps();
  Vec3 m = imu.magUT();

  MPU9255::RawSample raw;
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
