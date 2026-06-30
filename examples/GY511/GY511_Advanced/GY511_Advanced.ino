/*
  GY511_Advanced - Accel/mag ranges, ODR, data-ready and raw reads.
*/
#include <GY511.h>

GY511 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!imu.begin()) {
    Serial.println("GY-511/LSM303DLHC not found.");
    while (true) {
      delay(1000);
    }
  }

  imu.setAccelRangeG(8);
  imu.setSampleRateHz(200);
  imu.setMagRangeGauss(1.9f);
  imu.configureAccelInterruptPolarity(false);
  imu.setAccelDataReadyInterrupt(true);

  Serial.println("Calibrating accel - keep the module still...");
  imu.calibrateAccel(300);
  Serial.println("Streaming. Columns: ax ay az (g) | mx my mz (uT) | raw ax mx");
}

void loop() {
  if (!imu.dataReady() || !imu.magDataReady()) {
    return;
  }

  imu.update();
  Vec3 a = imu.accelG();
  Vec3 m = imu.magUT();

  GY511::RawSample raw;
  imu.readRaw(raw);

  Serial.print(a.x, 2); Serial.print(' ');
  Serial.print(a.y, 2); Serial.print(' ');
  Serial.print(a.z, 2); Serial.print("  |  ");
  Serial.print(m.x, 1); Serial.print(' ');
  Serial.print(m.y, 1); Serial.print(' ');
  Serial.print(m.z, 1); Serial.print("  |  ");
  Serial.print(raw.ax); Serial.print(' ');
  Serial.println(raw.mx);
}
