/* ITG3205_Advanced - INT routing, filters, rate, calibration and raw data. */
#include <ITG3205.h>

ITG3205 gyro;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  if (!gyro.begin()) {
    Serial.println("ITG-3200/ITG-3205 not found.");
    while (true) delay(1000);
  }
  gyro.setSampleRateHz(200);
  gyro.setLowPassFilterHz(42);
  gyro.configureInterrupt(false, false);
  gyro.setDataReadyInterrupt(true);
  gyro.setPllReadyInterrupt(true);
  gyro.calibrateGyro(300);
}

void loop() {
  if (!gyro.dataReady() || !gyro.update()) return;
  ITG3205::RawSample raw;
  gyro.readRaw(raw);
  Vec3 g = gyro.gyroDps();
  Serial.print(g.x, 2); Serial.print(',');
  Serial.print(g.y, 2); Serial.print(',');
  Serial.print(g.z, 2); Serial.print(" raw="); Serial.println(raw.gx);
}
