/*
  L3G4200D_Advanced - Range, sample rate, data-ready and raw reads.
*/
#include <L3G4200D.h>

L3G4200D gyro;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!gyro.begin()) {
    Serial.println("L3G4200D not found.");
    while (true) {
      delay(1000);
    }
  }

  gyro.setGyroRangeDps(2000);
  gyro.setSampleRateHz(200);
  gyro.setLowPassFilterHz(50);
  gyro.configureInterruptPins(false, false);
  gyro.setDataReadyInterrupt(true);  // GY-50 DR pad
  gyro.setThresholdInterrupt(true);  // GY-50 INT pad

  Serial.println("Calibrating gyro - keep the board still...");
  gyro.calibrateGyro(300);
  Serial.println("Streaming. Columns: gx gy gz (dps) | raw gx");
}

void loop() {
  if (!gyro.dataReady()) {
    return;
  }

  gyro.update();
  Vec3 g = gyro.gyroDps();

  L3G4200D::RawSample raw;
  gyro.readRaw(raw);

  Serial.print(g.x, 1); Serial.print(' ');
  Serial.print(g.y, 1); Serial.print(' ');
  Serial.print(g.z, 1); Serial.print("  |  ");
  Serial.println(raw.gx);
}
