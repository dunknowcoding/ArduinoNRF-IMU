/*
  GY291_Advanced - Range, sample rate, data-ready and raw reads.
*/
#include <GY291.h>

GY291 accel;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!accel.begin()) {
    Serial.println("GY-291/ADXL345 not found.");
    while (true) {
      delay(1000);
    }
  }

  accel.setAccelRangeG(16);
  accel.setSampleRateHz(200);
  accel.configureInterruptPolarity(false);
  accel.routeInterrupt(GY291::INTERRUPT_DATA_READY, 1);  // INT1 silk

  Serial.println("Calibrating accel - keep the module still...");
  accel.calibrateAccel(300);
  Serial.println("Streaming. Columns: ax ay az (g) | raw ax");
}

void loop() {
  if (!accel.dataReady()) {
    return;
  }

  accel.update();
  Vec3 a = accel.accelG();

  GY291::RawSample raw;
  accel.readRaw(raw);

  Serial.print(a.x, 2); Serial.print(' ');
  Serial.print(a.y, 2); Serial.print(' ');
  Serial.print(a.z, 2); Serial.print("  |  ");
  Serial.println(raw.ax);
}
