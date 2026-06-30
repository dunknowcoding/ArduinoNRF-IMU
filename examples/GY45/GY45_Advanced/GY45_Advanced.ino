/*
  GY45_Advanced - Range, ODR, data-ready and raw reads.
*/
#include <GY45.h>

GY45 accel;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!accel.begin()) {
    Serial.println("GY-45/MMA8452Q not found.");
    while (true) {
      delay(1000);
    }
  }

  accel.setAccelRangeG(8);
  accel.setSampleRateHz(200);
  accel.configureInterruptPins(true, false);
  accel.routeInterrupt(GY45::INTERRUPT_DATA_READY, 1);  // INT1 silk

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

  GY45::RawSample raw;
  accel.readRaw(raw);

  Serial.print(a.x, 2); Serial.print(' ');
  Serial.print(a.y, 2); Serial.print(' ');
  Serial.print(a.z, 2); Serial.print("  |  ");
  Serial.println(raw.ax);
}
