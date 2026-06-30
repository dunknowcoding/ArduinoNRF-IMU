/* ADXL345_Advanced - INT1/INT2 routing, range, rate and raw data. */
#include <ADXL345.h>

ADXL345 accel;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  if (!accel.begin()) {
    Serial.println("ADXL345 not found.");
    while (true) delay(1000);
  }
  accel.setAccelRangeG(16);
  accel.setSampleRateHz(200);
  accel.configureInterruptPolarity(false);
  accel.routeInterrupt(ADXL345::INTERRUPT_DATA_READY, 1);
}

void loop() {
  uint8_t source = accel.interruptSource();
  if ((source & ADXL345::INTERRUPT_DATA_READY) == 0) return;
  accel.update();
  ADXL345::RawSample raw;
  accel.readRaw(raw);
  Vec3 a = accel.accelG();
  Serial.print(a.x, 3); Serial.print(',');
  Serial.print(a.y, 3); Serial.print(',');
  Serial.print(a.z, 3); Serial.print(" raw="); Serial.println(raw.ax);
}
