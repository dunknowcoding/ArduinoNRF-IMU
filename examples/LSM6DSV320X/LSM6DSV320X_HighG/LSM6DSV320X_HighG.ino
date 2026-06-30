/* LSM6DSV320X_HighG - Simultaneous low-g and 320 g shock channels. */
#include <LSM6DSV320X.h>

LSM6DSV320X imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!imu.begin() ||
      !imu.setHighGRangeG(320) ||
      !imu.setHighGSampleRateHz(1920) ||
      !imu.configureHighGWakeup(8000, 2, 2)) {
    Serial.println("LSM6DSV320X high-g startup failed.");
    while (true) {}
  }
}

void loop() {
  nimu::Vec3 highG;
  if (!imu.update() || !imu.readHighG(highG)) return;
  if (imu.highGWakeupDetected()) {
    const nimu::Vec3 lowG = imu.accelG();
    Serial.print("low-g: "); Serial.print(lowG.x); Serial.print(',');
    Serial.print(lowG.y); Serial.print(','); Serial.print(lowG.z);
    Serial.print(" high-g: "); Serial.print(highG.x); Serial.print(',');
    Serial.print(highG.y); Serial.print(','); Serial.print(highG.z);
    Serial.print(" shock="); Serial.println(imu.highGShockState());
  }
}
