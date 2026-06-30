/* BNO08x_Advanced - Product info, SH-2 reports, accuracy and tare. */
#include <BNO08x.h>

BNO08x imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  if (!imu.beginI2C(Wire, 0x4B)) {
    Serial.println("BNO08x not found at 0x4B.");
    while (true) delay(1000);
  }

  imu.setSampleRateHz(50);
  imu.enableLinearAcceleration(20000);
  imu.enableGravity(20000);
  imu.enableStepCounter(100000);

  const BNO08x::ProductInfo& p = imu.productInfo();
  Serial.print("SH-2 "); Serial.print(p.versionMajor); Serial.print('.');
  Serial.print(p.versionMinor); Serial.print('.'); Serial.println(p.versionPatch);

  // Define the current orientation as heading zero. Persist only after the
  // mounting orientation is final: imu.saveTare();
  imu.tareNow(true);
}

void loop() {
  if (!imu.update()) return;
  BNO08x::Quaternion q = imu.quaternion();
  Vec3 linear = imu.linearAccelMs2();
  Serial.print("quat wxyz ");
  Serial.print(q.w, 4); Serial.print(','); Serial.print(q.x, 4);
  Serial.print(','); Serial.print(q.y, 4); Serial.print(','); Serial.print(q.z, 4);
  Serial.print(" acc="); Serial.print(q.accuracy);
  Serial.print(" linear[m/s2] ");
  Serial.print(linear.x, 2); Serial.print(',');
  Serial.print(linear.y, 2); Serial.print(','); Serial.print(linear.z, 2);
  Serial.print(" steps="); Serial.println(imu.stepCount());
}
