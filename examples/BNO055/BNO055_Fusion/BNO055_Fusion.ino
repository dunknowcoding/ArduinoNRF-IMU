/* BNO055_Fusion - Quaternion, Euler, linear acceleration and gravity. */
#include <BNO055.h>

BNO055 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  if (!imu.begin()) {
    Serial.println("BNO055 not found.");
    while (true) delay(1000);
  }
  imu.setMode(BNO055::MODE_NDOF);
}

void loop() {
  if (!imu.update()) return;
  BNO055::Quaternion q = imu.quaternion();
  Vec3 e = imu.eulerDeg();
  Vec3 linear = imu.linearAccelMs2();
  Vec3 gravity = imu.gravityMs2();
  Serial.print("q "); Serial.print(q.w, 4); Serial.print(',');
  Serial.print(q.x, 4); Serial.print(','); Serial.print(q.y, 4);
  Serial.print(','); Serial.print(q.z, 4);
  Serial.print(" euler "); Serial.print(e.x, 1); Serial.print(',');
  Serial.print(e.y, 1); Serial.print(','); Serial.print(e.z, 1);
  Serial.print(" linear "); Serial.print(linear.x, 2); Serial.print(',');
  Serial.print(linear.y, 2); Serial.print(','); Serial.print(linear.z, 2);
  Serial.print(" gravity "); Serial.print(gravity.x, 2); Serial.print(',');
  Serial.print(gravity.y, 2); Serial.print(','); Serial.println(gravity.z, 2);
  delay(20);
}
