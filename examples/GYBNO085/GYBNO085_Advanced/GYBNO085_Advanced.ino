/* GYBNO085_Advanced - Alternate address, product build and fusion accuracy. */
#include <GYBNO085.h>

GYBNO085 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  // On this exact board, ADDR low selects 0x4A. Leave it high/open for 0x4B.
  if (!imu.beginI2C(Wire, 0x4B)) {
    Serial.println("GY-BNO085 not found at 0x4B.");
    while (true) delay(1000);
  }
  imu.setSampleRateHz(100);
  const BNO08x::ProductInfo& p = imu.productInfo();
  Serial.print("part 0x"); Serial.print(p.partNumber, HEX);
  Serial.print(" build "); Serial.println(p.buildNumber);
}

void loop() {
  if (!imu.update()) return;
  BNO08x::Quaternion q = imu.quaternion();
  Serial.print("q ");
  Serial.print(q.w, 4); Serial.print(','); Serial.print(q.x, 4);
  Serial.print(','); Serial.print(q.y, 4); Serial.print(','); Serial.print(q.z, 4);
  Serial.print(" accuracy a/g/m/q ");
  Serial.print(imu.accelAccuracy()); Serial.print('/');
  Serial.print(imu.gyroAccuracy()); Serial.print('/');
  Serial.print(imu.magAccuracy()); Serial.print('/'); Serial.println(q.accuracy);
}
