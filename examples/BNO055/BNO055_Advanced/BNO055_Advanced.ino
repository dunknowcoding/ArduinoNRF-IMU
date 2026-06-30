/*
  BNO055_Advanced - Fusion mode, calibration status, Euler and quaternion.
*/
#include <BNO055.h>

BNO055 imu;
constexpr int8_t BNO_RST_PIN = 3;
constexpr int8_t BNO_INT_PIN = 2;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  imu.configurePins(BNO_RST_PIN, BNO_INT_PIN);
  if (!imu.begin()) {
    Serial.println("BNO055 not found.");
    while (true) {
      delay(1000);
    }
  }

  imu.setExternalCrystal(true);
  imu.setMode(BNO055::MODE_NDOF);
}

void loop() {
  imu.update();

  Vec3 e = imu.eulerDeg();
  BNO055::Quaternion q = imu.quaternion();

  Serial.print("cal sys/g/a/m ");
  Serial.print(imu.systemCalibration()); Serial.print("/");
  Serial.print(imu.gyroCalibration()); Serial.print("/");
  Serial.print(imu.accelCalibration()); Serial.print("/");
  Serial.print(imu.magCalibration());
  Serial.print("  INT=");
  Serial.print(imu.interruptAsserted());
  Serial.print(" source=0x");
  Serial.print(imu.interruptStatus(), HEX);

  Serial.print("  euler heading/roll/pitch ");
  Serial.print(e.x, 1); Serial.print(", ");
  Serial.print(e.y, 1); Serial.print(", ");
  Serial.print(e.z, 1);

  Serial.print("  quat ");
  Serial.print(q.w, 4); Serial.print(", ");
  Serial.print(q.x, 4); Serial.print(", ");
  Serial.print(q.y, 4); Serial.print(", ");
  Serial.println(q.z, 4);

  delay(100);
}
