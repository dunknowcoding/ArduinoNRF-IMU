/* BNO08x_Fusion - SH-2 orientation and motion classifiers. */
#include <BNO08x.h>

BNO08x imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  if (!imu.begin()) {
    Serial.println("BNO08x not found.");
    while (true) delay(1000);
  }
  imu.enableGameRotationVector(10000);
  imu.enableGeomagneticRotationVector(20000);
  imu.enableLinearAcceleration(10000);
  imu.enableGravity(10000);
  imu.enableStepCounter(100000);
  imu.enableStabilityClassifier();
  imu.enableActivityClassifier();
  imu.enableTapDetector();
}

void loop() {
  if (!imu.update()) return;
  BNO08x::Quaternion q = imu.quaternion();
  BNO08x::Quaternion game = imu.gameQuaternion();
  Serial.print("rotation w="); Serial.print(q.w, 4);
  Serial.print(" game w="); Serial.print(game.w, 4);
  Serial.print(" stability="); Serial.print(imu.stabilityClass());
  Serial.print(" activity="); Serial.print(imu.activityClass());
  Serial.print(" confidence=");
  Serial.print(imu.activityConfidence(imu.activityClass()));
  Serial.print(" tap=0x"); Serial.print(imu.tapCode(), HEX);
  Serial.print(" steps="); Serial.println(imu.stepCount());
}
