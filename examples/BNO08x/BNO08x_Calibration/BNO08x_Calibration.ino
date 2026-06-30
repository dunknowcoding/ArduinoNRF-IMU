/* BNO08x_Calibration - Guided SH-2 dynamic calibration and DCD save. */
#include <BNO08x.h>

BNO08x imu;
bool saved = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  if (!imu.begin()) {
    Serial.println("BNO08x not found.");
    while (true) delay(1000);
  }
  imu.setSampleRateHz(50);
  if (!imu.beginCalibration()) {
    Serial.println("Could not enable SH-2 calibration.");
  }
  Serial.println("Gyro: keep still for 2-3 seconds.");
  Serial.println("Accel: hold 4-6 distinct orientations for about 1 second each.");
  Serial.println("Mag: rotate about 180 degrees and back around all three axes.");
}

void loop() {
  imu.update();
  Serial.print("accuracy accel/gyro/mag: ");
  Serial.print(imu.accelAccuracy()); Serial.print('/');
  Serial.print(imu.gyroAccuracy()); Serial.print('/');
  Serial.println(imu.magAccuracy());

  if (imu.calibrationComplete() && !saved) {
    if (imu.saveCalibration()) Serial.println("Dynamic calibration saved to DCD.");
    imu.endCalibration();
    saved = true;
  }
  delay(100);
}
