/* BNO055_Calibration - Guided Bosch calibration and profile capture. */
#include <BNO055.h>

BNO055 imu;
bool printed = false;

void printProfile(const BNO055::CalibrationProfile& profile) {
  Serial.println("BNO055::CalibrationProfile profile = {{");
  for (uint8_t i = 0; i < sizeof(profile.data); ++i) {
    Serial.print("0x");
    if (profile.data[i] < 16) Serial.print('0');
    Serial.print(profile.data[i], HEX);
    Serial.print(i + 1 == sizeof(profile.data) ? "" : ", ");
  }
  Serial.println("}};");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  if (!imu.begin()) {
    Serial.println("BNO055 not found.");
    while (true) delay(1000);
  }
  imu.setExternalCrystal(true);
  imu.setMode(BNO055::MODE_NDOF);
  Serial.println("Gyro: hold still for several seconds.");
  Serial.println("Accel: hold each of six faces still for several seconds.");
  Serial.println("Mag: make slow figure-eight and full-axis rotations.");
}

void loop() {
  imu.update();
  Serial.print("system/gyro/accel/mag: ");
  Serial.print(imu.systemCalibration()); Serial.print('/');
  Serial.print(imu.gyroCalibration()); Serial.print('/');
  Serial.print(imu.accelCalibration()); Serial.print('/');
  Serial.println(imu.magCalibration());

  if (imu.fullyCalibrated() && !printed) {
    BNO055::CalibrationProfile profile;
    if (imu.readCalibrationProfile(profile)) printProfile(profile);
    printed = true;
  }
  delay(250);
}
