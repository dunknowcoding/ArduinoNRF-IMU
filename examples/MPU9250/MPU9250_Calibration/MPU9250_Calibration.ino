/*
  MPU9250_Calibration - Calibrate once, paste the numbers, never re-do it.

  This sketch walks you through all three calibrations and then prints a ready
  made block of C++ you can copy into your own sketch. Restoring calibration
  with setCalibration() is instant - no need to wave the board around at every
  power-up.

  Steps it runs:
    1. Gyro bias   - board still and flat.
    2. Accel bias  - board still and LEVEL (Z pointing up).
    3. Magnetometer hard/soft iron - slowly rotate the board through every
       orientation (a "figure-8" in the air) for 15 seconds.

  Open Serial Monitor at 115200 baud and follow the prompts.
*/
#include <MPU9250.h>

MPU9250 imu;

// Small helper: wait for the user to press Enter in the Serial Monitor.
void waitForEnter(const char* prompt) {
  Serial.println(prompt);
  while (Serial.available()) {
    Serial.read();  // drain anything left over
  }
  while (!Serial.available()) {
    delay(10);
  }
  while (Serial.available()) {
    Serial.read();
  }
}

void printVec(const char* label, Vec3 v) {
  Serial.print(label);
  Serial.print(v.x, 4); Serial.print(", ");
  Serial.print(v.y, 4); Serial.print(", ");
  Serial.println(v.z, 4);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!imu.begin()) {
    Serial.println("MPU9250 not found - check wiring.");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("=== MPU9250 calibration ===");

  // --- 1. Gyro ---
  waitForEnter("\n1) Place the board flat and STILL. Press Enter to start.");
  imu.calibrateGyro(500);
  Serial.println("   gyro done.");

  // --- 2. Accelerometer ---
  waitForEnter("\n2) Keep it LEVEL and still (Z up). Press Enter to start.");
  imu.calibrateAccel(500);
  Serial.println("   accel done.");

  // --- 3. Magnetometer ---
  if (imu.hasMagnetometer()) {
    waitForEnter(
        "\n3) Slowly rotate the board in all directions for 15 s.\n"
        "   Press Enter to start, then keep moving it...");
    if (imu.calibrateMag(15000)) {
      Serial.println("   mag done.");
    } else {
      Serial.println("   mag calibration failed (not enough motion).");
    }
  }

  // --- Print the result as copy-paste code ---
  IMUCalibration c = imu.getCalibration();
  Serial.println("\n=== Copy this into your sketch ===");
  Serial.println("IMUCalibration cal;");
  printVec("cal.accelBias  = {", c.accelBias);
  printVec("cal.accelScale = {", c.accelScale);
  printVec("cal.gyroBias   = {", c.gyroBias);
  printVec("cal.magBias    = {", c.magBias);
  printVec("cal.magScale   = {", c.magScale);
  Serial.println("imu.setCalibration(cal);");
  Serial.println("==================================");
}

void loop() {
  // Show the calibrated output so you can confirm it looks right:
  // level board ~ (0,0,1) g, still board ~ 0 deg/s.
  imu.update();
  Vec3 a = imu.accelG();
  Vec3 g = imu.gyroDps();
  Serial.print("A "); Serial.print(a.x, 2); Serial.print(",");
  Serial.print(a.y, 2); Serial.print(","); Serial.print(a.z, 2);
  Serial.print("  G "); Serial.print(g.x, 1); Serial.print(",");
  Serial.print(g.y, 1); Serial.print(","); Serial.println(g.z, 1);
  delay(200);
}
