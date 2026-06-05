/*
  GY91_Advanced - Full configuration + guided calibration for the GY-91.

  This sketch shows the parts you reach for once the basics work:
    1. Configure each sensor (IMU ranges/filter/rate, BMP280 oversampling).
    2. Run a guided calibration of everything the board supports, with on-screen
       prompts telling you exactly what to do and why.
    3. Print a copy-paste calibration block so you never have to redo it.

  Calibration, in plain terms:
    * Gyro bias   - the gyro never reads exactly zero at rest; we measure that
                    offset with the board STILL and subtract it. (1 second)
    * Accel bias  - gravity should read 1 g and nothing on the flat axes; we
                    measure the small per-axis error with the board STILL in any
                    fixed orientation (it auto-detects which way is down).
    * Magnetometer- only on genuine 9-axis boards: hard/soft-iron correction.
                    Rotate the board through every direction (a figure-8 in the
                    air) so it sees the full sphere of the Earth's field.
    * Altitude    - the BMP280 needs no calibration for pressure/temperature
                    (it is factory-trimmed), but altitude is relative to a
                    sea-level pressure. Either type your local QNH, or stand at a
                    known height and let calibrateAltitude() back-solve it.

  Open Serial Monitor at 115200 baud and follow the prompts.
*/
#include <GY91.h>

GY91 board;

// Wait for the user to press Enter in the Serial Monitor.
void waitForEnter(const char* prompt) {
  Serial.println(prompt);
  while (Serial.available()) {
    Serial.read();
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

  if (!board.begin()) {
    Serial.print("GY-91 incomplete - IMU:");
    Serial.print(board.imuOk() ? "ok" : "MISSING");
    Serial.print(" BMP280:");
    Serial.println(board.baroOk() ? "ok" : "MISSING");
  }

  // --- 1. Configure the sensors for this application ---
  // IMU: +/-4 g and +/-500 dps suit general motion; 41 Hz filter, 100 Hz rate.
  board.imu().setAccelRangeG(4);
  board.imu().setGyroRangeDps(500);
  board.imu().setLowPassFilterHz(41);
  board.imu().setSampleRateHz(100);
  // Barometer: max pressure oversampling + IIR filter for a steady altitude.
  board.baro().setSampling(/*pressure*/ BMP280::OSRS_X16, /*temp*/ BMP280::OSRS_X2,
                           BMP280::FILTER_16, BMP280::MODE_NORMAL);

  Serial.println("=== GY-91 calibration ===");

  // --- 2a. Gyro ---
  waitForEnter("\n1) Lay the board down and keep it STILL. Press Enter.");
  board.calibrateGyro(500);
  Serial.println("   gyro bias captured.");

  // --- 2b. Accelerometer ---
  waitForEnter("\n2) Keep it STILL in any fixed orientation. Press Enter.");
  board.calibrateAccel(500);
  Serial.println("   accel bias captured.");

  // --- 2c. Magnetometer (only if this board actually has one) ---
  if (board.hasMagnetometer()) {
    waitForEnter(
        "\n3) Slowly rotate the board through ALL directions (figure-8) for\n"
        "   15 seconds. Press Enter, then keep moving it...");
    if (board.calibrateMag(15000)) {
      Serial.println("   magnetometer hard/soft-iron captured.");
    } else {
      Serial.println("   mag calibration failed (not enough motion).");
    }
  } else {
    Serial.println("\n3) No magnetometer on this board - skipping.");
  }

  // --- 2d. Altitude reference ---
  waitForEnter(
      "\n4) Altitude reference: put the board at a known height (e.g. on the\n"
      "   floor = 0 m) and keep it still. Press Enter to set 0 m here.");
  if (board.calibrateAltitude(0.0f)) {
    Serial.print("   sea-level pressure set to ");
    Serial.print(board.baro().seaLevelPressureHpa(), 2);
    Serial.println(" hPa.");
  }

  // --- 3. Print the IMU calibration so you can hard-code it next time ---
  IMUCalibration c = board.getImuCalibration();
  Serial.println("\n=== Copy this into your sketch's setup() ===");
  Serial.println("IMUCalibration cal;");
  printVec("cal.accelBias  = {", c.accelBias);
  printVec("cal.accelScale = {", c.accelScale);
  printVec("cal.gyroBias   = {", c.gyroBias);
  printVec("cal.magBias    = {", c.magBias);
  printVec("cal.magScale   = {", c.magScale);
  Serial.println("board.setImuCalibration(cal);");
  Serial.print("board.setSeaLevelPressureHpa(");
  Serial.print(board.baro().seaLevelPressureHpa(), 2);
  Serial.println(");");
  Serial.println("============================================\n");
}

void loop() {
  board.update();

  Vec3 a = board.accelG();
  Vec3 g = board.gyroDps();
  Serial.print("A "); Serial.print(a.x, 2); Serial.print(",");
  Serial.print(a.y, 2); Serial.print(","); Serial.print(a.z, 2);
  Serial.print("  G "); Serial.print(g.x, 1); Serial.print(",");
  Serial.print(g.y, 1); Serial.print(","); Serial.print(g.z, 1);
  Serial.print("  Alt "); Serial.print(board.altitudeM(), 1);
  Serial.println(" m");
  delay(200);
}
