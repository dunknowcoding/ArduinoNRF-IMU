/*
  GY87_Advanced - Configure and calibrate a GY-87 marketplace module.
*/
#include <GY87.h>

GY87 board;

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

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  board.begin();
  Serial.print("IMU:");
  Serial.print(board.imuOk() ? "ok" : "MISSING");
  Serial.print(" BMP180:");
  Serial.print(board.baroOk() ? "ok" : "MISSING");
  Serial.print(" compass:");
  Serial.println(board.compassName());

  board.imu().setAccelRangeG(8);
  board.imu().setGyroRangeDps(1000);
  board.imu().setLowPassFilterHz(42);
  board.imu().setSampleRateHz(100);
  board.baro().setOversampling(BMP180::OSS_HIGH_RES);

  waitForEnter("Lay the board still, then press Enter for gyro calibration.");
  board.calibrateGyro(500);

  waitForEnter("Keep it still in any fixed orientation, then press Enter.");
  board.calibrateAccel(500);

  waitForEnter("Place it at a known 0 m reference height, then press Enter.");
  board.calibrateAltitude(0.0f);
}

void loop() {
  board.update();

  Vec3 a = board.accelG();
  Vec3 g = board.gyroDps();
  Vec3 m = board.magUT();

  Serial.print("A ");
  Serial.print(a.x, 2); Serial.print(",");
  Serial.print(a.y, 2); Serial.print(",");
  Serial.print(a.z, 2);
  Serial.print(" G ");
  Serial.print(g.x, 1); Serial.print(",");
  Serial.print(g.y, 1); Serial.print(",");
  Serial.print(g.z, 1);
  if (board.hasMagnetometer()) {
    Serial.print(" M ");
    Serial.print(m.x, 1); Serial.print(",");
    Serial.print(m.y, 1); Serial.print(",");
    Serial.print(m.z, 1);
  }
  Serial.print(" Alt ");
  Serial.println(board.altitudeM(), 1);
  delay(200);
}
