/*
  GY86_Advanced - Configure/calibrate GY-86 IMU, MS5611 and compass.
*/
#include <GY86.h>

GY86 board;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!board.begin()) {
    Serial.println("GY-86 IMU/barometer not found.");
    while (true) {
      delay(1000);
    }
  }

  Serial.print("MPU6050:");
  Serial.print(board.imuOk() ? "ok" : "missing");
  Serial.print(" MS5611:");
  Serial.print(board.baroOk() ? "ok" : "missing");
  Serial.print(" compass:");
  Serial.println(board.compassName());

  board.imu().setAccelRangeG(8);
  board.imu().setGyroRangeDps(1000);
  board.imu().setLowPassFilterHz(42);
  board.imu().setSampleRateHz(100);
  board.baro().setOversampling(MS5611::OSR_4096);

  Serial.println("Calibrating accel and gyro - keep the board still...");
  board.calibrateAccel(200);
  board.calibrateGyro(200);
  Serial.println("Streaming. Columns: ax ay az (g) | gx gy gz (dps) | pressure hPa");
}

void loop() {
  board.update();

  Vec3 a = board.accelG();
  Vec3 g = board.gyroDps();

  Serial.print(a.x, 2); Serial.print(' ');
  Serial.print(a.y, 2); Serial.print(' ');
  Serial.print(a.z, 2); Serial.print("  |  ");
  Serial.print(g.x, 1); Serial.print(' ');
  Serial.print(g.y, 1); Serial.print(' ');
  Serial.print(g.z, 1); Serial.print("  |  ");
  Serial.println(board.pressureHpa(), 2);

  delay(20);
}
