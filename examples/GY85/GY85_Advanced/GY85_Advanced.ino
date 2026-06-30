/*
  GY85_Advanced - Configure/calibrate a GY-85 module and show raw reads.
*/
#include <GY85.h>

GY85 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!imu.begin()) {
    Serial.println("GY-85 accel/gyro not found.");
    while (true) {
      delay(1000);
    }
  }

  imu.accel().setAccelRangeG(16);
  imu.accel().setSampleRateHz(100);
  imu.gyro().setSampleRateHz(100);
  imu.gyro().setLowPassFilterHz(42);

  Serial.print("Compass: ");
  Serial.println(imu.compassName());
  Serial.println("Calibrating accel and gyro - keep the board still...");
  imu.calibrateAccel(200);
  imu.calibrateGyro(200);
  Serial.println("Streaming. Columns: ax ay az (g) | gx gy gz (dps) | raw ax gx");
}

void loop() {
  imu.update();

  Vec3 a = imu.accelG();
  Vec3 g = imu.gyroDps();

  ADXL345::RawSample ar;
  ITG3205::RawSample gr;
  imu.accel().readRaw(ar);
  imu.gyro().readRaw(gr);

  Serial.print(a.x, 2); Serial.print(' ');
  Serial.print(a.y, 2); Serial.print(' ');
  Serial.print(a.z, 2); Serial.print("  |  ");
  Serial.print(g.x, 1); Serial.print(' ');
  Serial.print(g.y, 1); Serial.print(' ');
  Serial.print(g.z, 1); Serial.print("  |  ");
  Serial.print(ar.ax); Serial.print(' ');
  Serial.println(gr.gx);

  delay(20);
}
