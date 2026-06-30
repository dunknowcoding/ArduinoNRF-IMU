/* LSM6DSV320X_Fusion - SFLP game-rotation quaternion from the FIFO. */
#include <LSM6DSV320X.h>

LSM6DSV320X imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!imu.begin() || !imu.enableSflp(120)) {
    Serial.println("LSM6DSV320X SFLP startup failed.");
    while (true) {}
  }
}

void loop() {
  nimu::LSM6DSV::SflpQuaternion q;
  if (!imu.readSflpQuaternion(q)) return;
  Serial.print(q.w, 6); Serial.print(',');
  Serial.print(q.x, 6); Serial.print(',');
  Serial.print(q.y, 6); Serial.print(',');
  Serial.println(q.z, 6);
}
