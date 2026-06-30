/* LSM6DSV_Fusion - On-chip SFLP game-rotation quaternion. */
#include <LSM6DSV.h>

LSM6DSV imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!imu.begin() || !imu.enableSflp(120)) {
    Serial.println("LSM6DSV SFLP startup failed.");
    while (true) {}
  }
}

void loop() {
  LSM6DSV::SflpQuaternion q;
  if (imu.readSflpQuaternion(q)) {
    Serial.print(q.w, 5);
    Serial.print(',');
    Serial.print(q.x, 5);
    Serial.print(',');
    Serial.print(q.y, 5);
    Serial.print(',');
    Serial.println(q.z, 5);
  }
}
