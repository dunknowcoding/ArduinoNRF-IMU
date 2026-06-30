/* BNO08x_HostPins - Host INT, RST, and WAK control on BNO085/BNO086. */
#include <BNO08x.h>

constexpr int IMU_INT_PIN = 2;
constexpr int IMU_RST_PIN = 3;
constexpr int IMU_WAK_PIN = 4;
BNO08x imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  imu.configurePins(IMU_INT_PIN, IMU_RST_PIN, IMU_WAK_PIN);
  if (!imu.beginI2C(Wire, 0x4B)) {
    Serial.println("BNO08x not found.");
    while (true) {}
  }
  imu.setSampleRateHz(100);
}

void loop() {
  if (!imu.interruptAsserted() || !imu.update()) return;
  Vec3 a = imu.accelG();
  Serial.print(a.x, 3); Serial.print(',');
  Serial.print(a.y, 3); Serial.print(',');
  Serial.println(a.z, 3);
}
