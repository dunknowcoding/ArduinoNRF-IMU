/* LSM6DSV320X_Advanced - SPI, INT1 high-g DRDY, and SCX/SDX sensor hub. */
#include <LSM6DSV320X.h>

constexpr uint8_t CS_PIN = 10;
LSM6DSV320X imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!imu.beginSPI(SPI, CS_PIN) ||
      !imu.setHighGRangeG(320) ||
      !imu.setHighGSampleRateHz(7680) ||
      !imu.routeHighGDataReady(1)) {
    Serial.println("LSM6DSV320X advanced startup failed.");
    while (true) {}
  }

  imu.enableSensorHubPullups();
  uint8_t qmcId = 0;
  if (imu.sensorHubRead(0x7C, 0x00, &qmcId, 1)) {
    Serial.print("SCX/SDX device ID: 0x");
    Serial.println(qmcId, HEX);
  }
}

void loop() {
  nimu::Vec3 highG;
  if (!imu.readHighG(highG)) return;
  Serial.print(highG.x); Serial.print(',');
  Serial.print(highG.y); Serial.print(',');
  Serial.println(highG.z);
}
