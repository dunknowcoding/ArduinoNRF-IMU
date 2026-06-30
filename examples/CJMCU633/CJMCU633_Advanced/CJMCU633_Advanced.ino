/* CJMCU633_Advanced - INT1/2 and SCX/SDX sensor-hub access. */
#include <CJMCU633.h>

CJMCU633 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!imu.begin()) {
    Serial.println("CJMCU-633 not found.");
    while (true) {}
  }
  imu.configureInterruptPins(false, false);
  imu.setDataReadyInterrupt(1);
  imu.enableSensorHubPullups();

  uint8_t externalId = 0;
  if (imu.sensorHubRead(0x1E, 0x0F, &externalId, 1)) {
    Serial.print("SCX/SDX device ID: 0x");
    Serial.println(externalId, HEX);
  }
}

void loop() {
  if (imu.dataReady() && imu.update()) Serial.println(imu.data().gyro.x);
}
