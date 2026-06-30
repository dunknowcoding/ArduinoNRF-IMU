/* LSM6DS3TRC_Advanced - SPI and both interrupt pins. */
#include <LSM6DS3TRC.h>

constexpr uint8_t CS_PIN = 10;
LSM6DS3TRC imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!imu.beginSPI(SPI, CS_PIN)) {
    Serial.println("LSM6DS3TR-C SPI startup failed.");
    while (true) {}
  }
  imu.configureInterruptPins(false, false);
  imu.setDataReadyInterrupt(1, true, true);
  imu.setDataReadyInterrupt(2, true, false);
}

void loop() {
  if (imu.dataReady() && imu.update()) Serial.println(imu.data().gyro.x);
}
