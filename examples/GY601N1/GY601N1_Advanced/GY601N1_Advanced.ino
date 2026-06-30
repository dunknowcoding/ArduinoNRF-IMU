/* GY601N1_Advanced - Auto-detect over SPI and route INT1/INT2. */
#include <GY601N1.h>

GY601N1 imu;
constexpr uint8_t CS_PIN = 10;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!imu.beginSPI(SPI, CS_PIN)) {
    Serial.println("GY-601N1 SPI IMU not found.");
    while (true) {}
  }
  Serial.print("Detected: ");
  Serial.println(imu.coreName());
  imu.configureInterruptPin(1, true, false);
  imu.configureInterruptPin(2, true, false);
  if (imu.bmi323() != nullptr) {
    imu.bmi323()->routeDataReadyInterrupts(1, 2);
  } else {
    imu.routeDataReadyInterrupt(1);
  }
  if (imu.icm42688() != nullptr) {
    imu.icm42688()->setPin9Function(ICM42688P::Pin9Function::Interrupt2);
  }
}

void loop() {
  if (imu.dataReady() && imu.update()) {
    Serial.println(imu.gyroDps().x);
  }
}
