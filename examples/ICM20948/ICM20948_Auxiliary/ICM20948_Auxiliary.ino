/* ICM20948_Auxiliary - Use INT, FSY, ACL/ADA, and the ADDO strap. */
#include <ICM20948.h>

ICM20948 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  // ADDO low selects 0x68; ADDO high selects 0x69.
  if (!imu.beginI2C(Wire, 0x68)) {
    Serial.println("ICM-20948 not found.");
    while (true) {}
  }
  imu.configureInterruptPin(false, false);
  imu.setDataReadyInterrupt();
  imu.setFsyncInterrupt(true, false);

  // ACL/ADA are the ICM-20948 auxiliary I2C clock/data pins.
  uint8_t externalId = 0;
  if (imu.auxReadRegister(0x0C, 0x01, externalId)) {
    Serial.print("Aux device ID: 0x");
    Serial.println(externalId, HEX);
  }
}

void loop() {
  if (imu.dataReady() && imu.update()) {
    Serial.println(imu.data().gyro.x);
  }
}
