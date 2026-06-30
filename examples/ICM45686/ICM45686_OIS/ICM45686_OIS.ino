/* ICM45686_OIS - Select AUX1 SPI-slave mode on OCS/SCX/SDX/OSDO. */
#include <ICM45686.h>

ICM45686 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!imu.begin() || !imu.enableAuxOisSpi()) {
    Serial.println("ICM-45686 AUX1 OIS mode failed.");
    while (true) {}
  }
  Serial.println(imu.auxiliaryMode() == ICM45686::AuxiliaryMode::OisSpi
                     ? "AUX1 OIS SPI selected"
                     : "AUX1 mode mismatch");
}

void loop() {}
