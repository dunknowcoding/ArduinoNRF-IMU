/* MUMO_Advanced - INT, CLK/INT2 and auxiliary magnetometer access. */
#include <MUMO.h>

MUMO imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  if (!imu.begin()) {
    Serial.println("MUMO not found.");
    while (true) delay(1000);
  }

  // MUMO INT is ICM INT1; the silk CLK pad is ICM INT2.
  imu.configureInterruptPin(1, false, false);
  imu.routeDataReadyInterrupt(1, true, true);
  imu.configureInterruptPin(2, false, false);
  imu.routeDataReadyInterrupt(2, true);
}

void loop() {
  if (!imu.update()) return;
  Vec3 m = imu.magUT();
  Serial.print("mag uT ");
  Serial.print(m.x, 2); Serial.print(',');
  Serial.print(m.y, 2); Serial.print(',');
  Serial.println(m.z, 2);
}
