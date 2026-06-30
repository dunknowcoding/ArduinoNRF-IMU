/* ICM45686_QMC6309 - Read a QMC6309 connected to SCX/SDX. */
#include <ICM45686.h>

ICM45686 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!imu.begin() || !imu.configureQMC6309()) {
    Serial.println("ICM-45686 auxiliary QMC6309 startup failed.");
    while (true) {}
  }
}

void loop() {
  Vec3 magneticField;
  if (!imu.update() || !imu.readQMC6309(magneticField)) return;
  Serial.print(magneticField.x); Serial.print(',');
  Serial.print(magneticField.y); Serial.print(',');
  Serial.println(magneticField.z);
  delay(20);
}
