/* GYBNO055_Advanced - INT and the board's REST-labelled reset pad. */
#include <GYBNO055.h>

constexpr int8_t REST_PIN = 3;
constexpr int8_t INT_PIN = 2;
GYBNO055 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  imu.configurePins(REST_PIN, INT_PIN);
  imu.hardwareReset();
  if (!imu.begin()) {
    Serial.println("GY-BNO055 not found.");
    while (true) {}
  }
  imu.setExternalCrystal(true);
  imu.setMode(BNO055::MODE_NDOF);
}

void loop() {
  if (imu.interruptAsserted()) {
    Serial.print("INT source: 0x");
    Serial.println(imu.interruptStatus(), HEX);
    imu.clearInterrupt();
  }
  const BNO055::Quaternion q = imu.quaternion();
  Serial.print(q.w, 4); Serial.print(',');
  Serial.print(q.x, 4); Serial.print(',');
  Serial.print(q.y, 4); Serial.print(',');
  Serial.println(q.z, 4);
  delay(100);
}
