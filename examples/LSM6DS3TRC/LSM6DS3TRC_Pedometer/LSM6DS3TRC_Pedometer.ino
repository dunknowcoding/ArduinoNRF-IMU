/* LSM6DS3TRC_Pedometer - Embedded step counter routed to INT1. */
#include <LSM6DS3TRC.h>

LSM6DS3TRC imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!imu.begin() || !imu.enablePedometer(1)) {
    Serial.println("LSM6DS3TR-C pedometer startup failed.");
    while (true) {}
  }
  imu.resetStepCount();
}

void loop() {
  if (imu.stepDetected()) {
    Serial.print("Steps: ");
    Serial.println(imu.stepCount());
  }
}
