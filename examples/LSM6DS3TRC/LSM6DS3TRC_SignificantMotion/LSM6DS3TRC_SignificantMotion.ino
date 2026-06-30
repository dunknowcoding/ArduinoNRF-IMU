/* LSM6DS3TRC_SignificantMotion - Embedded motion event routed to INT1. */
#include <LSM6DS3TRC.h>

LSM6DS3TRC imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!imu.begin() || !imu.enableSignificantMotion(1)) {
    Serial.println("LSM6DS3TR-C significant-motion startup failed.");
    while (true) {}
  }
}

void loop() {
  if (imu.significantMotionDetected()) Serial.println("Significant motion");
}
