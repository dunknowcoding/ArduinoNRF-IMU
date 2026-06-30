/* LSM6DS3TRC_Tilt - Embedded tilt event routed to INT2. */
#include <LSM6DS3TRC.h>

LSM6DS3TRC imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!imu.begin() || !imu.enableTiltDetection(2)) {
    Serial.println("LSM6DS3TR-C tilt startup failed.");
    while (true) {}
  }
}

void loop() {
  if (imu.tiltDetected()) Serial.println("Tilt");
}
