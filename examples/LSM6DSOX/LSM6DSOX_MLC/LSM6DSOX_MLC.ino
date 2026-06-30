/* LSM6DSOX_MLC - Run ST's vertical angle-detection decision tree. */
#include <LSM6DSOX.h>
#include <LSM6DSOX_AngleDetectModel.h>

LSM6DSOX imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  if (!imu.begin()) {
    Serial.println("LSM6DSOX not found.");
    while (true) delay(1000);
  }
  if (!imu.loadUcf(angle_detect, sizeof(angle_detect) / sizeof(angle_detect[0]))) {
    Serial.println("Could not load the MLC model.");
    while (true) delay(1000);
  }
  Serial.println("ST vertical angle-detection MLC is running.");
}

void loop() {
  uint8_t classification = 0;
  if (!imu.readMlcOutput(0, classification)) return;
  Serial.print("MLC0 class: ");
  Serial.println(classification);
  delay(50);
}
