/* LSM6DSV_OIS - Configure the secondary OIS stream from the host UI. */
#include <LSM6DSV.h>

LSM6DSV imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!imu.begin() || !imu.configureOisInterface(true, true)) {
    Serial.println("LSM6DSV OIS startup failed.");
    while (true) {}
  }
}

void loop() {
  if (imu.oisDataReady()) Serial.println("OIS accel/gyro stream ready");
}
