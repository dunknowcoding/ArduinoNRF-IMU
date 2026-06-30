/* LSM303_L3GD20_Advanced - Identify chips and route exposed interrupt pads. */
#include <LSM303_L3GD20.h>

LSM303_L3GD20 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  if (!imu.begin()) {
    Serial.print("LSM303 available: "); Serial.println(imu.motionOk());
    Serial.print("L3GD20 available: "); Serial.println(imu.gyroOk());
    while (true) delay(1000);
  }
  Serial.print("LSM303 accel ID=0x"); Serial.println(imu.motion().whoAmI(), HEX);
  Serial.print("gyro ID=0x"); Serial.println(imu.gyro().whoAmI(), HEX);
  imu.motion().configureAccelInterruptPolarity(false);
  imu.motion().setAccelDataReadyInterrupt(true);
  imu.gyro().configureInterruptPins(false, false);
  imu.gyro().setDataReadyInterrupt(true);
  imu.gyro().setThresholdInterrupt(true);
}

void loop() {
  if (!imu.motion().dataReady() || !imu.gyro().dataReady()) return;
  if (!imu.update()) return;
  Serial.print("mag DRDY="); Serial.println(imu.motion().magDataReady());
}
