/* CJMCU055_Advanced - PS and all three chip-select pins in SPI mode. */
#include <CJMCU055.h>

constexpr uint8_t PS_PIN = 2;
constexpr uint8_t CSB1_PIN = 3;
constexpr uint8_t CSB2_PIN = 4;
constexpr uint8_t CSB3_PIN = 5;

CJMCU055 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  CJMCU055::selectSPI(PS_PIN, CSB1_PIN, CSB2_PIN, CSB3_PIN);
  if (!imu.beginSPI(SPI, CSB1_PIN, CSB2_PIN, CSB3_PIN)) {
    Serial.println("CJMCU-055 SPI startup failed.");
    while (true) {}
  }
  imu.setAccelRangeG(8);
  imu.setGyroRangeDps(1000);
}

void loop() {
  if (imu.dataReady() && imu.update()) {
    const IMUData& d = imu.data();
    Serial.print(d.accel.x);
    Serial.print(',');
    Serial.print(d.gyro.x);
    Serial.print(',');
    Serial.println(d.mag.x);
  }
}
