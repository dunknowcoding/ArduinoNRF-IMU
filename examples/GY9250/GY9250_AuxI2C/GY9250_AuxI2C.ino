/*
  GY9250_AuxI2C - Reach external I2C sensors through the MPU's aux bus (EDA/ECL).

  The MPU-9250 has a SECOND I2C bus on its EDA/ECL pins, driven by the chip's
  own internal I2C master (it is how the on-chip AK8963 magnetometer is reached).
  On a GY-9250 breakout these pins are broken out, so you can wire your OWN I2C
  sensors to EDA/ECL and read them THROUGH the MPU - handy to keep extra sensors
  on a separate, isolated bus, or to fan more sensors off one MCU.

  This sketch scans the aux bus and identifies what it finds. It was verified
  with a GY-91 (an MPU-6500 at 0x68 + a BMP280 at 0x76) wired to EDA/ECL: the
  AK8963 at 0x0C plus both GY-91 chips all answer.

  Wiring:
    MCU  <-(main I2C)->  GY-9250:  SDA->SDA(D6)  SCL->SCL(D7)  3V3  GND
                                   AD0->GND  (sets the MPU address to 0x68)
    GY-9250 EDA -> your sensor's SDA
    GY-9250 ECL -> your sensor's SCL
    (the external sensor also needs 3V3 + GND; its bus pull-ups serve the aux bus)

  Open Serial Monitor at 115200 baud.
*/
#include <MPU9250.h>

MPU9250 imu;

void identify(uint8_t addr) {
  Serial.print("  0x");
  if (addr < 16) Serial.print("0");
  Serial.print(addr, HEX);
  uint8_t v = 0;
  if (addr == 0x0C && imu.auxReadRegister(0x0C, 0x00, v)) {
    Serial.print("  AK8963 magnetometer (WIA=0x"); Serial.print(v, HEX); Serial.print(")");
  } else if ((addr == 0x68 || addr == 0x69) && imu.auxReadRegister(addr, 0x75, v)) {
    Serial.print("  MPU-6xxx/9xxx (WHO_AM_I=0x"); Serial.print(v, HEX); Serial.print(")");
  } else if ((addr == 0x76 || addr == 0x77) && imu.auxReadRegister(addr, 0xD0, v)) {
    Serial.print("  BMP280/BME280 (chip id=0x"); Serial.print(v, HEX); Serial.print(")");
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  // begin() over I2C enables the MPU's internal master, which drives EDA/ECL.
  if (!imu.begin()) {
    Serial.println("GY-9250 not found on the main I2C bus.");
    while (true) {
      delay(1000);
    }
  }
  Serial.print("GY-9250 ready (WHO=0x");
  Serial.print(imu.whoAmI(), HEX);
  Serial.println("). Scanning the aux bus (EDA/ECL)...");
}

void loop() {
  byte count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    if (imu.auxPing(addr)) {
      identify(addr);
      count++;
    }
  }
  Serial.print(count);
  Serial.println(count == 1 ? " device on EDA/ECL.\n" : " devices on EDA/ECL.\n");

  delay(3000);
}
