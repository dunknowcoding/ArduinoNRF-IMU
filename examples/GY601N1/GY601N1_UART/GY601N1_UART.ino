/* GY601N1_UART - Inspect the onboard MCU's processed UART byte stream. */
#include <GY601N1.h>

GY601N1 imu;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  imu.beginUART(Serial1);
}

void loop() {
  while (imu.uartAvailable() > 0) {
    int value = imu.readUART();
    if (value < 0x10) Serial.print('0');
    Serial.print(value, HEX);
    Serial.print(' ');
  }
}
