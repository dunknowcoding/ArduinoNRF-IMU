/*
  I2C_Scanner - Find every device on the I2C bus.

  This is the first sketch to run when bringing up a new board or a new sensor.
  It walks all 7-bit addresses and reports which ones acknowledge. It is also
  the tool this library uses to exercise the ArduinoNRF core's I2C (TwoWire)
  driver and to confirm the default SDA/SCL pins actually reach your wiring.

  Wiring (ArduinoNRF ProMicro nRF52840 default I2C, "Wire"):
    SDA -> pin marked SDA   (silkscreen D6)
    SCL -> pin marked SCL   (silkscreen D7)
    VCC -> 3V3              GND -> GND

  Expected with a GY-9250 attached: a device answers at 0x68 (or 0x69), and -
  once the MPU's bypass is on - the AK8963 magnetometer at 0x0C.

  Open Serial Monitor at 115200 baud.
*/
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for the USB serial port to be ready
  }

  Wire.begin();
  Wire.setClock(400000);

  Serial.println();
  Serial.println("ArduinoNRF-IMU I2C scanner");
  Serial.print("SDA pin = ");
  Serial.print(Wire.pinSDA());
  Serial.print("   SCL pin = ");
  Serial.println(Wire.pinSCL());
  Serial.println("Scanning...");
}

void loop() {
  byte count = 0;

  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("  device found at 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      count++;
    }
  }

  if (count == 0) {
    Serial.println("  no I2C devices found");
  } else {
    Serial.print("  done, ");
    Serial.print(count);
    Serial.println(" device(s)");
  }
  Serial.println();

  delay(2000);  // scan again every 2 seconds
}
