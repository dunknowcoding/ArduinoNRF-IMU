/*
  GY9250_DataReadyInterrupt - Read exactly when a fresh sample exists.

  Instead of polling on delay(), the MPU pulses its INT pin once per new sample.
  We attach an interrupt to that pin, and the main loop reads only when the flag
  is set - no missed samples, no wasted SPI traffic, perfect cadence.

  This works the same over I2C; SPI is shown because it is the usual choice when
  you care about fast, jitter-free sampling.

  Wiring (IMU silk -> ProMicro nRF52840 silk):
    SCLK -> SCK (D2)   SDI -> MOSI (D4)   SDO -> MISO (D3)   NCS -> CS (D5/SS)
    INT  -> D0         VCC -> 3V3         GND -> GND

  Open Serial Monitor at 115200 baud.
*/
#include <GY9250.h>
#include <SPI.h>

GY9250 imu;

const uint8_t INT_PIN = 0;          // IMU INT -> D0

volatile uint32_t sampleCount = 0;
volatile bool dataReady = false;
void onDataReady() {                 // keep ISRs tiny: flag and count only
  dataReady = true;
  sampleCount++;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!imu.beginSPI(SPI, SS)) {
    Serial.println("GY-9250 not found on SPI.");
    while (true) {
      delay(1000);
    }
  }

  imu.setSampleRateHz(100);          // one INT pulse every 10 ms
  imu.calibrateGyro();

  pinMode(INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), onDataReady, RISING);
  imu.setDataReadyInterrupt(true);   // drive INT on each new sample

  Serial.println("Reading on the data-ready interrupt...");
}

uint32_t lastReport = 0;
uint32_t readsThisSecond = 0;

void loop() {
  if (dataReady) {
    dataReady = false;
    imu.update();                    // fresh sample guaranteed
    readsThisSecond++;
  }

  // Once a second, report the effective sample rate (should track 100 Hz).
  if (millis() - lastReport >= 1000) {
    lastReport = millis();
    Vec3 a = imu.accelG();
    Serial.print("rate ");
    Serial.print(readsThisSecond);
    Serial.print(" Hz   A[g] ");
    Serial.print(a.x, 2); Serial.print(", ");
    Serial.print(a.y, 2); Serial.print(", ");
    Serial.println(a.z, 2);
    readsThisSecond = 0;
  }
}
