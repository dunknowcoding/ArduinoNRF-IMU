/*
  GY9250_FSYNC - Time-stamp an external event against the IMU samples.

  The MPU's FSYNC pin lets an external signal (a camera shutter, another sensor's
  sync output, a button...) be captured INSIDE the sample stream: its level is
  latched into the least-significant bit of one output register every sample, so
  you know exactly which IMU sample lines up with the event - no separate clock.

  setExternalSync() routes FSYNC into a register (here the temperature LSB, the
  least useful bit to sacrifice); after each update() the captured level is read
  with fsyncLevel() (0/1, or -1 if FSYNC is off).

  This sketch SIMULATES the external event by toggling D1 (the pin wired to
  FSYNC): every second it pulses D1 and you will see fsyncLevel() follow. In a
  real system you would instead wire your event source to FSYNC and leave D1
  alone.

  Wiring (IMU silk -> ProMicro nRF52840 silk):
    SCLK -> SCK (D2)   SDI -> MOSI (D4)   SDO -> MISO (D3)   NCS -> CS (D5/SS)
    FSYNC -> D1        VCC -> 3V3         GND -> GND

  Open Serial Monitor at 115200 baud.
*/
#include <GY9250.h>
#include <SPI.h>

GY9250 imu;

const uint8_t FSYNC_SIM_PIN = 1;    // FSYNC -> D1 (driven here to simulate events)

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

  imu.setSampleRateHz(100);
  // Route FSYNC into the temperature LSB. After this, fsyncLevel() is live.
  imu.setExternalSync(MPU6500::FSYNC_TEMP);

  // Drive D1 ourselves so the demo is self-contained. Remove this in a real
  // application and connect your event source to FSYNC instead.
  pinMode(FSYNC_SIM_PIN, OUTPUT);
  digitalWrite(FSYNC_SIM_PIN, LOW);

  Serial.println("Pulsing a simulated event on FSYNC once per second...");
}

void loop() {
  // Assert the simulated external event for 100 ms, then release it.
  static uint32_t lastEvent = 0;
  bool eventActive = (millis() - lastEvent) < 100;
  if (millis() - lastEvent >= 1000) {
    lastEvent = millis();
    eventActive = true;
  }
  digitalWrite(FSYNC_SIM_PIN, eventActive ? HIGH : LOW);

  imu.update();
  int fsync = imu.fsyncLevel();      // 0 or 1, captured with this sample

  if (fsync == 1) {
    Vec3 a = imu.accelG();
    Serial.print("EVENT @ sample - A[g] ");
    Serial.print(a.x, 2); Serial.print(", ");
    Serial.print(a.y, 2); Serial.print(", ");
    Serial.println(a.z, 2);
  }

  delay(10);
}
