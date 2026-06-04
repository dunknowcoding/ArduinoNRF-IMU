/*
  MPU9250_Advanced - Custom ranges, filtering, data-ready and raw values.

  Shows the chip-specific configuration on top of the common API:
    * pick full-scale ranges and an output data rate
    * set the digital low-pass (anti-alias) bandwidth
    * read only when a new sample is ready (data-ready flag)
    * read the unscaled register values for bus debugging

  Wiring: same default I2C as the other examples (SDA=D6, SCL=D7).
  Open Serial Monitor at 115200 baud.
*/
#include <MPU9250.h>

MPU9250 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!imu.begin()) {
    Serial.println("MPU9250 not found.");
    while (true) {
      delay(1000);
    }
  }

  // --- Configure the chip for a fast, wide-range setup ---
  imu.setAccelRangeG(16);        // +/-16 g  (impacts, shocks)
  imu.setGyroRangeDps(2000);     // +/-2000 deg/s (fast spins)
  imu.setLowPassFilterHz(92);    // 92 Hz anti-alias bandwidth
  imu.setSampleRateHz(200);      // 200 samples per second

  // Drive the data-ready flag so loop() reads only fresh samples.
  imu.setDataReadyInterrupt(true);

  // Keep still while we remove gyro drift at the new range.
  Serial.println("Calibrating gyro - hold still...");
  imu.calibrateGyro(300);

  Serial.println("Streaming. Columns: ax ay az (g) | gx gy gz (dps) | raw ax");
}

void loop() {
  // Poll the chip's data-ready flag instead of free-running with delay().
  if (!imu.dataReady()) {
    return;
  }

  imu.update();
  Vec3 a = imu.accelG();
  Vec3 g = imu.gyroDps();

  // The raw struct exposes the unscaled 16-bit registers - handy when you are
  // debugging the bus and want to see exactly what came off the wire.
  MPU9250::RawSample raw;
  imu.readRaw(raw);

  Serial.print(a.x, 2); Serial.print(' ');
  Serial.print(a.y, 2); Serial.print(' ');
  Serial.print(a.z, 2); Serial.print("  |  ");
  Serial.print(g.x, 1); Serial.print(' ');
  Serial.print(g.y, 1); Serial.print(' ');
  Serial.print(g.z, 1); Serial.print("  |  ");
  Serial.println(raw.ax);
}
