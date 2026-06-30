/*
  MPU6050_Advanced - Configuration, calibration, data-ready and raw reads.

  Use Basic first. This sketch shows the chip-specific controls:
    * full-scale accel/gyro ranges
    * digital low-pass filter and sample rate
    * data-ready polling
    * raw register values for bus debugging
    * auxiliary I2C bypass for GY-87-style compass boards
*/
#include <MPU6050.h>

MPU6050 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!imu.begin()) {
    Serial.println("MPU-6050 not found - check wiring, power and AD0.");
    while (true) {
      delay(1000);
    }
  }

  imu.setAccelRangeG(8);
  imu.setGyroRangeDps(1000);
  imu.setLowPassFilterHz(42);
  imu.setSampleRateHz(200);
  imu.setDataReadyInterrupt(true);
  imu.configureInterruptPin(false, false);
  imu.setExternalSync(MPU6050::FSYNC_ACCEL_X);

  // If this MPU-6050 is on a GY-87 board, this exposes the compass on the main
  // I2C bus so an I2C scanner can see HMC5883L/QMC5883L after begin().
  imu.setAuxI2CBypass(true);

  Serial.println("Calibrating gyro - keep the board still...");
  imu.calibrateGyro(300);
  Serial.println("Streaming. Columns: ax ay az (g) | gx gy gz (dps) | raw ax");
}

void loop() {
  if (!imu.dataReady()) {
    return;
  }

  imu.update();

  Vec3 a = imu.accelG();
  Vec3 g = imu.gyroDps();

  MPU6050::RawSample raw;
  imu.readRaw(raw);

  Serial.print(a.x, 2); Serial.print(' ');
  Serial.print(a.y, 2); Serial.print(' ');
  Serial.print(a.z, 2); Serial.print("  |  ");
  Serial.print(g.x, 1); Serial.print(' ');
  Serial.print(g.y, 1); Serial.print(' ');
  Serial.print(g.z, 1); Serial.print("  |  ");
  Serial.println(raw.ax);
}
