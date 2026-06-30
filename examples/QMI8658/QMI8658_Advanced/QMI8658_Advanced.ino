/*
  QMI8658_Advanced - Ranges, sample rate, data-ready and raw reads.
*/
#include <QMI8658.h>

QMI8658 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!imu.begin()) {
    Serial.println("QMI8658 not found.");
    while (true) {
      delay(1000);
    }
  }

  imu.setAccelRangeG(16);
  imu.setGyroRangeDps(2048);
  imu.setSampleRateHz(500);
  // SyncSample routes level data-ready to INT2; INT1 remains handshake/events.
  imu.setSynchronizedSampleMode(true);

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

  QMI8658::RawSample raw;
  imu.readRaw(raw);

  Serial.print(a.x, 2); Serial.print(' ');
  Serial.print(a.y, 2); Serial.print(' ');
  Serial.print(a.z, 2); Serial.print("  |  ");
  Serial.print(g.x, 1); Serial.print(' ');
  Serial.print(g.y, 1); Serial.print(' ');
  Serial.print(g.z, 1); Serial.print("  |  ");
  Serial.println(raw.ax);
}
