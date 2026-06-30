/*
  GY521_Advanced - Configuration, calibration, data-ready and raw reads.
*/
#include <GY521.h>

GY521 board;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!board.begin()) {
    Serial.println("GY-521 not found - check wiring, power and AD0.");
    while (true) {
      delay(1000);
    }
  }

  board.setAccelRangeG(8);
  board.setGyroRangeDps(1000);
  board.setLowPassFilterHz(42);
  board.setSampleRateHz(200);
  board.setDataReadyInterrupt(true);
  board.configureInterruptPin(false, false);

  Serial.println("Calibrating gyro - keep the board still...");
  board.calibrateGyro(300);
  Serial.println("Streaming. Columns: ax ay az (g) | gx gy gz (dps) | raw ax");
}

void loop() {
  if (!board.dataReady()) {
    return;
  }

  board.update();

  Vec3 a = board.accelG();
  Vec3 g = board.gyroDps();

  GY521::RawSample raw;
  board.readRaw(raw);

  Serial.print(a.x, 2); Serial.print(' ');
  Serial.print(a.y, 2); Serial.print(' ');
  Serial.print(a.z, 2); Serial.print("  |  ");
  Serial.print(g.x, 1); Serial.print(' ');
  Serial.print(g.y, 1); Serial.print(' ');
  Serial.print(g.z, 1); Serial.print("  |  ");
  Serial.println(raw.ax);
}
