/*
  GY80_Advanced - Configure/calibrate a GY-80/GY-801 module and show raw reads.
*/
#include <GY80.h>

GY80 board;
constexpr uint8_t P_XCLR_PIN = 8;
constexpr uint8_t P_EOC_PIN = 9;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!board.begin()) {
    Serial.println("GY-80 core sensors not found.");
    while (true) {
      delay(1000);
    }
  }

  Serial.print("ADXL345:");
  Serial.print(board.accelOk() ? "ok" : "missing");
  Serial.print(" L3G4200D:");
  Serial.print(board.gyroOk() ? "ok" : "missing");
  Serial.print(" BMP085/BMP180:");
  Serial.print(board.baroOk() ? "ok" : "missing");
  Serial.print(" compass:");
  Serial.println(board.compassName());

  board.accel().setAccelRangeG(16);
  board.accel().setSampleRateHz(100);
  board.gyro().setGyroRangeDps(2000);
  board.gyro().setSampleRateHz(200);
  board.gyro().setLowPassFilterHz(50);
  board.baro().setOversampling(BMP180::OSS_HIGH_RES);
  board.configurePressurePins(P_XCLR_PIN, P_EOC_PIN);
  board.resetPressure();
  board.accel().routeInterrupt(ADXL345::INTERRUPT_DATA_READY, 1);
  board.gyro().setThresholdInterrupt();

  Serial.println("Calibrating accel and gyro - keep the board still...");
  board.calibrateAccel(200);
  board.calibrateGyro(200);
  Serial.println("Streaming. Columns: ax ay az (g) | gx gy gz (dps) | raw ax gx");
}

void loop() {
  board.update();
  bool compassReady = board.magOk() &&
      (board.compassKind() == GY80::CompassKind::HMC5883L
           ? board.hmc().dataReady()
           : board.qmc().dataReady());

  Vec3 a = board.accelG();
  Vec3 g = board.gyroDps();

  ADXL345::RawSample ar;
  L3G4200D::RawSample gr;
  board.accel().readRaw(ar);
  board.gyro().readRaw(gr);

  Serial.print(a.x, 2); Serial.print(' ');
  Serial.print(a.y, 2); Serial.print(' ');
  Serial.print(a.z, 2); Serial.print("  |  ");
  Serial.print(g.x, 1); Serial.print(' ');
  Serial.print(g.y, 1); Serial.print(' ');
  Serial.print(g.z, 1); Serial.print("  |  ");
  Serial.print(ar.ax); Serial.print(' ');
  Serial.print(gr.gx); Serial.print("  |  ");
  Serial.print(compassReady); Serial.print(' ');
  Serial.println(board.pressureConversionComplete());

  delay(20);
}
