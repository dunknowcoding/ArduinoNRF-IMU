/*
  QMI8658_Basic - Minimal read loop for a QMI8658 6-axis IMU.
*/
#include <QMI8658.h>

QMI8658 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!imu.begin()) {
    Serial.println("QMI8658 not found - check wiring, power and SA0.");
    while (true) {
      delay(1000);
    }
  }
}

void loop() {
  imu.update();

  Vec3 a = imu.accelG();
  Vec3 g = imu.gyroDps();

  Serial.print("A[g] ");
  Serial.print(a.x, 2); Serial.print(", ");
  Serial.print(a.y, 2); Serial.print(", ");
  Serial.print(a.z, 2);

  Serial.print("  G[dps] ");
  Serial.print(g.x, 1); Serial.print(", ");
  Serial.print(g.y, 1); Serial.print(", ");
  Serial.print(g.z, 1);

  Serial.print("  T[C] ");
  Serial.println(imu.temperatureC(), 1);

  delay(100);
}
