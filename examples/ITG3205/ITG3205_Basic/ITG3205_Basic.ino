/*
  ITG3205_Basic - Minimal read loop for an ITG-3200/ITG-3205 gyroscope.
*/
#include <ITG3205.h>

ITG3205 gyro;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!gyro.begin()) {
    Serial.println("ITG3205 not found - check wiring, power and AD0.");
    while (true) {
      delay(1000);
    }
  }
}

void loop() {
  gyro.update();
  Vec3 g = gyro.gyroDps();

  Serial.print("G[dps] ");
  Serial.print(g.x, 1); Serial.print(", ");
  Serial.print(g.y, 1); Serial.print(", ");
  Serial.print(g.z, 1);
  Serial.print("  T[C] ");
  Serial.println(gyro.temperatureC(), 1);

  delay(100);
}
