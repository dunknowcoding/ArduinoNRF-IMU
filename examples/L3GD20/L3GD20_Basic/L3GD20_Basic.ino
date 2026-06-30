/* L3GD20_Basic - Minimal L3GD20/L3GD20H gyroscope example. */
#include <L3GD20.h>

L3GD20 gyro;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  if (!gyro.begin()) {
    Serial.println("L3GD20/L3GD20H not found.");
    while (true) delay(1000);
  }
}

void loop() {
  if (!gyro.update()) return;
  Vec3 g = gyro.gyroDps();
  Serial.print(g.x, 2); Serial.print(',');
  Serial.print(g.y, 2); Serial.print(',');
  Serial.println(g.z, 2);
  delay(20);
}
