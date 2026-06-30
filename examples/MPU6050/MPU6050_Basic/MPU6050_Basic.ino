/*
  MPU6050_Basic - Minimal read loop for a 6-axis MPU-6050 / GY-521 module.
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

  Serial.print("MPU-6050 ready. WHO_AM_I = 0x");
  Serial.println(imu.whoAmI(), HEX);
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
