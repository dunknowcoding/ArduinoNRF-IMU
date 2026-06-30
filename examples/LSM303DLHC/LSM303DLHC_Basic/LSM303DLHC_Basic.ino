/*
  LSM303DLHC_Basic - Minimal read loop for an LSM303DLHC eCompass.
*/
#include <LSM303DLHC.h>

LSM303DLHC imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!imu.begin()) {
    Serial.println("LSM303DLHC not found - check wiring, power and SA0.");
    while (true) {
      delay(1000);
    }
  }

  Serial.print("LSM303DLHC ready. WHO_AM_I_A = 0x");
  Serial.println(imu.whoAmI(), HEX);
}

void loop() {
  imu.update();

  Vec3 a = imu.accelG();
  Vec3 m = imu.magUT();

  Serial.print("A[g] ");
  Serial.print(a.x, 2); Serial.print(", ");
  Serial.print(a.y, 2); Serial.print(", ");
  Serial.print(a.z, 2);

  Serial.print("  M[uT] ");
  Serial.print(m.x, 1); Serial.print(", ");
  Serial.print(m.y, 1); Serial.print(", ");
  Serial.println(m.z, 1);

  delay(100);
}
