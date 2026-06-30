/*
  GY271_Advanced - Variant detection and HMC5883L gain setup.
*/
#include <GY271.h>

GY271 compass;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!compass.begin()) {
    Serial.println("GY-271 compass not found.");
    while (true) {
      delay(1000);
    }
  }

  Serial.print("Compass variant: ");
  Serial.println(compass.compassName());

  if (compass.compassKind() == GY271::CompassKind::HMC5883L) {
    compass.hmc().setGainGauss(1.3f);
    compass.hmc().setContinuousMode();
  }

  Serial.println("Streaming. Columns: mx my mz (uT)");
}

void loop() {
  compass.update();
  Vec3 m = compass.magUT();

  Serial.print(m.x, 1); Serial.print(' ');
  Serial.print(m.y, 1); Serial.print(' ');
  Serial.println(m.z, 1);

  delay(100);
}
