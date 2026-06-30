/* LSM6DSV320X_EmbeddedEvents - Step counter and tilt on INT1. */
#include <LSM6DSV320X.h>

LSM6DSV320X imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!imu.begin()) {
    Serial.println("LSM6DSV320X not found.");
    while (true) {}
  }

  lsm6dsv320x_stpcnt_mode_t stepMode = {};
  stepMode.step_counter_enable = 1;
  stepMode.false_step_rej = 1;
  lsm6dsv320x_pin_int_route_emb_t route = {};
  route.step_detector = 1;
  route.tilt = 1;
  int32_t status = lsm6dsv320x_stpcnt_mode_set(imu.stContext(), stepMode);
  status |= lsm6dsv320x_tilt_mode_set(imu.stContext(), 1);
  status |= lsm6dsv320x_pin_int1_route_embedded_set(imu.stContext(), &route);
  if (status != 0) {
    Serial.println("Embedded-event configuration failed.");
    while (true) {}
  }
}

void loop() {
  uint16_t steps = 0;
  if (lsm6dsv320x_stpcnt_steps_get(imu.stContext(), &steps) != 0) return;
  Serial.print("steps=");
  Serial.println(steps);
  delay(250);
}
