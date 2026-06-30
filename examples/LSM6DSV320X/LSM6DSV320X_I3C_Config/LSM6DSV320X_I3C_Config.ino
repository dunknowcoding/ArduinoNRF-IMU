/* LSM6DSV320X_I3C_Config - Configure sensor-side I3C and IBI behavior. */
#include <LSM6DSV320X.h>

LSM6DSV320X imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!imu.begin()) {
    Serial.println("LSM6DSV320X not found over I2C.");
    while (true) {}
  }

  lsm6dsv320x_i3c_config_t config = {};
  config.if2_ta0_pid = 0;
  config.rst_mode =
      lsm6dsv320x_i3c_config_t::LSM6DSV320X_SW_RST_DYN_ADDRESS_RST;
  config.ibi_time = lsm6dsv320x_i3c_config_t::LSM6DSV320X_IBI_50us;
  int32_t status = lsm6dsv320x_i3c_config_set(imu.stContext(), config);
  status |= lsm6dsv320x_ui_i2c_i3c_mode_set(
      imu.stContext(), LSM6DSV320X_I2C_I3C_ENABLE);
  status |= imu.setI3CInterrupts(true) ? 0 : -1;
  if (status != 0) {
    Serial.println("I3C/IBI configuration failed.");
    while (true) {}
  }
  Serial.println("Sensor-side I3C/IBI configuration applied over I2C.");
}

void loop() {}
