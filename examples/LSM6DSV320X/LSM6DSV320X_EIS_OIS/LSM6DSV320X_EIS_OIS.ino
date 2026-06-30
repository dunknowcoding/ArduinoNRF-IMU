/* LSM6DSV320X_EIS_OIS - Configure the EIS and OIS processing channels. */
#include <LSM6DSV320X.h>

LSM6DSV320X imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!imu.begin()) {
    Serial.println("LSM6DSV320X not found.");
    while (true) {}
  }

  lsm6dsv320x_ois_chain_t ois = {};
  ois.xl = 1;
  ois.gy = 1;
  int32_t status = lsm6dsv320x_gy_eis_data_rate_set(
      imu.stContext(), LSM6DSV320X_EIS_1920Hz);
  status |= lsm6dsv320x_eis_gy_full_scale_set(
      imu.stContext(), LSM6DSV320X_EIS_2000dps);
  status |= lsm6dsv320x_ois_ctrl_mode_set(
      imu.stContext(), LSM6DSV320X_OIS_CTRL_FROM_UI);
  status |= lsm6dsv320x_ois_chain_set(imu.stContext(), ois);
  status |= lsm6dsv320x_ois_gy_full_scale_set(
      imu.stContext(), LSM6DSV320X_OIS_2000dps);
  status |= lsm6dsv320x_ois_xl_full_scale_set(
      imu.stContext(), LSM6DSV320X_OIS_16g);
  if (status != 0) {
    Serial.println("EIS/OIS configuration failed.");
    while (true) {}
  }
  Serial.println("EIS and OIS channels configured.");
}

void loop() {}
