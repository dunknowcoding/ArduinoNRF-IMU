/* LSM6DSV320X_FIFO - Tagged low-g, gyro, and high-g FIFO stream. */
#include <LSM6DSV320X.h>

LSM6DSV320X imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!imu.begin()) {
    Serial.println("LSM6DSV320X not found.");
    while (true) {}
  }

  stmdev_ctx_t* ctx = imu.stContext();
  int32_t status = 0;
  status |= lsm6dsv320x_fifo_watermark_set(ctx, 16);
  status |= lsm6dsv320x_fifo_xl_batch_set(
      ctx, LSM6DSV320X_XL_BATCHED_AT_120Hz);
  status |= lsm6dsv320x_fifo_gy_batch_set(
      ctx, LSM6DSV320X_GY_BATCHED_AT_120Hz);
  status |= lsm6dsv320x_fifo_hg_xl_batch_set(ctx, 1);
  status |= lsm6dsv320x_fifo_mode_set(ctx, LSM6DSV320X_STREAM_MODE);
  if (status != 0) {
    Serial.println("FIFO configuration failed.");
    while (true) {}
  }
}

void loop() {
  lsm6dsv320x_fifo_status_t status = {};
  if (lsm6dsv320x_fifo_status_get(imu.stContext(), &status) != 0) return;
  while (status.fifo_level-- > 0) {
    lsm6dsv320x_fifo_out_raw_t sample = {};
    if (lsm6dsv320x_fifo_out_raw_get(imu.stContext(), &sample) != 0) return;
    Serial.print("tag="); Serial.print(static_cast<uint8_t>(sample.tag), HEX);
    Serial.print(" count="); Serial.println(sample.cnt);
  }
}
