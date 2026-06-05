#include "MPU9250.h"

namespace nimu {
using namespace mpu6500;
using namespace mpu9250;

namespace {
/** Combine two bytes, low byte first (AK8963 word order). */
inline int16_t le16(const uint8_t* p) {
  return static_cast<int16_t>((static_cast<uint16_t>(p[1]) << 8) | p[0]);
}
}  // namespace

// ------------------------------------------------------------- magnetometer --

bool MPU9250::initExtras() { return initMagnetometer(); }

bool MPU9250::initMagnetometer() {
  magPresent_ = false;
  magEnabled_ = false;
  hasMag_ = false;
  if (i2cWire_ == nullptr) {
    return true;  // SPI mode: no magnetometer path, but begin() still succeeds
  }

  // Make sure bypass is OFF, then (re)start the MPU's internal I2C master on the
  // auxiliary bus. The AK8963 is reached ONLY through this master, so a
  // dead/absent magnetometer can never jam the main accel/gyro bus.
  bus_.writeRegister(INT_PIN_CFG, 0x00);  // bypass disabled
  bus_.updateRegister(USER_CTRL, USERCTRL_I2C_MST_RST, USERCTRL_I2C_MST_RST);
  delay(10);
  bus_.updateRegister(USER_CTRL, USERCTRL_I2C_MST_EN, USERCTRL_I2C_MST_EN);
  bus_.writeRegister(I2C_MST_CTRL, I2C_MST_WAIT_FOR_ES | I2C_MST_CLK_400);
  delay(10);

  // Probe over the aux bus. A board without a working AK8963 just NACKs here;
  // we return success with magPresent_ = false so accel + gyro still run.
  uint8_t wia = 0;
  if (!magReadReg(MAG_WIA, wia) || wia != kMagWhoAmI) {
    return true;
  }

  // Soft-reset the AK8963, then read its fuse-ROM sensitivity adjustment.
  magWriteReg(MAG_CNTL2, 0x01);
  delay(10);
  magWriteReg(MAG_CNTL1, MAG_MODE_POWERDOWN);
  delay(10);
  magWriteReg(MAG_CNTL1, MAG_MODE_FUSE_ROM);
  delay(10);
  uint8_t asa[3] = {128, 128, 128};
  magReadReg(MAG_ASAX, asa[0]);
  magReadReg(MAG_ASAX + 1, asa[1]);
  magReadReg(MAG_ASAX + 2, asa[2]);
  // Per datasheet: adjusted = raw * ((ASA - 128) / 256 + 1).
  magAsa_ = Vec3{(asa[0] - 128) / 256.0f + 1.0f, (asa[1] - 128) / 256.0f + 1.0f,
                 (asa[2] - 128) / 256.0f + 1.0f};
  magWriteReg(MAG_CNTL1, MAG_MODE_POWERDOWN);
  delay(10);

  magPresent_ = true;
  return enableMagnetometer(true);
}

bool MPU9250::enableMagnetometer(bool enable) {
  if (!magPresent_) {
    return false;
  }
  magWriteReg(MAG_CNTL1, MAG_MODE_POWERDOWN);
  delay(10);
  if (enable) {
    // Continuous 100 Hz, 16-bit resolution.
    magWriteReg(MAG_CNTL1, MAG_MODE_CONT_100HZ | MAG_BIT_16);
    delay(10);
  }
  // Drive (or stop) the automatic per-sample reads of the AK8963 data block.
  magConfigureSlv0(enable);
  magEnabled_ = enable;
  hasMag_ = enable;
  return true;
}

uint8_t MPU9250::magWhoAmI() {
  if (i2cWire_ == nullptr) {
    return 0;
  }
  uint8_t wia = 0;
  magReadReg(MAG_WIA, wia);
  return wia;
}

// -------------------------------------------------------------- data read ---

bool MPU9250::readRaw(RawSample& out) {
  if (!MPU6500::readRaw(out)) {  // accel/gyro/temp (clears the mag fields)
    return false;
  }
  if (magEnabled_) {
    readMagRaw(out.mx, out.my, out.mz, out.magValid);
  }
  return true;
}

void MPU9250::processExtra(const RawSample& raw) {
  if (!raw.magValid) {
    return;
  }
  // The AK8963 die is rotated relative to the gyro/accel die. Remap its axes
  // into the MPU body frame (X<->Y swapped, Z negated) so all three sensors
  // share one coordinate system, then apply hard/soft-iron calibration.
  float axu = raw.mx * magAsa_.x * kMagScaleUT;
  float ayu = raw.my * magAsa_.y * kMagScaleUT;
  float azu = raw.mz * magAsa_.z * kMagScaleUT;
  Vec3 m{ayu, axu, -azu};
  data_.mag = correct(m, cal_.magBias, cal_.magScale);
}

bool MPU9250::readMagRaw(int16_t& mx, int16_t& my, int16_t& mz, bool& valid) {
  valid = false;
  mx = my = mz = 0;
  if (!magEnabled_) {
    return false;
  }
  // SLV0 mirrors the AK8963's ST1 + 6 data bytes + ST2 into EXT_SENS_DATA each
  // sample, so a fresh magnetometer reading is just one main-bus burst read.
  // buf: [0]=ST1, [1..6]=HXL..HZH, [7]=ST2.
  uint8_t buf[8];
  if (bus_.readRegisters(EXT_SENS_DATA_00, buf, 8) != IMUStatus::Ok) {
    return false;
  }
  if (!(buf[0] & MAG_ST1_DRDY)) {
    return true;  // no new sample yet - not an error, just nothing fresh
  }
  if (buf[7] & MAG_ST2_HOFL) {
    return true;  // magnetic overflow this sample; values discarded
  }
  mx = le16(&buf[1]);
  my = le16(&buf[3]);
  mz = le16(&buf[5]);
  valid = true;
  return true;
}

// ----------------------------------------- AK8963 over the internal master --

bool MPU9250::magWaitSlv4() {
  // SLV4 single transfers complete at the sample-rate cadence; 100 ms covers
  // even a slow 8 Hz config. I2C_MST_STATUS latches DONE until it is read.
  const uint32_t start = millis();
  while (millis() - start < 100) {
    uint8_t status = 0;
    if (bus_.readRegister(I2C_MST_STATUS, status) != IMUStatus::Ok) {
      return false;
    }
    if (status & I2C_MST_SLV4_DONE) {
      return (status & I2C_MST_SLV4_NACK) == 0;  // NACK => no device answered
    }
  }
  return false;
}

bool MPU9250::magWriteReg(uint8_t reg, uint8_t value) {
  bus_.writeRegister(I2C_SLV4_ADDR, kMagAddr);  // bit7 clear => write
  bus_.writeRegister(I2C_SLV4_REG, reg);
  bus_.writeRegister(I2C_SLV4_DO, value);
  bus_.writeRegister(I2C_SLV4_CTRL, I2C_SLV_EN);
  return magWaitSlv4();
}

bool MPU9250::magReadReg(uint8_t reg, uint8_t& value) {
  bus_.writeRegister(I2C_SLV4_ADDR, kMagAddr | I2C_READ_FLAG);
  bus_.writeRegister(I2C_SLV4_REG, reg);
  bus_.writeRegister(I2C_SLV4_CTRL, I2C_SLV_EN);
  if (!magWaitSlv4()) {
    return false;
  }
  return bus_.readRegister(I2C_SLV4_DI, value) == IMUStatus::Ok;
}

bool MPU9250::magConfigureSlv0(bool enable) {
  if (!enable) {
    return bus_.writeRegister(I2C_SLV0_CTRL, 0x00) == IMUStatus::Ok;
  }
  // Auto-read 8 bytes (ST1 + HXL..HZH + ST2) from the AK8963 every sample.
  // Reading through ST2 is mandatory - it releases the AK8963's data latch.
  bus_.writeRegister(I2C_SLV0_ADDR, kMagAddr | I2C_READ_FLAG);
  bus_.writeRegister(I2C_SLV0_REG, MAG_ST1);
  return bus_.writeRegister(I2C_SLV0_CTRL, I2C_SLV_EN | 0x08) == IMUStatus::Ok;
}

// ------------------------------------------------------- mag calibration ----

bool MPU9250::calibrateMag(uint16_t durationMs) {
  if (!magEnabled_) {
    return false;
  }
  // Track the min/max field seen on each axis while the user spins the board.
  Vec3 lo{32767, 32767, 32767};
  Vec3 hi{-32768, -32768, -32768};
  const Vec3 savedBias = cal_.magBias;
  const Vec3 savedScale = cal_.magScale;
  cal_.magBias = Vec3{0, 0, 0};
  cal_.magScale = Vec3{1, 1, 1};

  uint32_t start = millis();
  uint16_t taken = 0;
  while (millis() - start < durationMs) {
    if (update()) {
      Vec3 m = magUT();
      lo.x = min(lo.x, m.x); hi.x = max(hi.x, m.x);
      lo.y = min(lo.y, m.y); hi.y = max(hi.y, m.y);
      lo.z = min(lo.z, m.z); hi.z = max(hi.z, m.z);
      ++taken;
    }
    delay(10);
  }
  if (taken < 50) {
    cal_.magBias = savedBias;
    cal_.magScale = savedScale;
    return false;  // not enough motion/data to trust
  }

  // Hard iron = centre of the min/max box. Soft iron = equalise axis spans.
  Vec3 bias{(hi.x + lo.x) * 0.5f, (hi.y + lo.y) * 0.5f, (hi.z + lo.z) * 0.5f};
  Vec3 span{(hi.x - lo.x) * 0.5f, (hi.y - lo.y) * 0.5f, (hi.z - lo.z) * 0.5f};
  float avg = (span.x + span.y + span.z) / 3.0f;
  Vec3 scale{1, 1, 1};
  if (span.x > 1.0f && span.y > 1.0f && span.z > 1.0f) {
    scale = Vec3{avg / span.x, avg / span.y, avg / span.z};
  }
  cal_.magBias = bias;
  cal_.magScale = scale;
  return true;
}

}  // namespace nimu
