#include "MPU9250.h"

namespace nimu {
using namespace mpu9250;

namespace {
/** Combine two bytes, high byte first (MPU-6500 word order). */
inline int16_t be16(const uint8_t* p) {
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}
/** Combine two bytes, low byte first (AK8963 word order). */
inline int16_t le16(const uint8_t* p) {
  return static_cast<int16_t>((static_cast<uint16_t>(p[1]) << 8) | p[0]);
}
}  // namespace

// ---------------------------------------------------------------- bring-up --

bool MPU9250::begin() {
  // The GY-9250 defaults to AD0 low (0x68). Fall back to 0x69 in case the
  // board ties AD0 high.
  if (beginI2C(Wire, kAddrAD0Low)) {
    return true;
  }
  return beginI2C(Wire, kAddrAD0High);
}

bool MPU9250::beginI2C(TwoWire& wire, uint8_t address) {
  i2cWire_ = &wire;
  bus_.beginI2C(wire, address, clockHz_);
  // Clear a jammed bus left by a previous crashed run or a debugger halt, so a
  // cold start is reliable even if a slave is holding SDA low.
  bus_.recoverBus();
  if (!isConnected()) {
    return false;
  }
  return reset() && configureDefaults() && initMagnetometer();
}

bool MPU9250::beginSPI(SPIClass& spi, uint8_t csPin) {
  // 1 MHz is safe for configuration; the MPU tolerates 20 MHz for data reads.
  bus_.beginSPI(spi, csPin, 1000000, 0x80);
  i2cWire_ = nullptr;  // magnetometer bypass is unavailable over SPI
  if (!isConnected()) {
    return false;
  }
  // configureDefaults() but skip the magnetometer (no bypass path on SPI).
  if (!reset() || !configureDefaults()) {
    return false;
  }
  magPresent_ = false;
  magEnabled_ = false;
  hasMag_ = false;
  return true;
}

uint8_t MPU9250::whoAmI() {
  uint8_t id = 0;
  bus_.readRegister(WHO_AM_I, id);
  return id;
}

bool MPU9250::isConnected() {
  uint8_t id = whoAmI();
  return id == kWhoAmIMPU9250 || id == kWhoAmIMPU9255 || id == kWhoAmIMPU6500;
}

bool MPU9250::reset() {
  if (bus_.writeRegister(PWR_MGMT_1, PWR1_H_RESET) != IMUStatus::Ok) {
    return false;
  }
  delay(100);  // reset takes effect; registers return to power-on defaults
  // A device reset can momentarily disturb the bus; clear any jam before the
  // first post-reset access.
  if (bus_.isI2C()) {
    bus_.recoverBus();
  }
  // Wake from sleep and select the best available clock (PLL when ready).
  if (bus_.writeRegister(PWR_MGMT_1, PWR1_CLKSEL_AUTO) != IMUStatus::Ok) {
    return false;
  }
  delay(10);
  return true;
}

bool MPU9250::configureDefaults() {
  bool ok = true;
  ok &= setAccelRangeG(accelRangeG_);
  ok &= setGyroRangeDps(gyroRangeDps_);
  ok &= setLowPassFilterHz(41);          // gentle anti-alias by default
  ok &= setSampleRateHz(sampleRateHz_);  // 100 Hz
  return ok;
}

// ------------------------------------------------------------- magnetometer --

bool MPU9250::initMagnetometer() {
  magPresent_ = false;
  magEnabled_ = false;
  hasMag_ = false;
  if (i2cWire_ == nullptr) {
    return true;  // SPI mode: no magnetometer, but begin() still succeeds
  }

  // Expose the AK8963 on the main I2C bus: disable the MPU's internal I2C
  // master and turn on bypass so 0x0C answers directly.
  bus_.updateRegister(USER_CTRL, 0x20, 0x00);  // I2C_MST_EN = 0
  bus_.writeRegister(INT_PIN_CFG, INTCFG_BYPASS_EN);
  delay(10);

  magBus_.beginI2C(*i2cWire_, kMagAddr, clockHz_);
  uint8_t wia = 0;
  if (magBus_.readRegister(MAG_WIA, wia) != IMUStatus::Ok ||
      wia != kMagWhoAmI) {
    return true;  // genuine MPU-6500 clone with no magnetometer: not an error
  }

  // Soft-reset the AK8963, then read its fuse-ROM sensitivity adjustment.
  magBus_.writeRegister(MAG_CNTL2, 0x01);
  delay(10);
  magBus_.writeRegister(MAG_CNTL1, MAG_MODE_POWERDOWN);
  delay(10);
  magBus_.writeRegister(MAG_CNTL1, MAG_MODE_FUSE_ROM);
  delay(10);
  uint8_t asa[3] = {128, 128, 128};
  magBus_.readRegisters(MAG_ASAX, asa, 3);
  // Per datasheet: adjusted = raw * ((ASA - 128) / 256 + 1).
  magAsa_ = Vec3{(asa[0] - 128) / 256.0f + 1.0f, (asa[1] - 128) / 256.0f + 1.0f,
                 (asa[2] - 128) / 256.0f + 1.0f};

  magPresent_ = true;
  return enableMagnetometer(true);
}

bool MPU9250::enableMagnetometer(bool enable) {
  if (!magPresent_) {
    return false;
  }
  magBus_.writeRegister(MAG_CNTL1, MAG_MODE_POWERDOWN);
  delay(10);
  if (enable) {
    // Continuous 100 Hz, 16-bit resolution.
    magBus_.writeRegister(MAG_CNTL1, MAG_MODE_CONT_100HZ | MAG_BIT_16);
    delay(10);
  }
  magEnabled_ = enable;
  hasMag_ = enable;
  return true;
}

uint8_t MPU9250::magWhoAmI() {
  if (i2cWire_ == nullptr) {
    return 0;
  }
  uint8_t wia = 0;
  magBus_.readRegister(MAG_WIA, wia);
  return wia;
}

bool MPU9250::readMagRaw(int16_t& mx, int16_t& my, int16_t& mz, bool& valid) {
  valid = false;
  mx = my = mz = 0;
  if (!magEnabled_) {
    return false;
  }
  uint8_t st1 = 0;
  if (magBus_.readRegister(MAG_ST1, st1) != IMUStatus::Ok) {
    return false;
  }
  if (!(st1 & MAG_ST1_DRDY)) {
    return true;  // no new sample yet - not an error, just nothing fresh
  }
  // Read 6 data bytes plus ST2; ST2 MUST be read to release the data latch.
  uint8_t buf[7];
  if (magBus_.readRegisters(MAG_HXL, buf, 7) != IMUStatus::Ok) {
    return false;
  }
  if (buf[6] & MAG_ST2_HOFL) {
    return true;  // magnetic overflow this sample; values discarded
  }
  mx = le16(&buf[0]);
  my = le16(&buf[2]);
  mz = le16(&buf[4]);
  valid = true;
  return true;
}

// -------------------------------------------------------------- data read ---

bool MPU9250::readRaw(RawSample& out) {
  uint8_t buf[14];
  if (bus_.readRegisters(ACCEL_XOUT_H, buf, sizeof(buf)) != IMUStatus::Ok) {
    return false;
  }
  out.ax = be16(&buf[0]);
  out.ay = be16(&buf[2]);
  out.az = be16(&buf[4]);
  out.temp = be16(&buf[6]);
  out.gx = be16(&buf[8]);
  out.gy = be16(&buf[10]);
  out.gz = be16(&buf[12]);
  out.mx = out.my = out.mz = 0;
  out.magValid = false;
  if (magEnabled_) {
    readMagRaw(out.mx, out.my, out.mz, out.magValid);
  }
  return true;
}

bool MPU9250::update() {
  RawSample raw;
  if (!readRaw(raw)) {
    return false;
  }

  Vec3 a{raw.ax / accelLsbPerG_, raw.ay / accelLsbPerG_, raw.az / accelLsbPerG_};
  data_.accel = correct(a, cal_.accelBias, cal_.accelScale);

  Vec3 g{raw.gx / gyroLsbPerDps_, raw.gy / gyroLsbPerDps_,
         raw.gz / gyroLsbPerDps_};
  data_.gyro = correct(g, cal_.gyroBias, Vec3{1, 1, 1});

  data_.temperature = raw.temp / kTempSensitivity + kTempOffsetC;

  if (raw.magValid) {
    // The AK8963 die is rotated relative to the gyro/accel die. Remap its axes
    // into the MPU body frame (X<->Y swapped, Z negated) so all three sensors
    // share one coordinate system, then apply hard/soft-iron calibration.
    float axu = raw.mx * magAsa_.x * kMagScaleUT;
    float ayu = raw.my * magAsa_.y * kMagScaleUT;
    float azu = raw.mz * magAsa_.z * kMagScaleUT;
    Vec3 m{ayu, axu, -azu};
    data_.mag = correct(m, cal_.magBias, cal_.magScale);
  }

  data_.timestamp = micros();
  return true;
}

// ----------------------------------------------------------- configuration --

bool MPU9250::setAccelRangeG(uint16_t maxG) {
  uint8_t fs;
  if (maxG <= 2) {
    fs = ACCEL_FS_2;  accelLsbPerG_ = 16384.0f; accelRangeG_ = 2;
  } else if (maxG <= 4) {
    fs = ACCEL_FS_4;  accelLsbPerG_ = 8192.0f;  accelRangeG_ = 4;
  } else if (maxG <= 8) {
    fs = ACCEL_FS_8;  accelLsbPerG_ = 4096.0f;  accelRangeG_ = 8;
  } else {
    fs = ACCEL_FS_16; accelLsbPerG_ = 2048.0f;  accelRangeG_ = 16;
  }
  return bus_.updateRegister(ACCEL_CONFIG, ACCEL_FS_MASK, fs) == IMUStatus::Ok;
}

bool MPU9250::setGyroRangeDps(uint16_t maxDps) {
  uint8_t fs;
  if (maxDps <= 250) {
    fs = GYRO_FS_250;  gyroLsbPerDps_ = 131.0f;  gyroRangeDps_ = 250;
  } else if (maxDps <= 500) {
    fs = GYRO_FS_500;  gyroLsbPerDps_ = 65.5f;   gyroRangeDps_ = 500;
  } else if (maxDps <= 1000) {
    fs = GYRO_FS_1000; gyroLsbPerDps_ = 32.8f;   gyroRangeDps_ = 1000;
  } else {
    fs = GYRO_FS_2000; gyroLsbPerDps_ = 16.4f;   gyroRangeDps_ = 2000;
  }
  // Mask also clears Fchoice_b (bits[1:0]) so the digital low-pass stays active.
  return bus_.updateRegister(GYRO_CONFIG, GYRO_FS_MASK | 0x03, fs) ==
         IMUStatus::Ok;
}

bool MPU9250::setGyroDlpfConfig(uint8_t cfg) {
  return bus_.updateRegister(CONFIG, 0x07, cfg & 0x07) == IMUStatus::Ok;
}

bool MPU9250::setAccelDlpfConfig(uint8_t cfg) {
  return bus_.updateRegister(ACCEL_CONFIG2, 0x07, cfg & 0x07) == IMUStatus::Ok;
}

bool MPU9250::setLowPassFilterHz(uint16_t hz) {
  // Pick the gyro DLPF code whose bandwidth is the smallest one >= request.
  uint8_t gcfg;
  if (hz >= 250)      gcfg = 0;  // 250 Hz
  else if (hz >= 184) gcfg = 1;  // 184 Hz
  else if (hz >= 92)  gcfg = 2;  // 92 Hz
  else if (hz >= 41)  gcfg = 3;  // 41 Hz
  else if (hz >= 20)  gcfg = 4;  // 20 Hz
  else if (hz >= 10)  gcfg = 5;  // 10 Hz
  else                gcfg = 6;  // 5 Hz

  // Accelerometer DLPF codes for the nearest bandwidth.
  uint8_t acfg;
  if (hz >= 218)      acfg = 1;  // 218 Hz
  else if (hz >= 99)  acfg = 2;  // 99 Hz
  else if (hz >= 45)  acfg = 3;  // 45 Hz
  else if (hz >= 21)  acfg = 4;  // 21 Hz
  else if (hz >= 10)  acfg = 5;  // 10 Hz
  else                acfg = 6;  // 5 Hz

  return setGyroDlpfConfig(gcfg) && setAccelDlpfConfig(acfg);
}

bool MPU9250::setSampleRateHz(uint16_t hz) {
  if (hz == 0) {
    return false;
  }
  if (hz > 1000) {
    hz = 1000;  // internal base rate with the DLPF enabled
  }
  uint16_t div = (1000u / hz);
  if (div > 0) {
    div -= 1;
  }
  if (div > 255) {
    div = 255;
  }
  sampleRateHz_ = hz;
  return bus_.writeRegister(SMPLRT_DIV, static_cast<uint8_t>(div)) ==
         IMUStatus::Ok;
}

// ------------------------------------------------------------- interrupts ---

bool MPU9250::setDataReadyInterrupt(bool enable, bool latch) {
  uint8_t pinCfg = magEnabled_ ? INTCFG_BYPASS_EN : 0x00;
  if (latch) {
    pinCfg |= INTCFG_LATCH_INT;
  }
  bus_.writeRegister(INT_PIN_CFG, pinCfg);
  return bus_.writeRegister(INT_ENABLE, enable ? INT_RAW_RDY : 0x00) ==
         IMUStatus::Ok;
}

bool MPU9250::dataReady() {
  uint8_t status = 0;
  if (bus_.readRegister(INT_STATUS, status) != IMUStatus::Ok) {
    return false;
  }
  return (status & INT_RAW_RDY) != 0;
}

bool MPU9250::sleep(bool enable) {
  return bus_.updateRegister(PWR_MGMT_1, PWR1_SLEEP,
                             enable ? PWR1_SLEEP : 0x00) == IMUStatus::Ok;
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
