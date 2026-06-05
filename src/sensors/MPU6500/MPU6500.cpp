#include "MPU6500.h"

namespace nimu {
using namespace mpu6500;

namespace {
/** Combine two bytes, high byte first (MPU word order). */
inline int16_t be16(const uint8_t* p) {
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}
}  // namespace

// ---------------------------------------------------------------- bring-up --

bool MPU6500::begin() {
  // Default to AD0 low (0x68); fall back to 0x69 in case the board ties AD0 high.
  if (beginI2C(Wire, kAddrAD0Low)) {
    return true;
  }
  return beginI2C(Wire, kAddrAD0High);
}

bool MPU6500::beginI2C(TwoWire& wire, uint8_t address) {
  i2cWire_ = &wire;
  bus_.beginI2C(wire, address, clockHz_);
  // Clear a jammed bus left by a previous crashed run or a debugger halt, so a
  // cold start is reliable even if a slave is holding SDA low.
  bus_.recoverBus();
  if (!isConnected()) {
    return false;
  }
  return reset() && configureDefaults() && initExtras();
}

bool MPU6500::beginSPI(SPIClass& spi, uint8_t csPin) {
  // 1 MHz is safe for configuration; the MPU tolerates 20 MHz for data reads.
  bus_.beginSPI(spi, csPin, 1000000, 0x80);
  i2cWire_ = nullptr;
  if (!isConnected()) {
    return false;
  }
  if (!reset()) {
    return false;
  }
  // Pin the part to SPI: disable the I2C slave interface so activity on the
  // (now unused) I2C pads cannot disturb register access. Recommended by the
  // datasheet whenever SPI is used.
  bus_.updateRegister(USER_CTRL, USERCTRL_I2C_IF_DIS, USERCTRL_I2C_IF_DIS);
  return configureDefaults() && initExtras();
}

uint8_t MPU6500::whoAmI() {
  uint8_t id = 0;
  bus_.readRegister(WHO_AM_I, id);
  return id;
}

bool MPU6500::isConnected() {
  uint8_t id = whoAmI();
  return id == kWhoAmIMPU6500 || id == kWhoAmIMPU9250 || id == kWhoAmIMPU9255;
}

bool MPU6500::reset() {
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

bool MPU6500::configureDefaults() {
  bool ok = true;
  ok &= setAccelRangeG(accelRangeG_);
  ok &= setGyroRangeDps(gyroRangeDps_);
  ok &= setLowPassFilterHz(41);          // gentle anti-alias by default
  ok &= setSampleRateHz(sampleRateHz_);  // 100 Hz
  return ok;
}

// -------------------------------------------------------------- data read ---

bool MPU6500::readRaw(RawSample& out) {
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
  return true;
}

bool MPU6500::update() {
  RawSample raw;
  if (!readRaw(raw)) {  // virtual: the 9-axis driver also fills the mag bytes
    return false;
  }

  Vec3 a{raw.ax / accelLsbPerG_, raw.ay / accelLsbPerG_, raw.az / accelLsbPerG_};
  data_.accel = correct(a, cal_.accelBias, cal_.accelScale);

  Vec3 g{raw.gx / gyroLsbPerDps_, raw.gy / gyroLsbPerDps_,
         raw.gz / gyroLsbPerDps_};
  data_.gyro = correct(g, cal_.gyroBias, Vec3{1, 1, 1});

  data_.temperature = raw.temp / kTempSensitivity + kTempOffsetC;

  // FSYNC, if routed: the external pin's level is latched into the LSB of the
  // selected output register each sample. Pull it out before the user sees it.
  if (extSyncTarget_ != EXT_SYNC_DISABLED) {
    int16_t src = 0;
    switch (extSyncTarget_) {
      case EXT_SYNC_TEMP_OUT:   src = raw.temp; break;
      case EXT_SYNC_GYRO_XOUT:  src = raw.gx;   break;
      case EXT_SYNC_GYRO_YOUT:  src = raw.gy;   break;
      case EXT_SYNC_GYRO_ZOUT:  src = raw.gz;   break;
      case EXT_SYNC_ACCEL_XOUT: src = raw.ax;   break;
      case EXT_SYNC_ACCEL_YOUT: src = raw.ay;   break;
      case EXT_SYNC_ACCEL_ZOUT: src = raw.az;   break;
      default: break;
    }
    fsyncBit_ = src & 0x1;
  }

  processExtra(raw);  // 9-axis driver writes data_.mag; no-op on a 6-axis part
  data_.timestamp = micros();
  return true;
}

void MPU6500::setBusClockHz(uint32_t hz) {
  clockHz_ = hz;
  bus_.setClockHz(hz);
}

bool MPU6500::setExternalSync(uint8_t fsyncTarget) {
  extSyncTarget_ = fsyncTarget & EXT_SYNC_MASK;
  fsyncBit_ = (extSyncTarget_ == EXT_SYNC_DISABLED) ? -1 : 0;
  return bus_.updateRegister(CONFIG, EXT_SYNC_MASK, extSyncTarget_) ==
         IMUStatus::Ok;
}

// ----------------------------------------------------------- configuration --

bool MPU6500::setAccelRangeG(uint16_t maxG) {
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

bool MPU6500::setGyroRangeDps(uint16_t maxDps) {
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

bool MPU6500::setGyroDlpfConfig(uint8_t cfg) {
  return bus_.updateRegister(CONFIG, 0x07, cfg & 0x07) == IMUStatus::Ok;
}

bool MPU6500::setAccelDlpfConfig(uint8_t cfg) {
  return bus_.updateRegister(ACCEL_CONFIG2, 0x07, cfg & 0x07) == IMUStatus::Ok;
}

bool MPU6500::setLowPassFilterHz(uint16_t hz) {
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

bool MPU6500::setSampleRateHz(uint16_t hz) {
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

bool MPU6500::setDataReadyInterrupt(bool enable, bool latch) {
  // Never set BYPASS_EN here: the 9-axis driver reaches its magnetometer over
  // the internal master, and enabling bypass would short the aux bus onto the
  // main bus.
  uint8_t pinCfg = latch ? INTCFG_LATCH_INT : 0x00;
  bus_.writeRegister(INT_PIN_CFG, pinCfg);
  return bus_.writeRegister(INT_ENABLE, enable ? INT_RAW_RDY : 0x00) ==
         IMUStatus::Ok;
}

bool MPU6500::dataReady() {
  uint8_t status = 0;
  if (bus_.readRegister(INT_STATUS, status) != IMUStatus::Ok) {
    return false;
  }
  return (status & INT_RAW_RDY) != 0;
}

bool MPU6500::sleep(bool enable) {
  return bus_.updateRegister(PWR_MGMT_1, PWR1_SLEEP,
                             enable ? PWR1_SLEEP : 0x00) == IMUStatus::Ok;
}

}  // namespace nimu
