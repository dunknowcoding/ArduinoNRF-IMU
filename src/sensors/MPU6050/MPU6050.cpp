#include "MPU6050.h"

namespace nimu {
using namespace mpu6050;

namespace {
inline int16_t be16(const uint8_t* p) {
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}
}  // namespace

bool MPU6050::begin() {
  if (beginI2C(Wire, kAddrAD0Low)) {
    return true;
  }
  return beginI2C(Wire, kAddrAD0High);
}

bool MPU6050::beginI2C(TwoWire& wire, uint8_t address) {
  bus_.beginI2C(wire, address, clockHz_);
  bus_.recoverBus();
  if (!isConnected()) {
    return false;
  }
  return reset() && configureDefaults();
}

bool MPU6050::beginSPI(SPIClass& spi, uint8_t csPin) {
  (void)spi;
  (void)csPin;
  return false;  // MPU-6050/GY-521 is I2C-only.
}

uint8_t MPU6050::whoAmI() {
  uint8_t id = 0;
  bus_.readRegister(WHO_AM_I, id);
  return id;
}

bool MPU6050::isConnected() {
  return whoAmI() == kWhoAmI;
}

bool MPU6050::reset() {
  if (bus_.writeRegister(PWR_MGMT_1, PWR1_H_RESET) != IMUStatus::Ok) {
    return false;
  }
  delay(100);
  bus_.recoverBus();
  if (bus_.writeRegister(PWR_MGMT_1, PWR1_CLKSEL_PLL_XGYRO) != IMUStatus::Ok) {
    return false;
  }
  delay(10);
  return true;
}

bool MPU6050::configureDefaults() {
  bool ok = true;
  ok &= setAccelRangeG(accelRangeG_);
  ok &= setGyroRangeDps(gyroRangeDps_);
  ok &= setLowPassFilterHz(42);
  ok &= setSampleRateHz(sampleRateHz_);
  return ok;
}

bool MPU6050::readRaw(RawSample& out) {
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
  return true;
}

bool MPU6050::update() {
  RawSample raw;
  if (!readRaw(raw)) {
    return false;
  }

  Vec3 a{raw.ax / accelLsbPerG_, raw.ay / accelLsbPerG_,
         raw.az / accelLsbPerG_};
  data_.accel = correct(a, cal_.accelBias, cal_.accelScale);

  Vec3 g{raw.gx / gyroLsbPerDps_, raw.gy / gyroLsbPerDps_,
         raw.gz / gyroLsbPerDps_};
  data_.gyro = correct(g, cal_.gyroBias, Vec3{1, 1, 1});

  data_.mag = Vec3{0, 0, 0};
  data_.temperature = raw.temp / kTempSensitivity + kTempOffsetC;
  data_.timestamp = micros();
  return true;
}

void MPU6050::setBusClockHz(uint32_t hz) {
  clockHz_ = hz;
  bus_.setClockHz(hz);
}

bool MPU6050::setAccelRangeG(uint16_t maxG) {
  uint8_t fs;
  if (maxG <= 2) {
    fs = ACCEL_FS_2; accelLsbPerG_ = 16384.0f; accelRangeG_ = 2;
  } else if (maxG <= 4) {
    fs = ACCEL_FS_4; accelLsbPerG_ = 8192.0f; accelRangeG_ = 4;
  } else if (maxG <= 8) {
    fs = ACCEL_FS_8; accelLsbPerG_ = 4096.0f; accelRangeG_ = 8;
  } else {
    fs = ACCEL_FS_16; accelLsbPerG_ = 2048.0f; accelRangeG_ = 16;
  }
  return bus_.updateRegister(ACCEL_CONFIG, ACCEL_FS_MASK, fs) == IMUStatus::Ok;
}

bool MPU6050::setGyroRangeDps(uint16_t maxDps) {
  uint8_t fs;
  if (maxDps <= 250) {
    fs = GYRO_FS_250; gyroLsbPerDps_ = 131.0f; gyroRangeDps_ = 250;
  } else if (maxDps <= 500) {
    fs = GYRO_FS_500; gyroLsbPerDps_ = 65.5f; gyroRangeDps_ = 500;
  } else if (maxDps <= 1000) {
    fs = GYRO_FS_1000; gyroLsbPerDps_ = 32.8f; gyroRangeDps_ = 1000;
  } else {
    fs = GYRO_FS_2000; gyroLsbPerDps_ = 16.4f; gyroRangeDps_ = 2000;
  }
  return bus_.updateRegister(GYRO_CONFIG, GYRO_FS_MASK, fs) == IMUStatus::Ok;
}

bool MPU6050::setGyroDlpfConfig(uint8_t cfg) {
  return bus_.updateRegister(CONFIG, 0x07, cfg & 0x07) == IMUStatus::Ok;
}

bool MPU6050::setLowPassFilterHz(uint16_t hz) {
  uint8_t cfg;
  if (hz >= 256)      cfg = 0;  // accel 260 Hz, gyro 256 Hz
  else if (hz >= 188) cfg = 1;  // accel 184 Hz, gyro 188 Hz
  else if (hz >= 98)  cfg = 2;  // accel 94 Hz, gyro 98 Hz
  else if (hz >= 42)  cfg = 3;  // accel 44 Hz, gyro 42 Hz
  else if (hz >= 20)  cfg = 4;  // accel 21 Hz, gyro 20 Hz
  else if (hz >= 10)  cfg = 5;  // 10 Hz
  else                cfg = 6;  // 5 Hz
  return setGyroDlpfConfig(cfg);
}

bool MPU6050::setSampleRateHz(uint16_t hz) {
  if (hz == 0) {
    return false;
  }
  if (hz > 1000) {
    hz = 1000;
  }
  uint16_t div = 1000u / hz;
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

bool MPU6050::setDataReadyInterrupt(bool enable, bool latch) {
  if (bus_.updateRegister(INT_PIN_CFG, INTCFG_LATCH_INT,
                          latch ? INTCFG_LATCH_INT : 0x00) != IMUStatus::Ok) {
    return false;
  }
  return bus_.writeRegister(INT_ENABLE, enable ? INT_RAW_RDY : 0x00) ==
         IMUStatus::Ok;
}

bool MPU6050::configureInterruptPin(bool activeLow, bool openDrain,
                                    bool latched, bool clearOnAnyRead) {
  uint8_t mask = INTCFG_ACTIVE_LOW | INTCFG_OPEN_DRAIN |
                 INTCFG_LATCH_INT | INTCFG_CLEAR_ANY_READ;
  uint8_t value = (activeLow ? INTCFG_ACTIVE_LOW : 0) |
                  (openDrain ? INTCFG_OPEN_DRAIN : 0) |
                  (latched ? INTCFG_LATCH_INT : 0) |
                  (clearOnAnyRead ? INTCFG_CLEAR_ANY_READ : 0);
  return bus_.updateRegister(INT_PIN_CFG, mask, value) == IMUStatus::Ok;
}

bool MPU6050::setExternalSync(uint8_t target) {
  return bus_.updateRegister(CONFIG, EXT_SYNC_MASK, target & EXT_SYNC_MASK) ==
         IMUStatus::Ok;
}

bool MPU6050::dataReady() {
  uint8_t status = 0;
  if (bus_.readRegister(INT_STATUS, status) != IMUStatus::Ok) {
    return false;
  }
  return (status & INT_RAW_RDY) != 0;
}

bool MPU6050::sleep(bool enable) {
  return bus_.updateRegister(PWR_MGMT_1, PWR1_SLEEP,
                             enable ? PWR1_SLEEP : 0x00) == IMUStatus::Ok;
}

bool MPU6050::setAuxI2CBypass(bool enable) {
  if (bus_.updateRegister(USER_CTRL, USERCTRL_I2C_MST_EN, 0x00) !=
      IMUStatus::Ok) {
    return false;
  }
  return bus_.updateRegister(INT_PIN_CFG, INTCFG_BYPASS_EN,
                             enable ? INTCFG_BYPASS_EN : 0x00) ==
         IMUStatus::Ok;
}

}  // namespace nimu
