#include "ICM20948.h"

namespace nimu {
using namespace icm20948;

namespace {
inline int16_t be16(const uint8_t* p) {
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) |
                              static_cast<uint16_t>(p[1]));
}
}  // namespace

bool ICM20948::begin() {
  if (beginI2C(Wire, kAddrAD0High)) {
    return true;
  }
  return beginI2C(Wire, kAddrAD0Low);
}

bool ICM20948::beginI2C(TwoWire& wire, uint8_t address) {
  wire_ = &wire;
  bus_.beginI2C(wire, address, clockHz_);
  bus_.recoverBus();
  currentBank_ = 0xFF;
  if (!isConnected()) {
    return false;
  }
  if (!reset() || !configureDefaults()) return false;
  enableMagnetometer(true);
  return true;
}

bool ICM20948::beginSPI(SPIClass& spi, uint8_t csPin) {
  wire_ = nullptr;
  magEnabled_ = false;
  hasMag_ = false;
  bus_.beginSPI(spi, csPin, 1000000, 0x80);
  currentBank_ = 0xFF;
  if (!isConnected()) {
    return false;
  }
  return reset() && configureDefaults();
}

bool ICM20948::selectBank(uint8_t bank) {
  if (currentBank_ == bank) {
    return true;
  }
  if (bus_.writeRegister(REG_BANK_SEL, (bank & 0x03) << 4) != IMUStatus::Ok) {
    return false;
  }
  currentBank_ = bank;
  return true;
}

uint8_t ICM20948::whoAmI() {
  uint8_t id = 0;
  selectBank(0);
  bus_.readRegister(WHO_AM_I, id);
  return id;
}

bool ICM20948::isConnected() {
  return whoAmI() == kWhoAmI;
}

bool ICM20948::reset() {
  if (!selectBank(0)) {
    return false;
  }
  if (bus_.writeRegister(PWR_MGMT_1, PWR_DEVICE_RESET) != IMUStatus::Ok) {
    return false;
  }
  delay(100);
  currentBank_ = 0xFF;
  if (!selectBank(0)) {
    return false;
  }
  if (bus_.writeRegister(PWR_MGMT_1, PWR_CLK_AUTO) != IMUStatus::Ok) {
    return false;
  }
  if (bus_.writeRegister(PWR_MGMT_2, 0x00) != IMUStatus::Ok) {
    return false;
  }
  delay(10);
  return true;
}

bool ICM20948::configureDefaults() {
  bool ok = true;
  ok &= setAccelRangeG(accelRangeG_);
  ok &= setGyroRangeDps(gyroRangeDps_);
  ok &= setLowPassFilterHz(50);
  ok &= setSampleRateHz(sampleRateHz_);
  ok &= selectBank(0);
  return ok;
}

bool ICM20948::readRaw(RawSample& out) {
  if (!selectBank(0)) {
    return false;
  }
  uint8_t a[6];
  uint8_t g[6];
  uint8_t t[2];
  if (bus_.readRegisters(ACCEL_XOUT_H, a, sizeof(a)) != IMUStatus::Ok ||
      bus_.readRegisters(GYRO_XOUT_H, g, sizeof(g)) != IMUStatus::Ok ||
      bus_.readRegisters(TEMP_OUT_H, t, sizeof(t)) != IMUStatus::Ok) {
    return false;
  }
  out.ax = be16(&a[0]);
  out.ay = be16(&a[2]);
  out.az = be16(&a[4]);
  out.gx = be16(&g[0]);
  out.gy = be16(&g[2]);
  out.gz = be16(&g[4]);
  out.temp = be16(t);
  return true;
}

bool ICM20948::update() {
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

  if (magEnabled_) {
    uint8_t st1 = 0;
    uint8_t rawMag[8];
    if (magBus_.readRegister(0x10, st1) == IMUStatus::Ok &&
        (st1 & 0x01) != 0 &&
        magBus_.readRegisters(0x11, rawMag, sizeof(rawMag)) == IMUStatus::Ok &&
        (rawMag[7] & 0x08) == 0) {
      auto le16 = [](const uint8_t* p) {
        return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                                    (static_cast<uint16_t>(p[1]) << 8));
      };
      constexpr float kMicroteslaPerLsb = 0.15f;
      Vec3 m{le16(&rawMag[0]) * kMicroteslaPerLsb,
             le16(&rawMag[2]) * kMicroteslaPerLsb,
             le16(&rawMag[4]) * kMicroteslaPerLsb};
      data_.mag = correct(m, cal_.magBias, cal_.magScale);
    }
  } else {
    data_.mag = Vec3{0, 0, 0};
  }
  data_.temperature = 21.0f + (raw.temp / 333.87f);
  data_.timestamp = micros();
  return true;
}

bool ICM20948::setAccelRangeG(uint16_t maxG) {
  if (maxG <= 2) {
    accelFs_ = 0x00; accelLsbPerG_ = 16384.0f; accelRangeG_ = 2;
  } else if (maxG <= 4) {
    accelFs_ = 0x01; accelLsbPerG_ = 8192.0f; accelRangeG_ = 4;
  } else if (maxG <= 8) {
    accelFs_ = 0x02; accelLsbPerG_ = 4096.0f; accelRangeG_ = 8;
  } else {
    accelFs_ = 0x03; accelLsbPerG_ = 2048.0f; accelRangeG_ = 16;
  }
  if (!selectBank(2)) {
    return false;
  }
  return bus_.updateRegister(ACCEL_CONFIG, FS_MASK, accelFs_ << 1) ==
         IMUStatus::Ok;
}

bool ICM20948::setGyroRangeDps(uint16_t maxDps) {
  if (maxDps <= 250) {
    gyroFs_ = 0x00; gyroLsbPerDps_ = 131.0f; gyroRangeDps_ = 250;
  } else if (maxDps <= 500) {
    gyroFs_ = 0x01; gyroLsbPerDps_ = 65.5f; gyroRangeDps_ = 500;
  } else if (maxDps <= 1000) {
    gyroFs_ = 0x02; gyroLsbPerDps_ = 32.8f; gyroRangeDps_ = 1000;
  } else {
    gyroFs_ = 0x03; gyroLsbPerDps_ = 16.4f; gyroRangeDps_ = 2000;
  }
  if (!selectBank(2)) {
    return false;
  }
  return bus_.updateRegister(GYRO_CONFIG_1, FS_MASK, gyroFs_ << 1) ==
         IMUStatus::Ok;
}

uint8_t ICM20948::dlpfCodeForHz(uint16_t hz) const {
  if (hz <= 6) return 0x06;
  if (hz <= 12) return 0x05;
  if (hz <= 24) return 0x04;
  if (hz <= 51) return 0x03;
  if (hz <= 120) return 0x02;
  if (hz <= 196) return 0x01;
  return 0x00;
}

bool ICM20948::setLowPassFilterHz(uint16_t hz) {
  dlpfCode_ = dlpfCodeForHz(hz);
  uint8_t v = (dlpfCode_ << 3);
  if (!selectBank(2)) {
    return false;
  }
  bool ok = true;
  ok &= bus_.updateRegister(ACCEL_CONFIG, DLPF_MASK, v) == IMUStatus::Ok;
  ok &= bus_.updateRegister(GYRO_CONFIG_1, DLPF_MASK, v) == IMUStatus::Ok;
  return ok;
}

bool ICM20948::setSampleRateHz(uint16_t hz) {
  if (hz == 0) {
    return false;
  }
  uint16_t accelBase = 1125;
  uint16_t gyroBase = 1100;
  uint16_t accelDiv = (hz >= accelBase) ? 0 : ((accelBase / hz) - 1);
  uint16_t gyroDiv = (hz >= gyroBase) ? 0 : ((gyroBase / hz) - 1);
  if (accelDiv > 4095) accelDiv = 4095;
  if (gyroDiv > 255) gyroDiv = 255;
  sampleRateHz_ = accelBase / (accelDiv + 1);
  if (!selectBank(2)) {
    return false;
  }
  bool ok = true;
  ok &= bus_.writeRegister(ACCEL_SMPLRT_DIV_1,
                           static_cast<uint8_t>((accelDiv >> 8) & 0x0F)) ==
        IMUStatus::Ok;
  ok &= bus_.writeRegister(ACCEL_SMPLRT_DIV_2,
                           static_cast<uint8_t>(accelDiv & 0xFF)) ==
        IMUStatus::Ok;
  ok &= bus_.writeRegister(GYRO_SMPLRT_DIV, static_cast<uint8_t>(gyroDiv)) ==
        IMUStatus::Ok;
  return ok;
}

bool ICM20948::dataReady() {
  if (!selectBank(0)) {
    return false;
  }
  uint8_t status = 0;
  if (bus_.readRegister(INT_STATUS_1, status) != IMUStatus::Ok) {
    return false;
  }
  return (status & INT_RAW_DATA_0_RDY) != 0;
}

bool ICM20948::configureInterruptPin(bool activeLow, bool openDrain,
                                     bool latched, bool clearOnAnyRead) {
  if (!selectBank(0)) return false;
  uint8_t value = (activeLow ? INT_ACTIVE_LOW : 0) |
                  (openDrain ? INT_OPEN_DRAIN : 0) |
                  (latched ? INT_LATCH : 0) |
                  (clearOnAnyRead ? INT_CLEAR_ANY_READ : 0);
  return bus_.updateRegister(INT_PIN_CFG,
                             INT_ACTIVE_LOW | INT_OPEN_DRAIN | INT_LATCH |
                                 INT_CLEAR_ANY_READ,
                             value) == IMUStatus::Ok;
}

bool ICM20948::setDataReadyInterrupt(bool enable) {
  if (!selectBank(0)) return false;
  return bus_.updateRegister(INT_ENABLE_1, RAW_DATA_0_RDY_ENABLE,
                             enable ? RAW_DATA_0_RDY_ENABLE : 0) ==
         IMUStatus::Ok;
}

bool ICM20948::setFsyncInterrupt(bool enable, bool activeLow) {
  if (!selectBank(0)) return false;
  uint8_t pinValue = (enable ? FSYNC_INTERRUPT : 0) |
                     (activeLow ? FSYNC_ACTIVE_LOW : 0);
  bool ok = bus_.updateRegister(INT_PIN_CFG,
                                FSYNC_INTERRUPT | FSYNC_ACTIVE_LOW,
                                pinValue) == IMUStatus::Ok;
  ok &= bus_.updateRegister(INT_ENABLE, REG_WOF_ENABLE,
                            enable ? REG_WOF_ENABLE : 0) == IMUStatus::Ok;
  return ok;
}

bool ICM20948::setAuxI2CBypass(bool enable) {
  if (!selectBank(0)) return false;
  return bus_.updateRegister(INT_PIN_CFG, BYPASS_ENABLE,
                             enable ? BYPASS_ENABLE : 0) == IMUStatus::Ok;
}

bool ICM20948::enableAuxI2CMaster(bool enable) {
  if (!selectBank(0)) return false;
  if (enable && !setAuxI2CBypass(false)) return false;
  if (!selectBank(0)) return false;
  if (bus_.updateRegister(USER_CTRL, I2C_MST_ENABLE,
                          enable ? I2C_MST_ENABLE : 0) != IMUStatus::Ok)
    return false;
  if (!enable) return true;
  if (!selectBank(3)) return false;
  // 0x07 selects an approximately 345 kHz auxiliary I2C clock.
  bool ok = bus_.writeRegister(I2C_MST_CTRL, 0x07) == IMUStatus::Ok;
  return selectBank(0) && ok;
}

bool ICM20948::waitAuxTransaction(uint16_t timeoutMs) {
  uint32_t start = millis();
  do {
    if (!selectBank(0)) return false;
    uint8_t status = 0;
    if (bus_.readRegister(I2C_MST_STATUS, status) != IMUStatus::Ok)
      return false;
    if ((status & I2C_SLV4_NACK) != 0) return false;
    if ((status & I2C_SLV4_DONE) != 0) return true;
  } while (static_cast<uint32_t>(millis() - start) < timeoutMs);
  return false;
}

bool ICM20948::auxReadRegister(uint8_t address, uint8_t reg, uint8_t& value,
                               uint16_t timeoutMs) {
  if (!enableAuxI2CMaster() || !selectBank(3)) return false;
  bool ok = bus_.writeRegister(I2C_SLV4_ADDR, address | 0x80) == IMUStatus::Ok;
  ok &= bus_.writeRegister(I2C_SLV4_REG, reg) == IMUStatus::Ok;
  ok &= bus_.writeRegister(I2C_SLV4_CTRL, I2C_SLV4_ENABLE) == IMUStatus::Ok;
  if (!ok || !waitAuxTransaction(timeoutMs) || !selectBank(3)) return false;
  ok = bus_.readRegister(I2C_SLV4_DI, value) == IMUStatus::Ok;
  return selectBank(0) && ok;
}

bool ICM20948::auxWriteRegister(uint8_t address, uint8_t reg, uint8_t value,
                                uint16_t timeoutMs) {
  if (!enableAuxI2CMaster() || !selectBank(3)) return false;
  bool ok = bus_.writeRegister(I2C_SLV4_ADDR, address & 0x7F) == IMUStatus::Ok;
  ok &= bus_.writeRegister(I2C_SLV4_REG, reg) == IMUStatus::Ok;
  ok &= bus_.writeRegister(I2C_SLV4_DO, value) == IMUStatus::Ok;
  ok &= bus_.writeRegister(I2C_SLV4_CTRL, I2C_SLV4_ENABLE) == IMUStatus::Ok;
  ok &= waitAuxTransaction(timeoutMs);
  return selectBank(0) && ok;
}

bool ICM20948::auxPing(uint8_t address) {
  uint8_t scratch = 0;
  return auxReadRegister(address, 0x00, scratch);
}

uint8_t ICM20948::magWhoAmI() {
  uint8_t id = 0;
  magBus_.readRegister(0x01, id);  // AK09916 WIA2
  return id;
}

bool ICM20948::enableMagnetometer(bool enable) {
  if (!enable) {
    if (magEnabled_) magBus_.writeRegister(0x31, 0x00);
    magEnabled_ = false;
    hasMag_ = false;
    return true;
  }
  if (wire_ == nullptr || !setAuxI2CBypass(true)) return false;
  magBus_.beginI2C(*wire_, 0x0C, clockHz_);
  if (magWhoAmI() != 0x09) return false;
  if (magBus_.writeRegister(0x32, 0x01) != IMUStatus::Ok) return false;
  delay(1);
  if (magBus_.writeRegister(0x31, 0x08) != IMUStatus::Ok) return false;
  magEnabled_ = true;
  hasMag_ = true;
  return true;
}

void ICM20948::setBusClockHz(uint32_t hz) {
  clockHz_ = hz;
  bus_.setClockHz(hz);
}

}  // namespace nimu
