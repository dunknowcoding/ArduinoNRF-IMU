#include "ICM45686.h"

namespace nimu {
using namespace icm45686;

namespace {
int16_t be16(const uint8_t* p) {
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}
}

bool ICM45686::begin() { return beginI2C(Wire, kAddr); }

bool ICM45686::beginI2C(TwoWire& wire, uint8_t address) {
  bus_.beginI2C(wire, address, 400000);
  bus_.recoverBus();
  if (!isConnected() || !reset()) return false;
  if (bus_.writeRegister(PWR_MGMT0, PWR_ACCEL_GYRO_LN) != IMUStatus::Ok)
    return false;
  return setAccelRangeG(16) && setGyroRangeDps(2000) &&
         setSampleRateHz(100);
}

bool ICM45686::beginSPI(SPIClass& spi, uint8_t csPin) {
  bus_.beginSPI(spi, csPin, 1000000, 0x80);
  if (!isConnected() || !reset()) return false;
  if (bus_.writeRegister(PWR_MGMT0, PWR_ACCEL_GYRO_LN) != IMUStatus::Ok)
    return false;
  return setAccelRangeG(16) && setGyroRangeDps(2000) &&
         setSampleRateHz(100);
}

uint8_t ICM45686::whoAmI() {
  uint8_t id = 0;
  bus_.readRegister(WHO_AM_I, id);
  return id;
}

bool ICM45686::isConnected() { return whoAmI() == kWhoAmI; }

bool ICM45686::reset() {
  if (bus_.writeRegister(REG_MISC2, SOFT_RESET) != IMUStatus::Ok) return false;
  delay(35);
  return isConnected();
}

bool ICM45686::readRaw(RawSample& out) {
  uint8_t raw[14];
  if (bus_.readRegisters(ACCEL_DATA_X1, raw, sizeof(raw)) != IMUStatus::Ok)
    return false;
  out.ax = be16(&raw[0]); out.ay = be16(&raw[2]); out.az = be16(&raw[4]);
  out.gx = be16(&raw[6]); out.gy = be16(&raw[8]); out.gz = be16(&raw[10]);
  out.temp = be16(&raw[12]);
  return true;
}

bool ICM45686::update() {
  RawSample raw;
  if (!readRaw(raw)) return false;
  data_.accel = correct(Vec3{raw.ax / accelLsbPerG_, raw.ay / accelLsbPerG_,
                              raw.az / accelLsbPerG_},
                        cal_.accelBias, cal_.accelScale);
  data_.gyro = correct(Vec3{raw.gx / gyroLsbPerDps_, raw.gy / gyroLsbPerDps_,
                             raw.gz / gyroLsbPerDps_},
                       cal_.gyroBias, Vec3{1, 1, 1});
  if (qmcEnabled_) {
    Vec3 mag;
    if (readQMC6309(mag)) data_.mag = correct(mag, cal_.magBias, cal_.magScale);
  }
  data_.temperature = 25.0f + raw.temp / 128.0f;
  data_.timestamp = micros();
  return true;
}

bool ICM45686::setAccelRangeG(uint16_t maxG) {
  uint16_t range;
  if (maxG <= 2) { accelFsCode_ = 4; range = 2; }
  else if (maxG <= 4) { accelFsCode_ = 3; range = 4; }
  else if (maxG <= 8) { accelFsCode_ = 2; range = 8; }
  else if (maxG <= 16) { accelFsCode_ = 1; range = 16; }
  else { accelFsCode_ = 0; range = 32; }
  accelLsbPerG_ = 32768.0f / range;
  return bus_.writeRegister(ACCEL_CONFIG0, (accelFsCode_ << 4) | odrCode_) ==
         IMUStatus::Ok;
}

bool ICM45686::setGyroRangeDps(uint16_t maxDps) {
  uint16_t range;
  if (maxDps <= 125) { gyroFsCode_ = 5; range = 125; }
  else if (maxDps <= 250) { gyroFsCode_ = 4; range = 250; }
  else if (maxDps <= 500) { gyroFsCode_ = 3; range = 500; }
  else if (maxDps <= 1000) { gyroFsCode_ = 2; range = 1000; }
  else if (maxDps <= 2000) { gyroFsCode_ = 1; range = 2000; }
  else { gyroFsCode_ = 0; range = 4000; }
  gyroLsbPerDps_ = 32768.0f / range;
  return bus_.writeRegister(GYRO_CONFIG0, (gyroFsCode_ << 4) | odrCode_) ==
         IMUStatus::Ok;
}

bool ICM45686::setSampleRateHz(uint16_t hz) {
  if (hz == 0) return false;
  if (hz <= 2) odrCode_ = 15;
  else if (hz <= 4) odrCode_ = 14;
  else if (hz <= 7) odrCode_ = 13;
  else if (hz <= 13) odrCode_ = 12;
  else if (hz <= 25) odrCode_ = 11;
  else if (hz <= 50) odrCode_ = 10;
  else if (hz <= 100) odrCode_ = 9;
  else if (hz <= 200) odrCode_ = 8;
  else if (hz <= 400) odrCode_ = 7;
  else if (hz <= 800) odrCode_ = 6;
  else if (hz <= 1600) odrCode_ = 5;
  else if (hz <= 3200) odrCode_ = 4;
  else odrCode_ = 3;
  bool ok = bus_.writeRegister(ACCEL_CONFIG0,
                               (accelFsCode_ << 4) | odrCode_) == IMUStatus::Ok;
  ok &= bus_.writeRegister(GYRO_CONFIG0,
                           (gyroFsCode_ << 4) | odrCode_) == IMUStatus::Ok;
  return ok;
}

bool ICM45686::setLowPassFilterHz(uint16_t hz) {
  (void)hz;
  return true;
}

bool ICM45686::routeDataReadyInterrupt(uint8_t pin, bool enable,
                                       bool includeAux) {
  uint8_t reg = pin == 1 ? INT1_CONFIG0 : pin == 2 ? INT2_CONFIG0 : 0;
  if (reg == 0) return false;
  uint8_t mask = INT_DRDY | INT_AUX_DRDY;
  uint8_t value = enable ? INT_DRDY | (includeAux ? INT_AUX_DRDY : 0) : 0;
  return bus_.updateRegister(reg, mask, value) == IMUStatus::Ok;
}

bool ICM45686::configureInterruptPin(uint8_t pin, bool activeLow,
                                     bool openDrain, bool latched) {
  uint8_t reg = pin == 1 ? INT1_CONFIG2 : pin == 2 ? INT2_CONFIG2 : 0;
  if (reg == 0) return false;
  uint8_t value = (activeLow ? INT_ACTIVE_LOW : 0) |
                  (openDrain ? INT_OPEN_DRAIN : 0) |
                  (latched ? INT_LATCHED : 0);
  return bus_.writeRegister(reg, value) == IMUStatus::Ok;
}

bool ICM45686::dataReady(uint8_t pin) {
  uint8_t reg = pin == 1 ? INT1_STATUS0 : pin == 2 ? INT2_STATUS0 : 0;
  uint8_t status = 0;
  return reg != 0 && bus_.readRegister(reg, status) == IMUStatus::Ok &&
         (status & INT_DRDY) != 0;
}

bool ICM45686::indirectWrite(uint16_t address, uint8_t value) {
  uint8_t packet[3] = {static_cast<uint8_t>(address >> 8),
                       static_cast<uint8_t>(address), value};
  if (bus_.writeRegisters(IREG_ADDR, packet, sizeof(packet)) != IMUStatus::Ok)
    return false;
  delayMicroseconds(4);
  return true;
}

bool ICM45686::indirectRead(uint16_t address, uint8_t& value) {
  uint8_t packet[2] = {static_cast<uint8_t>(address >> 8),
                       static_cast<uint8_t>(address)};
  if (bus_.writeRegisters(IREG_ADDR, packet, sizeof(packet)) != IMUStatus::Ok)
    return false;
  delayMicroseconds(4);
  return bus_.readRegister(IREG_DATA, value) == IMUStatus::Ok;
}

bool ICM45686::enableAuxI2CMaster(bool enable) {
  return setAuxiliaryMode(enable ? AuxiliaryMode::I2CMaster
                                 : AuxiliaryMode::Disabled);
}

bool ICM45686::setAuxiliaryMode(AuxiliaryMode mode) {
  uint8_t value = AUX_OVERRIDE_MODE | AUX_OVERRIDE_ENABLE;
  switch (mode) {
    case AuxiliaryMode::Disabled:
      break;
    case AuxiliaryMode::OisSpi:
      value |= AUX_ENABLE;
      break;
    case AuxiliaryMode::I2CMaster:
      value |= AUX_MODE_I2C_MASTER | AUX_ENABLE;
      break;
    case AuxiliaryMode::I2CBypass:
      value |= AUX_MODE_I2C_BYPASS | AUX_ENABLE;
      break;
  }
  return bus_.writeRegister(IOC_PAD_SCENARIO_AUX_OVRD, value) == IMUStatus::Ok;
}

bool ICM45686::enableAuxOisSpi(bool enable) {
  return setAuxiliaryMode(enable ? AuxiliaryMode::OisSpi
                                 : AuxiliaryMode::Disabled);
}

bool ICM45686::enableAuxI2CBypass(bool enable) {
  return setAuxiliaryMode(enable ? AuxiliaryMode::I2CBypass
                                 : AuxiliaryMode::Disabled);
}

ICM45686::AuxiliaryMode ICM45686::auxiliaryMode() {
  uint8_t value = 0;
  if (bus_.readRegister(IOC_PAD_SCENARIO, value) != IMUStatus::Ok ||
      (value & AUX_ENABLE) == 0) return AuxiliaryMode::Disabled;
  switch ((value >> 1) & 0x03) {
    case 0: return AuxiliaryMode::OisSpi;
    case 1: return AuxiliaryMode::I2CMaster;
    case 2: return AuxiliaryMode::I2CBypass;
    default: return AuxiliaryMode::Disabled;
  }
}

bool ICM45686::setAuxAddress(uint8_t address8) {
  auxAddress8_ = address8;
  return indirectWrite(IPREG_TOP1 + I2CM_DEV_PROFILE1, address8);
}

bool ICM45686::auxReadRegister(uint8_t reg, uint8_t& value,
                               uint16_t timeoutUs) {
  if (!indirectWrite(IPREG_TOP1 + I2CM_DEV_PROFILE0, reg) ||
      !indirectWrite(IPREG_TOP1 + I2CM_COMMAND0, 0x91) ||
      !indirectWrite(IPREG_TOP1 + I2CM_CONTROL, 0x01)) return false;
  uint32_t started = micros();
  uint8_t status = 0;
  do {
    if (!indirectRead(IPREG_TOP1 + I2CM_STATUS, status)) return false;
    if ((status & I2CM_STATUS_BUSY) == 0) break;
  } while (static_cast<uint32_t>(micros() - started) < timeoutUs);
  return status == I2CM_STATUS_DONE &&
         indirectRead(IPREG_TOP1 + I2CM_READ_DATA0, value);
}

bool ICM45686::auxWriteRegister(uint8_t reg, uint8_t value,
                                uint16_t timeoutUs) {
  if (!indirectWrite(IPREG_TOP1 + I2CM_DEV_PROFILE0, reg) ||
      !indirectWrite(IPREG_TOP1 + I2CM_WRITE_DATA0, value) ||
      !indirectWrite(IPREG_TOP1 + I2CM_COMMAND0, 0x91) ||
      !indirectWrite(IPREG_TOP1 + I2CM_CONTROL, 0x01)) return false;
  uint32_t started = micros();
  uint8_t status = 0;
  do {
    if (!indirectRead(IPREG_TOP1 + I2CM_STATUS, status)) return false;
    if ((status & I2CM_STATUS_BUSY) == 0) break;
  } while (static_cast<uint32_t>(micros() - started) < timeoutUs);
  return status == I2CM_STATUS_DONE;
}

bool ICM45686::configureQMC6309() {
  if (!enableAuxI2CMaster() || !setAuxAddress(0x7C)) return false;
  uint8_t id = 0;
  if (!auxReadRegister(0x00, id) || id != 0x90) return false;
  if (!auxWriteRegister(0x0B, 0x80) || !auxWriteRegister(0x0B, 0x00))
    return false;
  delay(10);
  qmcEnabled_ = auxWriteRegister(0x0B, 0x48) &&
                auxWriteRegister(0x0A, 0x21);
  hasMag_ = qmcEnabled_;
  return qmcEnabled_;
}

bool ICM45686::readQMC6309(Vec3& magUT) {
  uint8_t raw[6];
  for (uint8_t i = 0; i < sizeof(raw); ++i) {
    if (!auxReadRegister(static_cast<uint8_t>(0x01 + i), raw[i])) return false;
  }
  constexpr float scale = 100.0f / 4000.0f;
  auto le16 = [](const uint8_t* p) {
    return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                                (static_cast<uint16_t>(p[1]) << 8));
  };
  magUT = Vec3{le16(&raw[0]) * scale, le16(&raw[2]) * scale,
               le16(&raw[4]) * scale};
  return true;
}

}  // namespace nimu
