#include "BNO055.h"

namespace nimu {
using namespace bno055;

namespace {
inline int16_t le16(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}
}  // namespace

bool BNO055::begin() {
  if (beginI2C(Wire, kAddrLow)) {
    return true;
  }
  return beginI2C(Wire, kAddrHigh);
}

bool BNO055::beginI2C(TwoWire& wire, uint8_t address) {
  bus_.beginI2C(wire, address, 400000);
  bus_.recoverBus();
  if (resetPin_ >= 0) hardwareReset();
  if (!isConnected()) {
    delay(650);
    if (!isConnected()) {
      return false;
    }
  }
  if (!reset()) {
    return false;
  }
  if (bus_.writeRegister(PWR_MODE, POWER_NORMAL) != IMUStatus::Ok) {
    return false;
  }
  delay(10);
  if (!setPage(0)) {
    return false;
  }
  if (bus_.writeRegister(UNIT_SEL, 0x00) != IMUStatus::Ok) {
    return false;
  }
  return setMode(MODE_NDOF);
}

bool BNO055::beginSPI(SPIClass& spi, uint8_t csPin) {
  (void)spi;
  (void)csPin;
  return false;  // Common BNO055 breakouts expose I2C/UART, not SPI.
}

uint8_t BNO055::whoAmI() {
  uint8_t id = 0;
  bus_.readRegister(CHIP_ID, id);
  return id;
}

bool BNO055::isConnected() {
  return whoAmI() == kChipId;
}

bool BNO055::reset() {
  if (!setMode(MODE_CONFIG)) {
    return false;
  }
  if (bus_.writeRegister(SYS_TRIGGER, SYS_TRIGGER_RST) != IMUStatus::Ok) {
    return false;
  }
  delay(650);
  return isConnected();
}

bool BNO055::setPage(uint8_t page) {
  return bus_.writeRegister(PAGE_ID, page) == IMUStatus::Ok;
}

bool BNO055::setMode(uint8_t mode) {
  if (!setPage(0)) {
    return false;
  }
  if (bus_.writeRegister(OPR_MODE, mode) != IMUStatus::Ok) {
    return false;
  }
  mode_ = mode;
  delay(mode == MODE_CONFIG ? 20 : 30);
  return true;
}

bool BNO055::setExternalCrystal(bool enable) {
  uint8_t oldMode = mode_;
  if (!setMode(MODE_CONFIG)) {
    return false;
  }
  bool ok = bus_.writeRegister(SYS_TRIGGER, enable ? SYS_TRIGGER_EXTCLK : 0x00) ==
            IMUStatus::Ok;
  if (ok) externalCrystal_ = enable;
  delay(10);
  return setMode(oldMode) && ok;
}

void BNO055::configurePins(int8_t resetPin, int8_t interruptPin) {
  resetPin_ = resetPin;
  interruptPin_ = interruptPin;
  if (resetPin_ >= 0) {
    pinMode(resetPin_, OUTPUT);
    digitalWrite(resetPin_, HIGH);
  }
  if (interruptPin_ >= 0) pinMode(interruptPin_, INPUT);
}

bool BNO055::hardwareReset(uint16_t bootDelayMs) {
  if (resetPin_ < 0) return false;
  digitalWrite(resetPin_, LOW);
  delay(10);
  digitalWrite(resetPin_, HIGH);
  delay(bootDelayMs);
  mode_ = MODE_CONFIG;
  externalCrystal_ = false;
  return true;
}

bool BNO055::interruptAsserted() const {
  return interruptPin_ >= 0 && digitalRead(interruptPin_) == HIGH;
}

uint8_t BNO055::interruptStatus() {
  uint8_t status = 0;
  setPage(0);
  bus_.readRegister(INT_STA, status);
  return status;
}

bool BNO055::clearInterrupt() {
  if (!setPage(0)) return false;
  uint8_t value = SYS_TRIGGER_RST_INT |
                  (externalCrystal_ ? SYS_TRIGGER_EXTCLK : 0);
  return bus_.writeRegister(SYS_TRIGGER, value) == IMUStatus::Ok;
}

bool BNO055::fullyCalibrated() {
  return systemCalibration() == 3 && gyroCalibration() == 3 &&
         accelCalibration() == 3 && magCalibration() == 3;
}

bool BNO055::readCalibrationProfile(CalibrationProfile& profile) {
  uint8_t oldMode = mode_;
  if (!setMode(MODE_CONFIG) || !setPage(0)) return false;
  bool ok = bus_.readRegisters(ACC_OFFSET_X_LSB, profile.data,
                               CALIBRATION_PROFILE_LENGTH) == IMUStatus::Ok;
  return setMode(oldMode) && ok;
}

bool BNO055::writeCalibrationProfile(const CalibrationProfile& profile) {
  uint8_t oldMode = mode_;
  if (!setMode(MODE_CONFIG) || !setPage(0)) return false;
  bool ok = bus_.writeRegisters(ACC_OFFSET_X_LSB, profile.data,
                                CALIBRATION_PROFILE_LENGTH) == IMUStatus::Ok;
  return setMode(oldMode) && ok;
}

bool BNO055::readPageRegister(uint8_t page, uint8_t reg, uint8_t& value) {
  if (!setPage(page)) return false;
  bool ok = bus_.readRegister(reg, value) == IMUStatus::Ok;
  setPage(0);
  return ok;
}

bool BNO055::writePageRegister(uint8_t page, uint8_t reg, uint8_t value,
                               bool configurationMode) {
  uint8_t oldMode = mode_;
  if (configurationMode && !setMode(MODE_CONFIG)) return false;
  if (!setPage(page)) return false;
  bool ok = bus_.writeRegister(reg, value) == IMUStatus::Ok;
  setPage(0);
  return (!configurationMode || setMode(oldMode)) && ok;
}

bool BNO055::readVector(uint8_t reg, int16_t& x, int16_t& y, int16_t& z) {
  uint8_t b[6];
  if (bus_.readRegisters(reg, b, sizeof(b)) != IMUStatus::Ok) {
    return false;
  }
  x = le16(&b[0]);
  y = le16(&b[2]);
  z = le16(&b[4]);
  return true;
}

bool BNO055::update() {
  int16_t ax = 0, ay = 0, az = 0;
  int16_t gx = 0, gy = 0, gz = 0;
  int16_t mx = 0, my = 0, mz = 0;
  if (!readVector(ACCEL_DATA_X_LSB, ax, ay, az) ||
      !readVector(GYRO_DATA_X_LSB, gx, gy, gz) ||
      !readVector(MAG_DATA_X_LSB, mx, my, mz)) {
    return false;
  }

  constexpr float kAccelMs2ToG = 1.0f / (100.0f * kGravityMs2);
  Vec3 a{ax * kAccelMs2ToG, ay * kAccelMs2ToG, az * kAccelMs2ToG};
  data_.accel = correct(a, cal_.accelBias, cal_.accelScale);

  Vec3 g{gx / 16.0f, gy / 16.0f, gz / 16.0f};
  data_.gyro = correct(g, cal_.gyroBias, Vec3{1, 1, 1});

  Vec3 m{mx / 16.0f, my / 16.0f, mz / 16.0f};
  data_.mag = correct(m, cal_.magBias, cal_.magScale);

  uint8_t temp = 0;
  bus_.readRegister(TEMP, temp);
  data_.temperature = static_cast<int8_t>(temp);
  data_.timestamp = micros();
  return true;
}

bool BNO055::setAccelRangeG(uint16_t maxG) {
  uint8_t range = 0x00;
  if (maxG <= 2) range = 0x00;
  else if (maxG <= 4) range = 0x01;
  else if (maxG <= 8) range = 0x02;
  else range = 0x03;
  uint8_t oldMode = mode_;
  if (!setMode(MODE_CONFIG) || !setPage(1)) return false;
  bool ok = bus_.writeRegister(0x08, range) == IMUStatus::Ok;
  setPage(0);
  return setMode(oldMode) && ok;
}

bool BNO055::setGyroRangeDps(uint16_t maxDps) {
  uint8_t range = 0x00;
  if (maxDps <= 125) range = 0x04;
  else if (maxDps <= 250) range = 0x03;
  else if (maxDps <= 500) range = 0x02;
  else if (maxDps <= 1000) range = 0x01;
  else range = 0x00;
  uint8_t oldMode = mode_;
  if (!setMode(MODE_CONFIG) || !setPage(1)) return false;
  bool ok = bus_.writeRegister(0x0A, range) == IMUStatus::Ok;
  setPage(0);
  return setMode(oldMode) && ok;
}

bool BNO055::setLowPassFilterHz(uint16_t hz) {
  (void)hz;
  return true;  // Fusion mode owns most filtering; keep API portable.
}

bool BNO055::setSampleRateHz(uint16_t hz) {
  (void)hz;
  return true;  // BNO055 vector output rates are mode-defined.
}

uint8_t BNO055::calibrationStatus() {
  uint8_t status = 0;
  bus_.readRegister(CALIB_STAT, status);
  return status;
}

uint8_t BNO055::systemCalibration() { return (calibrationStatus() >> 6) & 0x03; }
uint8_t BNO055::gyroCalibration() { return (calibrationStatus() >> 4) & 0x03; }
uint8_t BNO055::accelCalibration() { return (calibrationStatus() >> 2) & 0x03; }
uint8_t BNO055::magCalibration() { return calibrationStatus() & 0x03; }

Vec3 BNO055::eulerDeg() {
  int16_t h = 0, r = 0, p = 0;
  if (!readVector(EULER_H_LSB, h, r, p)) {
    return Vec3{};
  }
  return Vec3{h / 16.0f, r / 16.0f, p / 16.0f};
}

BNO055::Quaternion BNO055::quaternion() {
  uint8_t b[8];
  if (bus_.readRegisters(QUATERNION_W_LSB, b, sizeof(b)) != IMUStatus::Ok) {
    return Quaternion{1, 0, 0, 0};
  }
  constexpr float scale = 1.0f / 16384.0f;
  return Quaternion{le16(&b[0]) * scale, le16(&b[2]) * scale,
                    le16(&b[4]) * scale, le16(&b[6]) * scale};
}

Vec3 BNO055::linearAccelMs2() {
  int16_t x = 0, y = 0, z = 0;
  if (!readVector(LINEAR_ACCEL_DATA_X_LSB, x, y, z)) return Vec3{};
  return Vec3{x / 100.0f, y / 100.0f, z / 100.0f};
}

Vec3 BNO055::gravityMs2() {
  int16_t x = 0, y = 0, z = 0;
  if (!readVector(GRAVITY_DATA_X_LSB, x, y, z)) return Vec3{};
  return Vec3{x / 100.0f, y / 100.0f, z / 100.0f};
}

}  // namespace nimu
