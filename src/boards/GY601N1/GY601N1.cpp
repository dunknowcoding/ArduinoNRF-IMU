#include "GY601N1.h"

namespace nimu {

namespace {
Vec3 scaled(const Vec3& value, float scale) {
  return Vec3{value.x * scale, value.y * scale, value.z * scale};
}
}  // namespace

bool GY601N1::begin() {
  return beginI2C(Wire, 0x68) || beginI2C(Wire, 0x69);
}

bool GY601N1::beginI2C(TwoWire& wire, uint8_t address) {
  select(Core::None);
  if (icm42688_.beginI2C(wire, address)) {
    select(Core::ICM42688);
    return true;
  }
  if (icm45686_.beginI2C(wire, address)) {
    select(Core::ICM45686);
    return true;
  }
  if (bmi323_.beginI2C(wire, address)) {
    select(Core::BMI323);
    return true;
  }
  return false;
}

bool GY601N1::beginSPI(SPIClass& spi, uint8_t csPin) {
  select(Core::None);
  if (icm42688_.beginSPI(spi, csPin)) {
    select(Core::ICM42688);
    return true;
  }
  if (icm45686_.beginSPI(spi, csPin)) {
    select(Core::ICM45686);
    return true;
  }
  if (bmi323_.beginSPI(spi, csPin)) {
    select(Core::BMI323);
    return true;
  }
  return false;
}

const char* GY601N1::coreName() const {
  switch (core_) {
    case Core::ICM42688: return "ICM-42688-P";
    case Core::ICM45686: return "ICM-45686";
    case Core::BMI323: return "BMI323";
    default: return "none";
  }
}

IMUSensor* GY601N1::active() {
  switch (core_) {
    case Core::ICM42688: return &icm42688_;
    case Core::ICM45686: return &icm45686_;
    case Core::BMI323: return &bmi323_;
    default: return nullptr;
  }
}

const IMUSensor* GY601N1::active() const {
  switch (core_) {
    case Core::ICM42688: return &icm42688_;
    case Core::ICM45686: return &icm45686_;
    case Core::BMI323: return &bmi323_;
    default: return nullptr;
  }
}

void GY601N1::select(Core core) { core_ = core; }

uint8_t GY601N1::whoAmI() {
  IMUSensor* sensor = active();
  return sensor == nullptr ? 0 : sensor->whoAmI();
}

bool GY601N1::isConnected() {
  IMUSensor* sensor = active();
  return sensor != nullptr && sensor->isConnected();
}

bool GY601N1::update() {
  IMUSensor* sensor = active();
  return sensor != nullptr && sensor->update();
}

bool GY601N1::dataReady() {
  switch (core_) {
    case Core::ICM42688: return icm42688_.dataReady();
    case Core::ICM45686: return icm45686_.dataReady();
    case Core::BMI323: return bmi323_.dataReady();
    default: return false;
  }
}

const IMUData& GY601N1::data() const {
  const IMUSensor* sensor = active();
  return sensor == nullptr ? emptyData_ : sensor->data();
}

Vec3 GY601N1::accelMs2() const { return scaled(accelG(), kGravityMs2); }
Vec3 GY601N1::gyroRps() const { return scaled(gyroDps(), kDegToRad); }

bool GY601N1::setAccelRangeG(uint16_t maxG) {
  IMUSensor* sensor = active();
  return sensor != nullptr && sensor->setAccelRangeG(maxG);
}

bool GY601N1::setGyroRangeDps(uint16_t maxDps) {
  IMUSensor* sensor = active();
  return sensor != nullptr && sensor->setGyroRangeDps(maxDps);
}

bool GY601N1::setLowPassFilterHz(uint16_t hz) {
  IMUSensor* sensor = active();
  return sensor != nullptr && sensor->setLowPassFilterHz(hz);
}

bool GY601N1::setSampleRateHz(uint16_t hz) {
  IMUSensor* sensor = active();
  return sensor != nullptr && sensor->setSampleRateHz(hz);
}

bool GY601N1::configureInterruptPin(uint8_t pin, bool activeHigh,
                                    bool openDrain, bool latched) {
  switch (core_) {
    case Core::ICM42688:
      return icm42688_.configureInterruptPin(pin, activeHigh, openDrain,
                                             latched);
    case Core::ICM45686:
      return icm45686_.configureInterruptPin(pin, !activeHigh, openDrain,
                                             latched);
    case Core::BMI323:
      return bmi323_.configureInterruptPin(pin, activeHigh, openDrain, latched);
    default:
      return false;
  }
}

bool GY601N1::routeDataReadyInterrupt(uint8_t pin, bool enable) {
  switch (core_) {
    case Core::ICM42688:
      return icm42688_.routeDataReadyInterrupt(pin, enable);
    case Core::ICM45686:
      return icm45686_.routeDataReadyInterrupt(pin, enable, false);
    case Core::BMI323:
      return bmi323_.routeDataReadyInterrupt(pin, enable, enable);
    default:
      return false;
  }
}

bool GY601N1::calibrateGyro(uint16_t samples) {
  IMUSensor* sensor = active();
  return sensor != nullptr && sensor->calibrateGyro(samples);
}

bool GY601N1::calibrateAccel(uint16_t samples) {
  IMUSensor* sensor = active();
  return sensor != nullptr && sensor->calibrateAccel(samples);
}

IMUCalibration GY601N1::getCalibration() const {
  const IMUSensor* sensor = active();
  return sensor == nullptr ? IMUCalibration{} : sensor->getCalibration();
}

void GY601N1::setCalibration(const IMUCalibration& calibration) {
  IMUSensor* sensor = active();
  if (sensor != nullptr) sensor->setCalibration(calibration);
}

void GY601N1::clearCalibration() {
  IMUSensor* sensor = active();
  if (sensor != nullptr) sensor->clearCalibration();
}

ICM42688P* GY601N1::icm42688() {
  return core_ == Core::ICM42688 ? &icm42688_ : nullptr;
}

ICM45686* GY601N1::icm45686() {
  return core_ == Core::ICM45686 ? &icm45686_ : nullptr;
}

BMI323* GY601N1::bmi323() {
  return core_ == Core::BMI323 ? &bmi323_ : nullptr;
}

size_t GY601N1::readUART(uint8_t* data, size_t length) {
  if (uart_ == nullptr || data == nullptr) return 0;
  size_t count = 0;
  while (count < length && uart_->available() > 0)
    data[count++] = static_cast<uint8_t>(uart_->read());
  return count;
}

}  // namespace nimu
