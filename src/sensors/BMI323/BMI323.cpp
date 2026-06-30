#include "BMI323.h"

namespace nimu {
using namespace bmi323;

bool BMI323::begin() {
  if (beginI2C(Wire, kAddrLow)) return true;
  return beginI2C(Wire, kAddrHigh);
}

bool BMI323::beginI2C(TwoWire& wire, uint8_t address) {
  wire_ = &wire;
  spiMode_ = false;
  address_ = address;
  wire_->begin();
  wire_->setClock(400000);
  if (!isConnected()) return false;
  return reset() && configureDefaults();
}

bool BMI323::beginSPI(SPIClass& spi, uint8_t csPin) {
  wire_ = nullptr;
  spiMode_ = true;
  bus_.beginSPI(spi, csPin, 1000000, 0x80, 1);
  if (!isConnected()) return false;
  return reset() && configureDefaults();
}

bool BMI323::readWords(uint8_t reg, uint16_t* words, size_t count) {
  if (words == nullptr || count == 0 || count > 15) return false;
  if (spiMode_) {
    uint8_t raw[30];
    const size_t bytes = count * 2;
    if (bus_.readRegisters(reg, raw, bytes) != IMUStatus::Ok) return false;
    for (size_t i = 0; i < count; ++i) {
      words[i] = static_cast<uint16_t>(raw[i * 2]) |
                 (static_cast<uint16_t>(raw[i * 2 + 1]) << 8);
    }
    return true;
  }
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(address_);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) return false;
  const uint8_t bytes = static_cast<uint8_t>(count * 2 + 2);
  if (wire_->requestFrom(address_, bytes, true) != bytes) return false;
  (void)wire_->read();
  (void)wire_->read();  // BMI323 I2C reads return two protocol dummy bytes.
  for (size_t i = 0; i < count; ++i) {
    uint8_t lsb = static_cast<uint8_t>(wire_->read());
    uint8_t msb = static_cast<uint8_t>(wire_->read());
    words[i] = static_cast<uint16_t>(lsb) |
               (static_cast<uint16_t>(msb) << 8);
  }
  return true;
}

bool BMI323::writeWord(uint8_t reg, uint16_t value) {
  if (spiMode_) {
    uint8_t raw[2] = {static_cast<uint8_t>(value),
                      static_cast<uint8_t>(value >> 8)};
    return bus_.writeRegisters(reg, raw, sizeof(raw)) == IMUStatus::Ok;
  }
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(address_);
  wire_->write(reg);
  wire_->write(static_cast<uint8_t>(value));
  wire_->write(static_cast<uint8_t>(value >> 8));
  return wire_->endTransmission() == 0;
}

uint8_t BMI323::whoAmI() {
  uint16_t id = 0;
  return readWords(CHIP_ID, &id, 1) ? static_cast<uint8_t>(id) : 0;
}

bool BMI323::isConnected() { return whoAmI() == kChipId; }

bool BMI323::reset() {
  if (!writeWord(CMD, SOFT_RESET)) return false;
  delay(2);
  uint16_t dummy = 0;
  readWords(CHIP_ID, &dummy, 1);
  return isConnected();
}

uint16_t BMI323::configWord(uint8_t odr, uint8_t range) const {
  return static_cast<uint16_t>(odr | (range << 4) | (1u << 8) | (4u << 12));
}

bool BMI323::configureDefaults() {
  bool ok = writeWord(ACC_CONF, configWord(odrCode_, accelRangeCode_));
  ok &= writeWord(GYR_CONF, configWord(odrCode_, gyroRangeCode_));
  delay(50);
  return ok;
}

bool BMI323::readRaw(RawSample& out) {
  uint16_t words[7];
  if (!readWords(ACC_DATA_X, words, 7)) return false;
  out.ax = static_cast<int16_t>(words[0]);
  out.ay = static_cast<int16_t>(words[1]);
  out.az = static_cast<int16_t>(words[2]);
  out.gx = static_cast<int16_t>(words[3]);
  out.gy = static_cast<int16_t>(words[4]);
  out.gz = static_cast<int16_t>(words[5]);
  out.temp = static_cast<int16_t>(words[6]);
  return true;
}

bool BMI323::update() {
  RawSample raw;
  if (!readRaw(raw)) return false;
  data_.accel = correct(Vec3{raw.ax / accelLsbPerG_, raw.ay / accelLsbPerG_,
                             raw.az / accelLsbPerG_},
                        cal_.accelBias, cal_.accelScale);
  data_.gyro = correct(Vec3{raw.gx / gyroLsbPerDps_, raw.gy / gyroLsbPerDps_,
                            raw.gz / gyroLsbPerDps_},
                       cal_.gyroBias, Vec3{1, 1, 1});
  data_.mag = Vec3{0, 0, 0};
  data_.temperature = (raw.temp == static_cast<int16_t>(0x8000))
                          ? 0.0f
                          : raw.temp / 256.0f + 23.0f;
  data_.timestamp = micros();
  return true;
}

bool BMI323::setAccelRangeG(uint16_t maxG) {
  if (maxG <= 2) { accelRangeCode_ = 0; accelRangeG_ = 2; accelLsbPerG_ = 16384.0f; }
  else if (maxG <= 4) { accelRangeCode_ = 1; accelRangeG_ = 4; accelLsbPerG_ = 8192.0f; }
  else if (maxG <= 8) { accelRangeCode_ = 2; accelRangeG_ = 8; accelLsbPerG_ = 4096.0f; }
  else { accelRangeCode_ = 3; accelRangeG_ = 16; accelLsbPerG_ = 2048.0f; }
  return writeWord(ACC_CONF, configWord(odrCode_, accelRangeCode_));
}

bool BMI323::setGyroRangeDps(uint16_t maxDps) {
  if (maxDps <= 125) { gyroRangeCode_ = 0; gyroRangeDps_ = 125; gyroLsbPerDps_ = 262.144f; }
  else if (maxDps <= 250) { gyroRangeCode_ = 1; gyroRangeDps_ = 250; gyroLsbPerDps_ = 131.072f; }
  else if (maxDps <= 500) { gyroRangeCode_ = 2; gyroRangeDps_ = 500; gyroLsbPerDps_ = 65.536f; }
  else if (maxDps <= 1000) { gyroRangeCode_ = 3; gyroRangeDps_ = 1000; gyroLsbPerDps_ = 32.768f; }
  else { gyroRangeCode_ = 4; gyroRangeDps_ = 2000; gyroLsbPerDps_ = 16.384f; }
  return writeWord(GYR_CONF, configWord(odrCode_, gyroRangeCode_));
}

uint8_t BMI323::odrForHz(uint16_t hz, uint16_t& actualHz) const {
  if (hz <= 13) { actualHz = 12; return 0x05; }
  if (hz <= 25) { actualHz = 25; return 0x06; }
  if (hz <= 50) { actualHz = 50; return 0x07; }
  if (hz <= 100) { actualHz = 100; return 0x08; }
  if (hz <= 200) { actualHz = 200; return 0x09; }
  if (hz <= 400) { actualHz = 400; return 0x0A; }
  if (hz <= 800) { actualHz = 800; return 0x0B; }
  if (hz <= 1600) { actualHz = 1600; return 0x0C; }
  if (hz <= 3200) { actualHz = 3200; return 0x0D; }
  actualHz = 6400; return 0x0E;
}

bool BMI323::setSampleRateHz(uint16_t hz) {
  if (hz == 0) return false;
  odrCode_ = odrForHz(hz, sampleRateHz_);
  return configureDefaults();
}

bool BMI323::setLowPassFilterHz(uint16_t hz) {
  return setSampleRateHz(hz == 0 ? sampleRateHz_ : hz * 2);
}

bool BMI323::dataReady() {
  uint16_t status = 0;
  return readWords(STATUS, &status, 1) && (status & 0x00C0) == 0x00C0;
}

bool BMI323::configureInterruptPin(uint8_t pin, bool activeHigh,
                                   bool openDrain, bool latched) {
  if (pin != 1 && pin != 2) return false;
  uint16_t current = 0;
  if (!readWords(IO_INT_CTRL, &current, 1)) return false;
  uint8_t shift = pin == 1 ? 0 : 2;
  uint16_t mask = static_cast<uint16_t>(0x03u << shift);
  uint16_t value = static_cast<uint16_t>(((openDrain ? 1u : 0u) << 1 |
                                          (activeHigh ? 1u : 0u)) << shift);
  bool ok = writeWord(IO_INT_CTRL, (current & ~mask) | value);
  ok &= writeWord(INT_CONF, latched ? 0x0001 : 0x0000);
  return ok;
}

bool BMI323::routeDataReadyInterrupt(uint8_t pin, bool accel, bool gyro) {
  if (pin != 1 && pin != 2) return false;
  uint16_t current = 0;
  if (!readWords(INT_MAP2, &current, 1)) return false;
  const uint16_t route = pin;
  current &= static_cast<uint16_t>(~((0x03u << 10) | (0x03u << 8)));
  if (accel) current |= static_cast<uint16_t>(route << 10);
  if (gyro) current |= static_cast<uint16_t>(route << 8);
  return writeWord(INT_MAP2, current);
}

bool BMI323::routeDataReadyInterrupts(uint8_t accelPin, uint8_t gyroPin) {
  if ((accelPin != 1 && accelPin != 2) ||
      (gyroPin != 1 && gyroPin != 2))
    return false;
  uint16_t current = 0;
  if (!readWords(INT_MAP2, &current, 1)) return false;
  current &= static_cast<uint16_t>(~((0x03u << 10) | (0x03u << 8)));
  current |= static_cast<uint16_t>(accelPin << 10);
  current |= static_cast<uint16_t>(gyroPin << 8);
  return writeWord(INT_MAP2, current);
}

bool BMI323::enableFeatureEngine(bool enable) {
  return writeWord(FEATURE_CTRL, enable ? 0x0001 : 0x0000);
}

}  // namespace nimu
