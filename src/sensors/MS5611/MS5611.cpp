#include "MS5611.h"

#if defined(__AVR__)
#include <math.h>  // avr-libc provides powf; the forward-decl below conflicts on AVR
#else
extern "C" float powf(float, float);
#endif

namespace nimu {

namespace {
constexpr uint8_t CMD_RESET = 0x1E;
constexpr uint8_t CMD_ADC_READ = 0x00;
constexpr uint8_t CMD_CONVERT_D1 = 0x40;
constexpr uint8_t CMD_CONVERT_D2 = 0x50;
constexpr uint8_t CMD_PROM_READ = 0xA0;
}  // namespace

bool MS5611::begin(uint8_t address, TwoWire& wire) {
  if (beginAt(address, wire)) {
    return true;
  }
  if (address == kAddrHigh) {
    return beginAt(kAddrLow, wire);
  }
  return false;
}

bool MS5611::beginAt(uint8_t address, TwoWire& wire) {
  wire_ = &wire;
  spi_ = nullptr;
  transport_ = Transport::I2C;
  address_ = address;
  wire_->begin();
  wire_->setClock(400000);
  return isConnected() && reset() && readPROM();
}

bool MS5611::beginSPI(SPIClass& spi, uint8_t csPin) {
  wire_ = nullptr;
  spi_ = &spi;
  transport_ = Transport::SPI;
  csPin_ = csPin;
  pinMode(csPin_, OUTPUT);
  digitalWrite(csPin_, HIGH);
  spi_->begin();
  return reset() && readPROM();
}

bool MS5611::isConnected() {
  if (transport_ == Transport::SPI) {
    uint16_t coefficient = 0;
    return readPROMWord(1, coefficient) && coefficient != 0 &&
           coefficient != 0xFFFF;
  }
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(address_);
  return wire_->endTransmission() == 0;
}

bool MS5611::sendCommand(uint8_t command) {
  if (transport_ == Transport::SPI) {
    if (spi_ == nullptr || csPin_ == 0xFF) return false;
    spi_->beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWrite(csPin_, LOW);
    spi_->transfer(command);
    digitalWrite(csPin_, HIGH);
    spi_->endTransaction();
    return true;
  }
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(address_);
  wire_->write(command);
  return wire_->endTransmission() == 0;
}

bool MS5611::readCommand(uint8_t command, uint8_t* data, uint8_t length) {
  if (data == nullptr || length == 0) return false;
  if (transport_ == Transport::SPI) {
    if (spi_ == nullptr || csPin_ == 0xFF) return false;
    spi_->beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWrite(csPin_, LOW);
    spi_->transfer(command);
    for (uint8_t i = 0; i < length; ++i) data[i] = spi_->transfer(0x00);
    digitalWrite(csPin_, HIGH);
    spi_->endTransaction();
    return true;
  }
  if (!sendCommand(command) ||
      wire_->requestFrom(address_, length, static_cast<uint8_t>(true)) !=
          length) {
    return false;
  }
  for (uint8_t i = 0; i < length; ++i) data[i] = wire_->read();
  return true;
}

bool MS5611::reset() {
  if (!sendCommand(CMD_RESET)) {
    return false;
  }
  delay(3);
  return true;
}

uint8_t MS5611::conversionDelayMs() const {
  switch (osr_) {
    case OSR_256: return 1;
    case OSR_512: return 2;
    case OSR_1024: return 3;
    case OSR_2048: return 5;
    default: return 10;
  }
}

bool MS5611::readADC(uint32_t& out) {
  uint8_t raw[3];
  if (!readCommand(CMD_ADC_READ, raw, sizeof(raw))) return false;
  out = (static_cast<uint32_t>(raw[0]) << 16) |
        (static_cast<uint32_t>(raw[1]) << 8) |
        static_cast<uint32_t>(raw[2]);
  return out != 0;
}

bool MS5611::readPROMWord(uint8_t index, uint16_t& out) {
  uint8_t raw[2];
  if (index > 7 ||
      !readCommand(CMD_PROM_READ + (index * 2), raw, sizeof(raw))) return false;
  out = (static_cast<uint16_t>(raw[0]) << 8) |
        static_cast<uint16_t>(raw[1]);
  return true;
}

bool MS5611::readPROM() {
  for (uint8_t i = 0; i < 8; ++i) {
    if (!readPROMWord(i, prom_[i])) {
      return false;
    }
  }
  return prom_[1] != 0 && prom_[2] != 0 && prom_[3] != 0 &&
         prom_[4] != 0 && prom_[5] != 0 && prom_[6] != 0;
}

bool MS5611::readRaw(uint32_t& pressureD1, uint32_t& temperatureD2) {
  if (!sendCommand(CMD_CONVERT_D1 | static_cast<uint8_t>(osr_))) {
    return false;
  }
  delay(conversionDelayMs());
  if (!readADC(pressureD1)) {
    return false;
  }

  if (!sendCommand(CMD_CONVERT_D2 | static_cast<uint8_t>(osr_))) {
    return false;
  }
  delay(conversionDelayMs());
  return readADC(temperatureD2);
}

bool MS5611::update() {
  uint32_t d1 = 0;
  uint32_t d2 = 0;
  if (!readRaw(d1, d2)) {
    return false;
  }

  int64_t dT = static_cast<int64_t>(d2) -
               (static_cast<int64_t>(prom_[5]) << 8);
  int64_t temp = 2000 + ((dT * static_cast<int64_t>(prom_[6])) >> 23);
  int64_t off = (static_cast<int64_t>(prom_[2]) << 16) +
                ((static_cast<int64_t>(prom_[4]) * dT) >> 7);
  int64_t sens = (static_cast<int64_t>(prom_[1]) << 15) +
                 ((static_cast<int64_t>(prom_[3]) * dT) >> 8);

  int64_t t2 = 0;
  int64_t off2 = 0;
  int64_t sens2 = 0;
  if (temp < 2000) {
    int64_t diff = temp - 2000;
    t2 = (dT * dT) >> 31;
    off2 = (5 * diff * diff) >> 1;
    sens2 = (5 * diff * diff) >> 2;
    if (temp < -1500) {
      int64_t cold = temp + 1500;
      off2 += 7 * cold * cold;
      sens2 += (11 * cold * cold) >> 1;
    }
  }

  temp -= t2;
  off -= off2;
  sens -= sens2;

  int64_t pressure = (((static_cast<int64_t>(d1) * sens) >> 21) - off) >> 15;
  tempC_ = temp * 0.01f;
  pressPa_ = static_cast<float>(pressure);
  return true;
}

float MS5611::altitudeM() const {
  float ratio = pressureHpa() / seaLevelHpa_;
  return 44330.0f * (1.0f - powf(ratio, 0.1902949572f));
}

bool MS5611::calibrateAltitude(float knownAltitudeM) {
  if (!update()) {
    return false;
  }
  float base = 1.0f - (knownAltitudeM / 44330.0f);
  if (base <= 0.0f) {
    return false;
  }
  seaLevelHpa_ = pressureHpa() / powf(base, 5.2553f);
  return true;
}

}  // namespace nimu
