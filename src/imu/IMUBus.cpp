#include "IMUBus.h"

namespace nimu {

void IMUBus::beginI2C(TwoWire& wire, uint8_t addr7, uint32_t clockHz) {
  mode_ = Mode::I2C;
  wire_ = &wire;
  address_ = addr7;
  clockHz_ = clockHz;
  wire_->begin();
  wire_->setClock(clockHz_);
}

void IMUBus::beginSPI(SPIClass& spi, uint8_t csPin, uint32_t clockHz,
                      uint8_t readFlag) {
  mode_ = Mode::SPI;
  spi_ = &spi;
  csPin_ = csPin;
  clockHz_ = clockHz;
  spiReadFlag_ = readFlag;
  pinMode(csPin_, OUTPUT);
  digitalWrite(csPin_, HIGH);  // CS idle high
  spi_->begin();
}

void IMUBus::setClockHz(uint32_t clockHz) {
  clockHz_ = clockHz;
  if (mode_ == Mode::I2C && wire_ != nullptr) {
    wire_->setClock(clockHz_);
  }
}

IMUStatus IMUBus::writeRegister(uint8_t reg, uint8_t value) {
  return writeRegisters(reg, &value, 1);
}

IMUStatus IMUBus::readRegister(uint8_t reg, uint8_t& value) {
  return readRegisters(reg, &value, 1);
}

IMUStatus IMUBus::readRegisters(uint8_t reg, uint8_t* buffer, size_t len) {
  if (buffer == nullptr || len == 0) {
    return IMUStatus::BadParameter;
  }
  return (mode_ == Mode::I2C) ? i2cRead(reg, buffer, len)
                              : spiRead(reg, buffer, len);
}

IMUStatus IMUBus::writeRegisters(uint8_t reg, const uint8_t* buffer,
                                 size_t len) {
  if (buffer == nullptr || len == 0) {
    return IMUStatus::BadParameter;
  }
  return (mode_ == Mode::I2C) ? i2cWrite(reg, buffer, len)
                              : spiWrite(reg, buffer, len);
}

IMUStatus IMUBus::updateRegister(uint8_t reg, uint8_t mask, uint8_t value) {
  uint8_t current = 0;
  IMUStatus status = readRegister(reg, current);
  if (status != IMUStatus::Ok) {
    return status;
  }
  current = (current & ~mask) | (value & mask);
  return writeRegister(reg, current);
}

IMUStatus IMUBus::ping() {
  if (mode_ != Mode::I2C || wire_ == nullptr) {
    return IMUStatus::Ok;  // SPI has no addressed ACK to probe
  }
  wire_->beginTransmission(address_);
  return (wire_->endTransmission() == 0) ? IMUStatus::Ok
                                         : IMUStatus::NotConnected;
}

// ---------------------------------------------------------------- I2C ------

IMUStatus IMUBus::i2cWrite(uint8_t reg, const uint8_t* buffer, size_t len) {
  wire_->beginTransmission(address_);
  wire_->write(reg);
  wire_->write(buffer, len);
  return (wire_->endTransmission() == 0) ? IMUStatus::Ok : IMUStatus::BusError;
}

IMUStatus IMUBus::i2cRead(uint8_t reg, uint8_t* buffer, size_t len) {
  size_t offset = 0;
  while (offset < len) {
    // Burst no more than the TwoWire buffer can hold; re-address each chunk so
    // this works on plain register-addressed (auto-incrementing) devices.
    size_t chunk = len - offset;
    if (chunk > kMaxI2CBurst) {
      chunk = kMaxI2CBurst;
    }

    wire_->beginTransmission(address_);
    wire_->write(static_cast<uint8_t>(reg + offset));
    // Repeated start (no stop) keeps the read tied to the address we just set.
    if (wire_->endTransmission(false) != 0) {
      return IMUStatus::BusError;
    }

    size_t got = wire_->requestFrom(address_, static_cast<uint8_t>(chunk), true);
    if (got != chunk) {
      return IMUStatus::BusError;
    }
    for (size_t i = 0; i < chunk; ++i) {
      buffer[offset + i] = static_cast<uint8_t>(wire_->read());
    }
    offset += chunk;
  }
  return IMUStatus::Ok;
}

// ---------------------------------------------------------------- SPI ------

IMUStatus IMUBus::spiWrite(uint8_t reg, const uint8_t* buffer, size_t len) {
  spi_->beginTransaction(SPISettings(clockHz_, MSBFIRST, SPI_MODE0));
  digitalWrite(csPin_, LOW);
  spi_->transfer(reg & ~spiReadFlag_);  // clear read flag => write
  for (size_t i = 0; i < len; ++i) {
    spi_->transfer(buffer[i]);
  }
  digitalWrite(csPin_, HIGH);
  spi_->endTransaction();
  return IMUStatus::Ok;
}

IMUStatus IMUBus::spiRead(uint8_t reg, uint8_t* buffer, size_t len) {
  spi_->beginTransaction(SPISettings(clockHz_, MSBFIRST, SPI_MODE0));
  digitalWrite(csPin_, LOW);
  spi_->transfer(reg | spiReadFlag_);  // set read flag
  for (size_t i = 0; i < len; ++i) {
    buffer[i] = spi_->transfer(0x00);
  }
  digitalWrite(csPin_, HIGH);
  spi_->endTransaction();
  return IMUStatus::Ok;
}

}  // namespace nimu
