#include "IMUBus.h"
#include "I2CRecovery.h"

namespace nimu {

void IMUBus::beginI2C(TwoWire& wire, uint8_t addr7, uint32_t clockHz) {
  mode_ = Mode::I2C;
  wire_ = &wire;
  address_ = addr7;
  clockHz_ = clockHz;
  wire_->begin();
  wire_->setClock(clockHz_);

  // Give the controller a bounded settling interval, then warm the selected
  // address before the first register transaction.
  delay(10);
  wire_->beginTransmission(address_);
  (void)wire_->endTransmission();
  delay(5);
}

void IMUBus::beginSPI(SPIClass& spi, uint8_t csPin, uint32_t clockHz,
                      uint8_t readFlag, uint8_t readDummyBytes) {
  mode_ = Mode::SPI;
  spi_ = &spi;
  csPin_ = csPin;
  clockHz_ = clockHz;
  spiReadFlag_ = readFlag;
  spiReadDummyBytes_ = readDummyBytes;
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

IMUStatus IMUBus::writeRegisterOnce(uint8_t reg, uint8_t value) {
  return (mode_ == Mode::I2C) ? i2cWriteOnce(reg, &value, 1)
                              : spiWrite(reg, &value, 1);
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

// TwoWire::end() is absent on the ESP8266 core; a plain begin() re-inits its
// software I2C, so make the teardown a no-op there.
static inline void imuWireEnd(TwoWire *wire) {
#if defined(ESP8266)
  (void)wire;
#else
  wire->end();
#endif
}

// Bus recovery is conditional. Releasing the peripheral and calling the
// pin-less begin() can discard a caller's custom pin mapping, so an idle bus
// must remain untouched.
IMUStatus IMUBus::recoverBus() {
  if (mode_ != Mode::I2C || wire_ == nullptr) {
    return IMUStatus::Ok;  // nothing to recover on SPI
  }
  // TwoWire::pinSDA()/pinSCL() exist only on a few cores (ArduinoNRF, classic
  // ESP32 2.x). Use the portable default SDA/SCL pin macros that every target
  // core defines.
#if !(defined(SDA) && defined(SCL))
  // Without the line identities, recovery cannot be performed safely.
  return IMUStatus::Ok;
#else
  const uint8_t sda = static_cast<uint8_t>(SDA);
  const uint8_t scl = static_cast<uint8_t>(SCL);

  // An idle bus has both lines released high.
  if (digitalRead(sda) == HIGH && digitalRead(scl) == HIGH) {
    return IMUStatus::Ok;
  }

  // A low line requires the bounded clock-pulse and STOP recovery sequence.
  imuWireEnd(wire_);
  pinMode(scl, OUTPUT);
  pinMode(sda, INPUT_PULLUP);

  // Pulse SCL until the slave lets SDA float high (max 9 bits + a margin).
  bool released = (digitalRead(sda) == HIGH);
  for (uint8_t i = 0; i < 18 && !released; ++i) {
    digitalWrite(scl, LOW);
    delayMicroseconds(5);
    digitalWrite(scl, HIGH);
    delayMicroseconds(5);
    released = (digitalRead(sda) == HIGH);
  }

  // Issue a STOP condition (SDA low -> high while SCL is high).
  pinMode(sda, OUTPUT);
  digitalWrite(sda, LOW);
  delayMicroseconds(5);
  digitalWrite(scl, HIGH);
  delayMicroseconds(5);
  digitalWrite(sda, HIGH);
  delayMicroseconds(5);

  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, INPUT_PULLUP);

  // Bring the hardware I2C back up at the configured clock.
  wire_->begin();
  wire_->setClock(clockHz_);
  return released ? IMUStatus::Ok : IMUStatus::BusError;
#endif  // SDA && SCL
}

IMUStatus IMUBus::ping() {
  if (mode_ != Mode::I2C || wire_ == nullptr) {
    return IMUStatus::Ok;  // SPI has no addressed ACK to probe
  }
  wire_->beginTransmission(address_);
  if (wire_->endTransmission() != 0) return IMUStatus::NotConnected;
  settleAfterTransaction();
  return IMUStatus::Ok;
}

// ---------------------------------------------------------------- I2C ------

IMUStatus IMUBus::i2cWrite(uint8_t reg, const uint8_t* buffer, size_t len) {
  // Retry bounded transient transfer errors; persistent errors fail closed.
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    wire_->beginTransmission(address_);
    wire_->write(reg);
    if (len != 0) wire_->write(buffer, len);
    if (wire_->endTransmission() == 0) {
      settleAfterTransaction();
      return IMUStatus::Ok;
    }
    detail::resetI2CControllerAfterError(*wire_);
    delay(2);
  }
  return IMUStatus::BusError;
}

IMUStatus IMUBus::i2cWriteOnce(uint8_t reg, const uint8_t* buffer, size_t len) {
  wire_->beginTransmission(address_);
  wire_->write(reg);
  if (len != 0) wire_->write(buffer, len);
  if (wire_->endTransmission() != 0) {
    detail::resetI2CControllerAfterError(*wire_);
    return IMUStatus::BusError;
  }
  settleAfterTransaction();
  return IMUStatus::Ok;
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

    // Retry the complete addressed read, including requestFrom(), so a short
    // transfer cannot leave stale bytes queued for the next transaction.
    bool complete = false;
    for (uint8_t attempt = 0; attempt < 3 && !complete; ++attempt) {
      wire_->beginTransmission(address_);
      wire_->write(static_cast<uint8_t>(reg + offset));
      // Repeated start (no stop) keeps the read tied to the address just set.
      if (wire_->endTransmission(false) != 0) {
        while (wire_->available()) (void)wire_->read();
        detail::resetI2CControllerAfterError(*wire_);
        delay(2);
        continue;
      }

      const uint8_t requested = static_cast<uint8_t>(chunk);
      const size_t got = static_cast<size_t>(
          wire_->requestFrom(address_, requested, static_cast<uint8_t>(true)));
      if (got == chunk) {
        for (size_t i = 0; i < chunk; ++i) {
          buffer[offset + i] = static_cast<uint8_t>(wire_->read());
        }
        settleAfterTransaction();
        complete = true;
      } else {
        while (wire_->available()) (void)wire_->read();
        detail::resetI2CControllerAfterError(*wire_);
        delay(2);
      }
    }
    if (!complete) return IMUStatus::BusError;
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
  settleAfterTransaction();
  return IMUStatus::Ok;
}

IMUStatus IMUBus::spiRead(uint8_t reg, uint8_t* buffer, size_t len) {
  spi_->beginTransaction(SPISettings(clockHz_, MSBFIRST, SPI_MODE0));
  digitalWrite(csPin_, LOW);
  spi_->transfer(reg | spiReadFlag_);  // set read flag
  for (uint8_t i = 0; i < spiReadDummyBytes_; ++i) {
    spi_->transfer(0x00);
  }
  for (size_t i = 0; i < len; ++i) {
    buffer[i] = spi_->transfer(0x00);
  }
  digitalWrite(csPin_, HIGH);
  spi_->endTransaction();
  settleAfterTransaction();
  return IMUStatus::Ok;
}

}  // namespace nimu
