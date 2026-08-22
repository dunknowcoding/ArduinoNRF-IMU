#include "IMUBus.h"

namespace nimu {

void IMUBus::beginI2C(TwoWire& wire, uint8_t addr7, uint32_t clockHz) {
  mode_ = Mode::I2C;
  wire_ = &wire;
  address_ = addr7;
  clockHz_ = clockHz;
  wire_->begin();
  wire_->setClock(clockHz_);

  // Let the controller settle, then spend a throwaway transaction on it.
  //
  // The first transfer after Wire.begin() is unreliable on ESP32: it comes
  // back NACKed or with a bus fault on a bus that is fine a millisecond later.
  // Every driver's first act after this call is a WHO_AM_I, so that read got
  // the bad transfer, the identity check failed, and begin() reported "not
  // found" for a part sitting right there answering. A real BMI270 reading
  // 0x24 perfectly well from a scanner was rejected this way.
  //
  // A driver cannot assume its caller warmed the bus, so warm it here. The
  // probe costs microseconds and its result is deliberately ignored.
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

// Recovering a bus that is not stuck is not a no-op, it is damage.
//
// The only way to bit-bang the lines is to release the peripheral first, and
// the only way to bring it back is wire_->begin() - which takes no pins here,
// so it reopens on the core's DEFAULT SDA/SCL. Any custom pin mapping the
// sketch chose is silently lost. Worse, on ESP32 core 3.x an end() followed by
// a pin-less begin() can leave the peripheral in a state where every later
// transfer returns error 4, so a healthy bus goes permanently dead.
//
// Most drivers call this unconditionally at the top of beginI2C(), which meant
// every single open tore the bus down and rebuilt it. On a bench with a BNO085
// and a BMI270 sharing one bus, the first driver to initialise killed I2C for
// everything after it - including itself on the next attempt.
//
// So: look before leaping. An idle I2C bus has both lines pulled high. If they
// are, there is nothing to recover and we must not touch the peripheral.
IMUStatus IMUBus::recoverBus() {
  if (mode_ != Mode::I2C || wire_ == nullptr) {
    return IMUStatus::Ok;  // nothing to recover on SPI
  }
  // TwoWire::pinSDA()/pinSCL() exist only on a few cores (ArduinoNRF, classic
  // ESP32 2.x). Use the portable default SDA/SCL pin macros that every target
  // core defines.
#if !(defined(SDA) && defined(SCL))
  // Without knowing the lines we cannot tell a stuck bus from a healthy one,
  // and a blind end() + begin() is more likely to break a working bus than to
  // rescue a jammed one. Leave it alone.
  return IMUStatus::Ok;
#else
  const uint8_t sda = static_cast<uint8_t>(SDA);
  const uint8_t scl = static_cast<uint8_t>(SCL);

  // Reading the pads works while the peripheral owns the pins on every core
  // this library targets, so this costs nothing and risks nothing.
  if (digitalRead(sda) == HIGH && digitalRead(scl) == HIGH) {
    return IMUStatus::Ok;  // idle and healthy - do not touch it
  }

  // Genuinely stuck: a device is holding SDA low and nobody can start a
  // transfer. Now the teardown is worth its cost. Note that the bus comes back
  // on the default pins - unavoidable, since begin() is the only way back.
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
  return (wire_->endTransmission() == 0) ? IMUStatus::Ok
                                         : IMUStatus::NotConnected;
}

// ---------------------------------------------------------------- I2C ------

IMUStatus IMUBus::i2cWrite(uint8_t reg, const uint8_t* buffer, size_t len) {
  // Retry a refused write, the same way i2cRead retries its addressing phase.
  //
  // A single dropped write is not a dead device, and treating it as one is
  // ruinous for anything that writes a lot. Measured on a healthy bench bus, a
  // BMI270 configuration upload - roughly 1500 transactions - saw fifteen
  // refusals. One of them landed on INIT_CTRL, so the part was never told to
  // begin initialising: the whole 8192-byte image went nowhere and the chip
  // reported not_init, which reads exactly like a sensor that ignored you.
  //
  // Three attempts. The bus recovers within a couple of milliseconds or it is
  // genuinely broken.
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    wire_->beginTransmission(address_);
    wire_->write(reg);
    if (len != 0) wire_->write(buffer, len);
    if (wire_->endTransmission() == 0) return IMUStatus::Ok;
    delay(2);
  }
  return IMUStatus::BusError;
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

    // Two attempts, not one.
    //
    // A transfer that NACKs does not always mean the device is absent: the
    // controller is briefly unhappy after a failed transfer to a different
    // address, and the read that follows fails even though its target is
    // answering. begin() walks the primary address before the alternate, so
    // on any board strapped to the alternate the identity read is always the
    // one immediately after a NACK - and a real BMI270 reporting 0x24 to a
    // scanner was rejected as "not found" every time because of it.
    bool addressed = false;
    for (uint8_t attempt = 0; attempt < 2 && !addressed; ++attempt) {
      wire_->beginTransmission(address_);
      wire_->write(static_cast<uint8_t>(reg + offset));
      // Repeated start (no stop) keeps the read tied to the address just set.
      addressed = (wire_->endTransmission(false) == 0);
      if (!addressed) delay(2);
    }
    if (!addressed) {
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
  for (uint8_t i = 0; i < spiReadDummyBytes_; ++i) {
    spi_->transfer(0x00);
  }
  for (size_t i = 0; i < len; ++i) {
    buffer[i] = spi_->transfer(0x00);
  }
  digitalWrite(csPin_, HIGH);
  spi_->endTransaction();
  return IMUStatus::Ok;
}

}  // namespace nimu
