/*
  IMUBus.h - One register-access surface for both I2C and SPI.

  Every sensor driver in this library talks to its chip through an IMUBus
  instead of touching Wire or SPI directly. That keeps the drivers tiny and,
  just as importantly for the ArduinoNRF board package, it puts ALL bus traffic
  in one place so I2C/SPI bring-up and debugging happens here once.

  The same driver code therefore works whether the chip is wired for I2C or
  SPI - you only change how you call begin().

  Chunking: the ArduinoNRF TwoWire (like classic Arduino) carries a 32-byte
  buffer, so a burst longer than 32 bytes would silently truncate. readRegisters
  splits long reads into <=32-byte register-addressed bursts automatically, so
  drivers can ask for a full FIFO frame without worrying about the limit.
*/
#ifndef NIUSIMU_IMUBUS_H
#define NIUSIMU_IMUBUS_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "IMUTypes.h"

namespace nimu {

class IMUBus {
 public:
  /** Largest single I2C burst, matching the TwoWire buffer on this core. */
  static constexpr size_t kMaxI2CBurst = 32;

  IMUBus() = default;

  /**
   * Configure this bus for I2C.
   * @param wire   the TwoWire instance (Wire by default, Wire1 for the 2nd bus)
   * @param addr7  the 7-bit device address
   * @param clockHz I2C clock; 400000 (fast mode) is safe for every IMU here
   */
  void beginI2C(TwoWire& wire, uint8_t addr7, uint32_t clockHz = 400000);

  /**
   * Configure this bus for SPI. Register reads set the MSB of the address
   * (the convention used by InvenSense MPU/ICM parts); pass a different
   * readFlag if your chip differs.
   * @param spi     the SPIClass instance (SPI by default)
   * @param csPin   chip-select pin (driven LOW during a transfer)
   * @param clockHz SPI clock in Hz
   */
  void beginSPI(SPIClass& spi, uint8_t csPin, uint32_t clockHz = 1000000,
                uint8_t readFlag = 0x80);

  /** True if this bus is in I2C mode (vs SPI). */
  bool isI2C() const { return mode_ == Mode::I2C; }

  /** The configured 7-bit I2C address (0 in SPI mode). */
  uint8_t address() const { return address_; }

  /** Change the bus clock after begin*(). */
  void setClockHz(uint32_t clockHz);

  // --- Register primitives. All return IMUStatus::Ok on success. ---

  /** Write one 8-bit register. */
  IMUStatus writeRegister(uint8_t reg, uint8_t value);

  /** Read one 8-bit register into @p value. */
  IMUStatus readRegister(uint8_t reg, uint8_t& value);

  /** Read @p len consecutive registers starting at @p reg (auto-chunked). */
  IMUStatus readRegisters(uint8_t reg, uint8_t* buffer, size_t len);

  /** Write @p len consecutive registers starting at @p reg. */
  IMUStatus writeRegisters(uint8_t reg, const uint8_t* buffer, size_t len);

  /**
   * Set/clear bits of a register in one read-modify-write.
   * Only the bits in @p mask are touched; @p value supplies their new state.
   */
  IMUStatus updateRegister(uint8_t reg, uint8_t mask, uint8_t value);

  /**
   * Probe for an I2C device by issuing an empty transaction.
   * Returns Ok if the device ACKs its address. Always Ok in SPI mode.
   */
  IMUStatus ping();

 private:
  enum class Mode : uint8_t { None, I2C, SPI };

  IMUStatus i2cWrite(uint8_t reg, const uint8_t* buffer, size_t len);
  IMUStatus i2cRead(uint8_t reg, uint8_t* buffer, size_t len);
  IMUStatus spiWrite(uint8_t reg, const uint8_t* buffer, size_t len);
  IMUStatus spiRead(uint8_t reg, uint8_t* buffer, size_t len);

  Mode mode_ = Mode::None;
  TwoWire* wire_ = nullptr;
  SPIClass* spi_ = nullptr;
  uint8_t address_ = 0;
  uint8_t csPin_ = 0xFF;
  uint8_t spiReadFlag_ = 0x80;
  uint32_t clockHz_ = 400000;
};

}  // namespace nimu

#endif  // NIUSIMU_IMUBUS_H
