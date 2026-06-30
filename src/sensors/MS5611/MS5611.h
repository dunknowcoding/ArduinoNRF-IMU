/*
  MS5611.h - ArduinoNRF-IMU driver for TE/MS MS5611 barometer.
*/
#ifndef ARDUINONRF_IMU_MS5611_H
#define ARDUINONRF_IMU_MS5611_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

namespace nimu {

class MS5611 {
 public:
  static constexpr uint8_t kAddrLow = 0x76;
  static constexpr uint8_t kAddrHigh = 0x77;

  enum Oversampling : uint8_t {
    OSR_256 = 0x00,
    OSR_512 = 0x02,
    OSR_1024 = 0x04,
    OSR_2048 = 0x06,
    OSR_4096 = 0x08,
  };

  bool begin(uint8_t address = kAddrHigh, TwoWire& wire = Wire);
  bool beginSPI(SPIClass& spi, uint8_t csPin);
  bool isConnected();
  bool reset();
  bool update();

  float temperatureC() const { return tempC_; }
  float pressurePa() const { return pressPa_; }
  float pressureHpa() const { return pressPa_ * 0.01f; }
  float altitudeM() const;

  void setSeaLevelPressureHpa(float hpa) { seaLevelHpa_ = hpa; }
  float seaLevelPressureHpa() const { return seaLevelHpa_; }
  bool calibrateAltitude(float knownAltitudeM = 0.0f);

  void setOversampling(Oversampling osr) { osr_ = osr; }
  Oversampling oversampling() const { return osr_; }

  bool readRaw(uint32_t& pressureD1, uint32_t& temperatureD2);
  uint16_t coefficient(uint8_t index) const {
    return (index < 8) ? prom_[index] : 0;
  }

 private:
  enum class Transport : uint8_t { None, I2C, SPI };

  bool beginAt(uint8_t address, TwoWire& wire);
  bool sendCommand(uint8_t command);
  bool readCommand(uint8_t command, uint8_t* data, uint8_t length);
  bool readADC(uint32_t& out);
  bool readPROM();
  bool readPROMWord(uint8_t index, uint16_t& out);
  uint8_t conversionDelayMs() const;

  TwoWire* wire_ = nullptr;
  SPIClass* spi_ = nullptr;
  Transport transport_ = Transport::None;
  uint8_t csPin_ = 0xFF;
  uint8_t address_ = kAddrHigh;
  uint16_t prom_[8]{};
  Oversampling osr_ = OSR_4096;
  float tempC_ = 0.0f;
  float pressPa_ = 0.0f;
  float seaLevelHpa_ = 1013.25f;
};

}  // namespace nimu

using nimu::MS5611;

#endif  // ARDUINONRF_IMU_MS5611_H
