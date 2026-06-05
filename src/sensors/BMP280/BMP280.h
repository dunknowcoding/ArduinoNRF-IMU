/*
  BMP280.h - NiusIMU driver for the Bosch BMP280 barometric pressure + die
  temperature sensor (the second chip on a GY-91 board).

  It is not an inertial sensor, so it does not derive from IMUSensor; it has its
  own small surface but reuses the same IMUBus register layer and the same
  unit-first philosophy: pressure in pascals/hPa, temperature in degrees C,
  altitude in metres.

  Quick start (I2C, GY-91 default address 0x76):

      #include <BMP280.h>
      BMP280 baro;

      void setup() {
        Serial.begin(115200);
        baro.begin();                       // Wire @ 0x76
        baro.setSeaLevelPressureHpa(1013.25);// set local QNH for accurate alt
      }

      void loop() {
        baro.update();
        float hpa = baro.pressureHpa();
        float t   = baro.temperatureC();
        float alt = baro.altitudeM();
        delay(200);
      }

  "Calibration": the BMP280 carries factory trim coefficients (read once in
  begin()), so pressure/temperature need no user calibration. Altitude, however,
  is derived from pressure against a sea-level reference; enter your local QNH
  with setSeaLevelPressureHpa() - that is the user-facing calibration step.
*/
#ifndef NIUSIMU_BMP280_H
#define NIUSIMU_BMP280_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "../../imu/IMUBus.h"
#include "../../imu/IMUTypes.h"
#include "BMP280_Registers.h"

namespace nimu {

class BMP280 {
 public:
  BMP280() = default;

  // Oversampling / filter / mode codes for setSampling(), exposed on the class
  // so sketches can write BMP280::OSRS_X16 without the internal namespace.
  static constexpr uint8_t OSRS_SKIP = bmp280::OSRS_SKIP;
  static constexpr uint8_t OSRS_X1 = bmp280::OSRS_X1;
  static constexpr uint8_t OSRS_X2 = bmp280::OSRS_X2;
  static constexpr uint8_t OSRS_X4 = bmp280::OSRS_X4;
  static constexpr uint8_t OSRS_X8 = bmp280::OSRS_X8;
  static constexpr uint8_t OSRS_X16 = bmp280::OSRS_X16;
  static constexpr uint8_t FILTER_OFF = bmp280::FILTER_OFF;
  static constexpr uint8_t FILTER_2 = bmp280::FILTER_2;
  static constexpr uint8_t FILTER_4 = bmp280::FILTER_4;
  static constexpr uint8_t FILTER_8 = bmp280::FILTER_8;
  static constexpr uint8_t FILTER_16 = bmp280::FILTER_16;
  static constexpr uint8_t MODE_SLEEP = bmp280::MODE_SLEEP;
  static constexpr uint8_t MODE_FORCED = bmp280::MODE_FORCED;
  static constexpr uint8_t MODE_NORMAL = bmp280::MODE_NORMAL;

  // ------------------------------------------------------------ lifecycle --

  /**
   * Bring the sensor up on I2C with a sensible weather/indoor configuration
   * (pressure x16 + temp x2 oversampling, IIR filter 16, normal mode).
   * @param address 0x76 (default, GY-91) or 0x77.
   * @param wire    the TwoWire bus (Wire by default).
   * @return true once the chip id (0x58) is confirmed and trim is loaded.
   */
  bool begin(uint8_t address = bmp280::kAddrPrimary, TwoWire& wire = Wire);

  /** Begin in SPI mode (GY-91 exposes the BMP280 on the shared SPI too). */
  bool beginSPI(SPIClass& spi, uint8_t csPin);

  /** Chip id register (0x58 for BMP280, 0x60 for a BME280). */
  uint8_t chipId();

  /** True if the chip id matches and the device answers on the bus. */
  bool isConnected();

  // -------------------------------------------------------------- reading --

  /** Read one fresh pressure + temperature sample and compensate it. */
  bool update();

  float temperatureC() const { return tempC_; }      ///< degrees C
  float pressurePa() const { return pressPa_; }       ///< pascals
  float pressureHpa() const { return pressPa_ * 0.01f; }  ///< hPa (= mbar)

  /**
   * Altitude in metres from the last pressure sample, using the international
   * barometric formula against the configured sea-level pressure. Accuracy
   * depends entirely on setSeaLevelPressureHpa() matching your local weather.
   */
  float altitudeM() const;

  // -------------------------------------------------- altitude reference ---

  /** Set local sea-level pressure (QNH) in hPa for accurate altitude. */
  void setSeaLevelPressureHpa(float hpa) { seaLevelHpa_ = hpa; }
  float seaLevelPressureHpa() const { return seaLevelHpa_; }

  /**
   * One-shot helper: take the current reading as a known altitude (often 0 at
   * your location) and back-solve the sea-level pressure. Hold the board still
   * at a known height, call this, and altitudeM() is then referenced to it.
   */
  bool calibrateAltitude(float knownAltitudeM = 0.0f);

  // ----------------------------------------------------------- advanced ----

  /**
   * Override the oversampling / filter / mode. Codes are the bmp280::OSRS_*,
   * bmp280::FILTER_* and bmp280::MODE_* constants. Call after begin().
   */
  bool setSampling(uint8_t pressureOsrs, uint8_t tempOsrs, uint8_t filter,
                   uint8_t mode = bmp280::MODE_NORMAL,
                   uint8_t standby = bmp280::STANDBY_0_5MS);

  /** Soft-reset the chip to its power-on defaults. */
  bool reset();

 private:
  bool readCalibration();
  bool applyDefaultConfig();

  IMUBus bus_;

  // Factory trim (datasheet names). T1/P1 are unsigned, the rest signed.
  uint16_t digT1_ = 0;
  int16_t digT2_ = 0, digT3_ = 0;
  uint16_t digP1_ = 0;
  int16_t digP2_ = 0, digP3_ = 0, digP4_ = 0, digP5_ = 0, digP6_ = 0;
  int16_t digP7_ = 0, digP8_ = 0, digP9_ = 0;

  int32_t tFine_ = 0;     ///< shared term linking the temp and pressure maths
  float tempC_ = 0.0f;
  float pressPa_ = 0.0f;
  float seaLevelHpa_ = 1013.25f;  ///< standard atmosphere until the user sets it
};

}  // namespace nimu

using nimu::BMP280;

#endif  // NIUSIMU_BMP280_H
