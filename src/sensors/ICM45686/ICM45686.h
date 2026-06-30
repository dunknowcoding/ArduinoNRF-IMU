/* ICM45686.h - ArduinoNRF-IMU driver for TDK InvenSense ICM-45686. */
#ifndef ARDUINONRF_IMU_ICM45686_H
#define ARDUINONRF_IMU_ICM45686_H

#include "../../imu/IMUSensor.h"
#include "ICM45686_Registers.h"

namespace nimu {

class ICM45686 : public IMUSensor {
 public:
  enum class AuxiliaryMode : uint8_t {
    Disabled,
    OisSpi,
    I2CMaster,
    I2CBypass,
  };

  ICM45686() { name_ = "ICM45686"; }

  bool begin() override;
  bool beginI2C(TwoWire& wire, uint8_t address) override;
  bool beginSPI(SPIClass& spi, uint8_t csPin) override;
  uint8_t whoAmI() override;
  bool isConnected() override;
  bool update() override;
  bool setAccelRangeG(uint16_t maxG) override;
  bool setGyroRangeDps(uint16_t maxDps) override;
  bool setLowPassFilterHz(uint16_t hz) override;
  bool setSampleRateHz(uint16_t hz) override;

  bool reset();
  bool dataReady(uint8_t pin = 1);
  bool routeDataReadyInterrupt(uint8_t pin, bool enable = true,
                               bool includeAux = false);
  bool configureInterruptPin(uint8_t pin, bool activeLow,
                             bool openDrain, bool latched = false);
  bool enableAuxI2CMaster(bool enable = true);
  bool setAuxiliaryMode(AuxiliaryMode mode);
  bool enableAuxOisSpi(bool enable = true);
  bool enableAuxI2CBypass(bool enable = true);
  AuxiliaryMode auxiliaryMode();
  bool setAuxAddress(uint8_t address8);
  bool auxReadRegister(uint8_t reg, uint8_t& value,
                       uint16_t timeoutUs = 2000);
  bool auxWriteRegister(uint8_t reg, uint8_t value,
                        uint16_t timeoutUs = 2000);
  bool configureQMC6309();
  bool readQMC6309(Vec3& magUT);

  struct RawSample { int16_t ax, ay, az, gx, gy, gz, temp; };
  bool readRaw(RawSample& out);

 protected:
  bool indirectRead(uint16_t address, uint8_t& value);
  bool indirectWrite(uint16_t address, uint8_t value);

  float accelLsbPerG_ = 2048.0f;
  float gyroLsbPerDps_ = 8.192f;
  uint8_t accelFsCode_ = 0;
  uint8_t gyroFsCode_ = 0;
  uint8_t odrCode_ = 9;
  uint8_t auxAddress8_ = 0x7C;
  bool qmcEnabled_ = false;
};

}  // namespace nimu

using nimu::ICM45686;
using nimu::IMUCalibration;
using nimu::IMUData;
using nimu::Vec3;

#endif
