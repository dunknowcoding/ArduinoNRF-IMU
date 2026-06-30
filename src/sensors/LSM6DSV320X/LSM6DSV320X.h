/* LSM6DSV320X.h - Dual low-g/high-g IMU driver with embedded AI access. */
#ifndef ARDUINONRF_IMU_LSM6DSV320X_H
#define ARDUINONRF_IMU_LSM6DSV320X_H

#include "st/lsm6dsv320x_reg.h"
#include "../LSM6DSV/LSM6DSV.h"
#include "LSM6DSV320X_Registers.h"

namespace nimu {

class LSM6DSV320X : public LSM6DSV {
 public:
  enum class UcfOperation : uint8_t {
    Read = 0,
    Write = 1,
    Delay = 2,
    PollSet = 3,
    PollReset = 4,
  };

  struct UcfLineExtended {
    UcfOperation operation;
    uint8_t address;
    uint8_t data;
  };

  LSM6DSV320X();
  LSM6DSV320X(const LSM6DSV320X&) = delete;
  LSM6DSV320X& operator=(const LSM6DSV320X&) = delete;

  bool begin() override;
  bool beginI2C(TwoWire& wire, uint8_t address) override;
  bool beginSPI(SPIClass& spi, uint8_t csPin) override;
  bool isConnected() override;
  bool setGyroRangeDps(uint16_t maxDps) override;

  bool setHighGRangeG(uint16_t maxG);
  bool setHighGSampleRateHz(uint16_t hz);
  bool routeHighGDataReady(uint8_t pin, bool enable = true);
  bool configureHighGWakeup(uint16_t thresholdMg, uint8_t duration,
                            uint8_t interruptPin = 1);
  bool highGWakeupDetected();
  bool highGShockState();
  bool readHighG(Vec3& accelG);

  template <typename UcfLine>
  bool loadUcf(const UcfLine* configuration, size_t count) {
    if (configuration == nullptr || count == 0) return false;
    for (size_t i = 0; i < count; ++i) {
      if (bus_.writeRegister(configuration[i].address,
                             configuration[i].data) != IMUStatus::Ok)
        return false;
    }
    return true;
  }
  bool loadUcfExtended(const UcfLineExtended* configuration, size_t count,
                       uint16_t pollTimeoutMs = 100);
  bool setMlcEnabled(bool enable = true);
  bool setFsmEnabled(uint8_t programMask);
  bool readMlcOutput(uint8_t index, uint8_t& value);
  bool readFsmOutput(uint8_t index, uint8_t& value);
  bool enableAdaptiveSelfConfiguration(bool enable = true);
  bool adaptiveSelfConfigurationActive();
  bool setI3CInterrupts(bool enable = true);

  /*
    Complete ST register API context. Pass this to any function declared in
    st/lsm6dsv320x_reg.h when a feature has no Arduino-style wrapper.
  */
  stmdev_ctx_t* stContext() { return &stContext_; }
  const stmdev_ctx_t* stContext() const { return &stContext_; }

 private:
  static int32_t stWrite(void* handle, uint8_t reg, uint8_t* data,
                         uint16_t length);
  static int32_t stRead(void* handle, uint8_t reg, uint8_t* data,
                        uint16_t length);
  static void stDelay(uint32_t milliseconds);

  stmdev_ctx_t stContext_{};
  float highGLsbPerG_ = 1024.5902f;
};

}  // namespace nimu

using nimu::LSM6DSV320X;

#endif
