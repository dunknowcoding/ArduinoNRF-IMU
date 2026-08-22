/* ArduinoNRF-IMU I2C driver for CEVA BNO085/BNO086 sensor hubs. */
#ifndef ARDUINONRF_IMU_BNO08X_H
#define ARDUINONRF_IMU_BNO08X_H

#include "../../imu/IMUSensor.h"
#include "BNO08x_Protocol.h"

namespace nimu {

class BNO08x : public IMUSensor {
 public:
  struct Quaternion {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float accuracyRad = 0.0f;
    uint8_t accuracy = 0;
  };

  struct ProductInfo {
    uint8_t resetReason = 0;
    uint8_t versionMajor = 0;
    uint8_t versionMinor = 0;
    uint16_t versionPatch = 0;
    uint32_t partNumber = 0;
    uint32_t buildNumber = 0;
  };

  BNO08x() {
    name_ = "BNO08x";
    hasMag_ = true;
  }

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

  bool enableReport(uint8_t reportId, uint32_t intervalUs);
  bool enableLinearAcceleration(uint32_t intervalUs) {
    return enableReport(bno08x::SENSOR_LINEAR_ACCELERATION, intervalUs);
  }
  bool enableGravity(uint32_t intervalUs) {
    return enableReport(bno08x::SENSOR_GRAVITY, intervalUs);
  }
  bool enableStepCounter(uint32_t intervalUs) {
    return enableReport(bno08x::SENSOR_STEP_COUNTER, intervalUs);
  }
  bool enableGameRotationVector(uint32_t intervalUs) {
    return enableReport(bno08x::SENSOR_GAME_ROTATION_VECTOR, intervalUs);
  }
  bool enableGeomagneticRotationVector(uint32_t intervalUs) {
    return enableReport(bno08x::SENSOR_GEOMAGNETIC_ROTATION_VECTOR, intervalUs);
  }
  bool enableTapDetector(uint32_t intervalUs = 0) {
    return enableReport(bno08x::SENSOR_TAP_DETECTOR, intervalUs);
  }
  bool enableStabilityClassifier(uint32_t intervalUs = 100000) {
    return enableReport(bno08x::SENSOR_STABILITY_CLASSIFIER, intervalUs);
  }
  bool enableActivityClassifier(uint32_t intervalUs = 100000) {
    return enableReport(bno08x::SENSOR_PERSONAL_ACTIVITY_CLASSIFIER, intervalUs);
  }
  void configurePins(int8_t interruptPin, int8_t resetPin = -1,
                     int8_t wakePin = -1);
  bool hardwareReset(uint16_t bootDelayMs = 300);

  // Why the last begin/beginI2C failed. A bare bool cannot tell "nothing is
  // there" from "something is there but is not speaking SHTP", and those two
  // send you to opposite ends of the bench.
  enum class Error : uint8_t {
    None,
    ResetFailed,
    NoResponse,
    NoProductId,
    ReportEnableFailed
  };
  Error lastError() const { return lastError_; }

  // The raw Wire::endTransmission() code behind a NoResponse: 2 = nobody at
  // that address, 3 = there but rejected the data, 4 = bus fault, 5 = timeout.
  uint8_t lastWireError() const { return lastWireError_; }
  const char* lastErrorText() const;

  // Clock this driver applies in beginI2C(). Zero, the default, means "leave
  // whatever the caller configured alone" - which matters on a shared bus
  // whose slowest device sets the rate.
  void setBusClockHz(uint32_t hz) { busClockHz_ = hz; }
  bool interruptAsserted() const;
  void setAwake(bool awake);
  bool tareNow(bool zAxisOnly = false);
  bool saveTare();
  bool beginCalibration(bool accel = true, bool gyro = true,
                        bool mag = true, bool planarAccel = false,
                        bool onTable = false);
  bool endCalibration();
  bool calibrationComplete() const;
  bool saveCalibration();
  bool setPeriodicCalibrationSave(bool enable);
  uint8_t lastCommandStatus() const { return lastCommandStatus_; }
  bool wasReset();

  Quaternion quaternion() const { return quaternion_; }
  Quaternion gameQuaternion() const { return gameQuaternion_; }
  Quaternion geomagneticQuaternion() const { return geomagneticQuaternion_; }
  Vec3 linearAccelMs2() const { return linearAccelMs2_; }
  Vec3 gravityMs2() const { return gravityMs2_; }
  uint32_t stepCount() const { return stepCount_; }
  uint8_t stabilityClass() const { return stabilityClass_; }
  uint8_t activityClass() const { return activityClass_; }
  uint8_t activityConfidence(uint8_t activity) const {
    return activity < 9 ? activityConfidence_[activity] : 0;
  }
  uint8_t tapCode() const { return tapCode_; }
  uint8_t accelAccuracy() const { return accelAccuracy_; }
  uint8_t gyroAccuracy() const { return gyroAccuracy_; }
  uint8_t magAccuracy() const { return magAccuracy_; }
  const ProductInfo& productInfo() const { return product_; }

 private:
  static constexpr size_t kPacketCapacity = 128;

  bool softReset();
  bool requestProductId(uint16_t timeoutMs = 500);
  bool captureProductId();
  uint8_t drainQueue(uint16_t budgetMs);
  bool sendPacket(uint8_t channel, const uint8_t* payload, uint8_t length);
  bool receivePacket();
  bool readPayload(uint16_t length);
  bool parsePacket();
  bool parseSensorReport(const uint8_t* report, size_t length);
  bool sendTare(uint8_t subcommand, uint8_t axes);
  bool sendCommand(uint8_t command, const uint8_t* parameters = nullptr,
                   uint8_t parameterCount = 0, bool waitForResponse = true);
  bool waitForCommandResponse(uint8_t command, uint16_t timeoutMs = 500);
  static int16_t le16(const uint8_t* p);
  static uint32_t le32(const uint8_t* p);
  static float qToFloat(int16_t value, uint8_t qPoint);

  TwoWire* wire_ = nullptr;
  uint8_t address_ = bno08x::kAddrDefault;
  uint8_t txSequence_[6] = {0};
  uint8_t commandSequence_ = 0;
  uint8_t header_[4] = {0};
  uint8_t payload_[kPacketCapacity] = {0};
  uint16_t payloadLength_ = 0;
  bool productValid_ = false;
  bool resetSeen_ = false;
  uint32_t reportIntervalUs_ = 50000;

  ProductInfo product_;
  Quaternion quaternion_;
  Quaternion gameQuaternion_;
  Quaternion geomagneticQuaternion_;
  Vec3 linearAccelMs2_;
  Vec3 gravityMs2_;
  uint32_t stepCount_ = 0;
  uint8_t stabilityClass_ = 0;
  uint8_t activityClass_ = 0;
  uint8_t activityConfidence_[9] = {0};
  uint8_t tapCode_ = 0;
  uint8_t accelAccuracy_ = 0;
  uint8_t gyroAccuracy_ = 0;
  uint8_t magAccuracy_ = 0;
  uint8_t lastCommandStatus_ = 0xFF;
  Error lastError_ = Error::None;
  uint8_t lastWireError_ = 0;
  uint32_t busClockHz_ = 0;   // 0 = do not touch the caller's clock
  int8_t interruptPin_ = -1;
  int8_t resetPin_ = -1;
  int8_t wakePin_ = -1;
};

using BNO085 = BNO08x;
using BNO086 = BNO08x;

}  // namespace nimu

using nimu::BNO08x;
using nimu::BNO085;
using nimu::BNO086;
using nimu::IMUCalibration;
using nimu::IMUData;
using nimu::Vec3;

#endif
