/* GY601N1.h - Shared facade for GY-601N1 module populations. */
#ifndef ARDUINONRF_IMU_GY601N1_H
#define ARDUINONRF_IMU_GY601N1_H

#include "../../sensors/BMI323/BMI323.h"
#include "../../sensors/ICM42688P/ICM42688P.h"
#include "../../sensors/ICM45686/ICM45686.h"

namespace nimu {

/*
  All observed populations use the same silk.

  Raw side:
    VCC, GND, SCL/SCLK, SDA/SDI, SA0/SDO, CS, INT1, INT2

  MCU side:
    VCC, GND, RX/CL, TX/DA, INT, PS
*/
class GY601N1 {
 public:
  enum class Core : uint8_t {
    None,
    ICM42688,
    ICM45686,
    BMI323,
  };

  bool begin();
  bool beginI2C(TwoWire& wire, uint8_t address);
  bool beginSPI(SPIClass& spi, uint8_t csPin);

  Core core() const { return core_; }
  const char* coreName() const;
  uint8_t whoAmI();
  bool isConnected();
  bool update();
  bool dataReady();

  const IMUData& data() const;
  Vec3 accelG() const { return data().accel; }
  Vec3 accelMs2() const;
  Vec3 gyroDps() const { return data().gyro; }
  Vec3 gyroRps() const;
  float temperatureC() const { return data().temperature; }
  uint32_t timestamp() const { return data().timestamp; }

  bool setAccelRangeG(uint16_t maxG);
  bool setGyroRangeDps(uint16_t maxDps);
  bool setLowPassFilterHz(uint16_t hz);
  bool setSampleRateHz(uint16_t hz);
  bool configureInterruptPin(uint8_t pin, bool activeHigh,
                             bool openDrain, bool latched = false);
  bool routeDataReadyInterrupt(uint8_t pin, bool enable = true);

  bool calibrateGyro(uint16_t samples = 200);
  bool calibrateAccel(uint16_t samples = 200);
  IMUCalibration getCalibration() const;
  void setCalibration(const IMUCalibration& calibration);
  void clearCalibration();

  ICM42688P* icm42688();
  ICM45686* icm45686();
  BMI323* bmi323();

  void beginUART(Stream& serial) { uart_ = &serial; }
  int uartAvailable() const { return uart_ == nullptr ? 0 : uart_->available(); }
  int readUART() { return uart_ == nullptr ? -1 : uart_->read(); }
  size_t readUART(uint8_t* data, size_t length);

  static void driveProtocolSelect(uint8_t psPin, bool high) {
    pinMode(psPin, OUTPUT);
    digitalWrite(psPin, high ? HIGH : LOW);
  }

 private:
  IMUSensor* active();
  const IMUSensor* active() const;
  void select(Core core);

  Core core_ = Core::None;
  ICM42688P icm42688_;
  ICM45686 icm45686_;
  BMI323 bmi323_;
  IMUData emptyData_;
  Stream* uart_ = nullptr;
};

}  // namespace nimu

using nimu::GY601N1;

#endif
