#include "LSM303D.h"

namespace nimu {
namespace {
constexpr uint8_t WHO_AM_I = 0x0F;
constexpr uint8_t TEMP_OUT_L = 0x05;
constexpr uint8_t STATUS_M = 0x07;
constexpr uint8_t OUT_X_L_M = 0x08;
constexpr uint8_t CTRL1 = 0x20;
constexpr uint8_t CTRL2 = 0x21;
constexpr uint8_t CTRL5 = 0x24;
constexpr uint8_t CTRL6 = 0x25;
constexpr uint8_t CTRL7 = 0x26;
constexpr uint8_t STATUS_A = 0x27;
constexpr uint8_t OUT_X_L_A = 0x28;
constexpr uint8_t STATUS_ZYXDA = 0x08;

int16_t le16(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}
}

bool LSM303D::begin() {
  if (beginI2C(Wire, kAddrLow)) return true;
  return beginI2C(Wire, kAddrHigh);
}

bool LSM303D::beginI2C(TwoWire& wire, uint8_t address) {
  bus_.beginI2C(wire, address, 400000);
  bus_.recoverBus();
  return isConnected() && configureDefaults();
}

bool LSM303D::beginSPI(SPIClass& spi, uint8_t csPin) {
  bus_.beginSPI(spi, csPin, 1000000, 0xC0);
  return isConnected() && configureDefaults();
}

uint8_t LSM303D::whoAmI() {
  uint8_t id = 0;
  bus_.readRegister(WHO_AM_I, id);
  return id;
}
bool LSM303D::isConnected() { return whoAmI() == kWhoAmI; }

bool LSM303D::configureDefaults() {
  bool ok = bus_.writeRegister(CTRL1, 0x67) == IMUStatus::Ok;
  ok &= bus_.writeRegister(CTRL2, 0x08) == IMUStatus::Ok;
  ok &= bus_.writeRegister(CTRL5, 0xF4) == IMUStatus::Ok;
  ok &= bus_.writeRegister(CTRL6, 0x20) == IMUStatus::Ok;
  ok &= bus_.writeRegister(CTRL7, 0x00) == IMUStatus::Ok;
  return ok;
}

bool LSM303D::readRaw(RawSample& out) {
  uint8_t a[6], m[6], t[2];
  if (bus_.readRegisters(OUT_X_L_A, a, 6) != IMUStatus::Ok ||
      bus_.readRegisters(OUT_X_L_M, m, 6) != IMUStatus::Ok ||
      bus_.readRegisters(TEMP_OUT_L, t, 2) != IMUStatus::Ok) return false;
  out.ax = le16(&a[0]); out.ay = le16(&a[2]); out.az = le16(&a[4]);
  out.mx = le16(&m[0]); out.my = le16(&m[2]); out.mz = le16(&m[4]);
  out.temp = static_cast<int16_t>(le16(t) >> 4);
  return true;
}

bool LSM303D::update() {
  RawSample raw;
  if (!readRaw(raw)) return false;
  data_.accel = correct(Vec3{raw.ax * accelGPerLsb_, raw.ay * accelGPerLsb_,
                              raw.az * accelGPerLsb_},
                        cal_.accelBias, cal_.accelScale);
  data_.gyro = Vec3{};
  data_.mag = correct(Vec3{raw.mx * magUtPerLsb_, raw.my * magUtPerLsb_,
                            raw.mz * magUtPerLsb_},
                      cal_.magBias, cal_.magScale);
  data_.temperature = 25.0f + raw.temp / 8.0f;
  data_.timestamp = micros();
  return true;
}

bool LSM303D::setAccelRangeG(uint16_t maxG) {
  if (maxG <= 2) { accelRangeCode_ = 0; accelGPerLsb_ = 0.000061f; }
  else if (maxG <= 4) { accelRangeCode_ = 1; accelGPerLsb_ = 0.000122f; }
  else if (maxG <= 6) { accelRangeCode_ = 2; accelGPerLsb_ = 0.000183f; }
  else if (maxG <= 8) { accelRangeCode_ = 3; accelGPerLsb_ = 0.000244f; }
  else { accelRangeCode_ = 4; accelGPerLsb_ = 0.000732f; }
  return bus_.updateRegister(CTRL2, 0x38, accelRangeCode_ << 3) == IMUStatus::Ok;
}

bool LSM303D::setGyroRangeDps(uint16_t maxDps) { (void)maxDps; return false; }
bool LSM303D::setLowPassFilterHz(uint16_t hz) { (void)hz; return true; }

bool LSM303D::setSampleRateHz(uint16_t hz) {
  if (hz == 0) return false;
  if (hz <= 4) accelOdrCode_ = 1;
  else if (hz <= 7) accelOdrCode_ = 2;
  else if (hz <= 13) accelOdrCode_ = 3;
  else if (hz <= 25) accelOdrCode_ = 4;
  else if (hz <= 50) accelOdrCode_ = 5;
  else if (hz <= 100) accelOdrCode_ = 6;
  else if (hz <= 200) accelOdrCode_ = 7;
  else if (hz <= 400) accelOdrCode_ = 8;
  else if (hz <= 800) accelOdrCode_ = 9;
  else accelOdrCode_ = 10;
  return bus_.updateRegister(CTRL1, 0xF0, accelOdrCode_ << 4) == IMUStatus::Ok;
}

bool LSM303D::setMagRangeGauss(uint16_t gauss) {
  uint8_t code;
  if (gauss <= 2) { code = 0; magUtPerLsb_ = 0.008f; }
  else if (gauss <= 4) { code = 1; magUtPerLsb_ = 0.016f; }
  else if (gauss <= 8) { code = 2; magUtPerLsb_ = 0.032f; }
  else { code = 3; magUtPerLsb_ = 0.0479f; }
  return bus_.updateRegister(CTRL6, 0x60, code << 5) == IMUStatus::Ok;
}

bool LSM303D::dataReady() {
  uint8_t status = 0;
  return bus_.readRegister(STATUS_A, status) == IMUStatus::Ok &&
         (status & STATUS_ZYXDA) != 0;
}
bool LSM303D::magDataReady() {
  uint8_t status = 0;
  return bus_.readRegister(STATUS_M, status) == IMUStatus::Ok &&
         (status & STATUS_ZYXDA) != 0;
}

}  // namespace nimu
