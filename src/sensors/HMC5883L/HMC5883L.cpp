#include "HMC5883L.h"

namespace nimu {

namespace {
constexpr uint8_t REG_CONFIG_A = 0x00;
constexpr uint8_t REG_CONFIG_B = 0x01;
constexpr uint8_t REG_MODE = 0x02;
constexpr uint8_t REG_DATA = 0x03;
constexpr uint8_t REG_STATUS = 0x09;
constexpr uint8_t REG_ID_A = 0x0A;

inline int16_t be16(const uint8_t* p) {
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}
}  // namespace

bool HMC5883L::begin(uint8_t address, TwoWire& wire) {
  bus_.beginI2C(wire, address, 400000);
  if (!isConnected()) {
    return false;
  }
  // 8-sample average, 15 Hz output rate, normal measurement.
  if (bus_.writeRegister(REG_CONFIG_A, 0x70) != IMUStatus::Ok) {
    return false;
  }
  if (!setGainGauss(1.3f)) {
    return false;
  }
  return setContinuousMode();
}

bool HMC5883L::isConnected() {
  uint8_t id[3] = {};
  if (bus_.readRegisters(REG_ID_A, id, sizeof(id)) != IMUStatus::Ok) {
    return false;
  }
  return id[0] == 'H' && id[1] == '4' && id[2] == '3';
}

bool HMC5883L::setGainGauss(float gauss) {
  uint8_t gain = 0x20;
  lsbPerGauss_ = 1090.0f;
  if (gauss <= 0.88f) {
    gain = 0x00; lsbPerGauss_ = 1370.0f;
  } else if (gauss <= 1.3f) {
    gain = 0x20; lsbPerGauss_ = 1090.0f;
  } else if (gauss <= 1.9f) {
    gain = 0x40; lsbPerGauss_ = 820.0f;
  } else if (gauss <= 2.5f) {
    gain = 0x60; lsbPerGauss_ = 660.0f;
  } else if (gauss <= 4.0f) {
    gain = 0x80; lsbPerGauss_ = 440.0f;
  } else if (gauss <= 4.7f) {
    gain = 0xA0; lsbPerGauss_ = 390.0f;
  } else if (gauss <= 5.6f) {
    gain = 0xC0; lsbPerGauss_ = 330.0f;
  } else {
    gain = 0xE0; lsbPerGauss_ = 230.0f;
  }
  return bus_.writeRegister(REG_CONFIG_B, gain) == IMUStatus::Ok;
}

bool HMC5883L::setContinuousMode() {
  return bus_.writeRegister(REG_MODE, 0x00) == IMUStatus::Ok;
}

bool HMC5883L::update() {
  uint8_t b[6];
  if (bus_.readRegisters(REG_DATA, b, sizeof(b)) != IMUStatus::Ok) {
    return false;
  }
  int16_t x = be16(&b[0]);
  int16_t z = be16(&b[2]);
  int16_t y = be16(&b[4]);
  float scale = 100.0f / lsbPerGauss_;
  mag_ = Vec3{x * scale, y * scale, z * scale};
  return true;
}

bool HMC5883L::dataReady() {
  uint8_t status = 0;
  return bus_.readRegister(REG_STATUS, status) == IMUStatus::Ok &&
         (status & 0x01) != 0;
}

bool HMC5883L::dataLocked() {
  uint8_t status = 0;
  return bus_.readRegister(REG_STATUS, status) == IMUStatus::Ok &&
         (status & 0x02) != 0;
}

}  // namespace nimu
