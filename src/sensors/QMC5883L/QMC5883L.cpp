#include "QMC5883L.h"

namespace nimu {

namespace {
constexpr uint8_t REG_DATA = 0x00;
constexpr uint8_t REG_STATUS = 0x06;
constexpr uint8_t REG_CONTROL1 = 0x09;
constexpr uint8_t REG_CONTROL2 = 0x0A;
constexpr uint8_t REG_SET_RESET = 0x0B;
constexpr uint8_t STATUS_DRDY = 0x01;

inline int16_t le16(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}
}  // namespace

bool QMC5883L::begin(uint8_t address, TwoWire& wire) {
  bus_.beginI2C(wire, address, 400000);
  if (!isConnected()) {
    return false;
  }
  bus_.writeRegister(REG_CONTROL2, 0x80);  // soft reset
  delay(10);
  if (bus_.writeRegister(REG_SET_RESET, 0x01) != IMUStatus::Ok) {
    return false;
  }
  // OSR=512, range=2G, ODR=50Hz, continuous mode.
  return bus_.writeRegister(REG_CONTROL1, 0x1D) == IMUStatus::Ok;
}

bool QMC5883L::isConnected() {
  return bus_.ping() == IMUStatus::Ok;
}

bool QMC5883L::update() {
  uint8_t status = 0;
  bus_.readRegister(REG_STATUS, status);
  if ((status & STATUS_DRDY) == 0) {
    delay(20);
  }
  uint8_t b[6];
  if (bus_.readRegisters(REG_DATA, b, sizeof(b)) != IMUStatus::Ok) {
    return false;
  }
  // In +/-2G mode most QMC5883L examples use about 12000 LSB/Gauss.
  constexpr float scale = 100.0f / 12000.0f;
  mag_ = Vec3{le16(&b[0]) * scale, le16(&b[2]) * scale, le16(&b[4]) * scale};
  return true;
}

bool QMC5883L::dataReady() {
  uint8_t status = 0;
  return bus_.readRegister(REG_STATUS, status) == IMUStatus::Ok &&
         (status & STATUS_DRDY) != 0;
}

bool QMC5883L::dataOverflow() {
  uint8_t status = 0;
  return bus_.readRegister(REG_STATUS, status) == IMUStatus::Ok &&
         (status & 0x02) != 0;
}

}  // namespace nimu
