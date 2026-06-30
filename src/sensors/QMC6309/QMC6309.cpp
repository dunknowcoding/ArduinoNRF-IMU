#include "QMC6309.h"

namespace nimu {
namespace {
constexpr uint8_t REG_WHO_AM_I = 0x00;
constexpr uint8_t REG_DATA = 0x01;
constexpr uint8_t REG_STATUS = 0x09;
constexpr uint8_t REG_CONTROL1 = 0x0A;
constexpr uint8_t REG_CONTROL2 = 0x0B;
constexpr uint8_t WHO_AM_I_VALUE = 0x90;
constexpr uint8_t STATUS_DRDY = 0x01;

int16_t le16(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}
}  // namespace

bool QMC6309::begin(uint8_t address, TwoWire& wire) {
  bus_.beginI2C(wire, address, 400000);
  if (!isConnected() || !reset()) return false;
  if (bus_.writeRegister(REG_CONTROL2, 0x48) != IMUStatus::Ok) return false;
  return bus_.writeRegister(REG_CONTROL1, 0x21) == IMUStatus::Ok;
}

uint8_t QMC6309::whoAmI() {
  uint8_t id = 0;
  bus_.readRegister(REG_WHO_AM_I, id);
  return id;
}

bool QMC6309::isConnected() { return whoAmI() == WHO_AM_I_VALUE; }

bool QMC6309::reset() {
  if (bus_.writeRegister(REG_CONTROL2, 0x80) != IMUStatus::Ok) return false;
  if (bus_.writeRegister(REG_CONTROL2, 0x00) != IMUStatus::Ok) return false;
  delay(10);
  return true;
}

bool QMC6309::dataReady() {
  uint8_t status = 0;
  return bus_.readRegister(REG_STATUS, status) == IMUStatus::Ok &&
         (status & STATUS_DRDY) != 0;
}

bool QMC6309::update() {
  uint8_t raw[6];
  if (bus_.readRegisters(REG_DATA, raw, sizeof(raw)) != IMUStatus::Ok) {
    return false;
  }
  // The MUMO/QMC6309 setup uses the +/-8 gauss, 4000 LSB/gauss range.
  constexpr float kMicroteslaPerLsb = 100.0f / 4000.0f;
  mag_ = Vec3{le16(&raw[0]) * kMicroteslaPerLsb,
              le16(&raw[2]) * kMicroteslaPerLsb,
              le16(&raw[4]) * kMicroteslaPerLsb};
  return true;
}

}  // namespace nimu
