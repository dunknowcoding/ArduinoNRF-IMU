#include "BMP180.h"

#if defined(__AVR__)
#include <math.h>  // avr-libc provides powf; the forward-decl below conflicts on AVR
#else
extern "C" float powf(float, float);
#endif

namespace nimu {
using namespace bmp180;

namespace {
inline uint16_t u16be(const uint8_t* p) {
  return (static_cast<uint16_t>(p[0]) << 8) | p[1];
}
inline int16_t s16be(const uint8_t* p) {
  return static_cast<int16_t>(u16be(p));
}
}  // namespace

bool BMP180::begin(uint8_t address, TwoWire& wire) {
  bus_.beginI2C(wire, address, 400000);
  if (!isConnected()) {
    return false;
  }
  return readCalibration();
}

uint8_t BMP180::chipId() {
  uint8_t id = 0;
  bus_.readRegister(REG_ID, id);
  return id;
}

bool BMP180::isConnected() { return chipId() == kChipId; }

bool BMP180::readCalibration() {
  uint8_t b[22];
  if (bus_.readRegisters(REG_CALIB, b, sizeof(b)) != IMUStatus::Ok) {
    return false;
  }
  ac1_ = s16be(&b[0]);
  ac2_ = s16be(&b[2]);
  ac3_ = s16be(&b[4]);
  ac4_ = u16be(&b[6]);
  ac5_ = u16be(&b[8]);
  ac6_ = u16be(&b[10]);
  b1_ = s16be(&b[12]);
  b2_ = s16be(&b[14]);
  mb_ = s16be(&b[16]);
  mc_ = s16be(&b[18]);
  md_ = s16be(&b[20]);
  return ac4_ != 0 && ac5_ != 0 && ac6_ != 0;
}

bool BMP180::readRawTemperature(int32_t& out) {
  if (bus_.writeRegister(REG_CONTROL, CMD_TEMP) != IMUStatus::Ok) {
    return false;
  }
  delay(5);
  uint8_t b[2];
  if (bus_.readRegisters(REG_DATA, b, sizeof(b)) != IMUStatus::Ok) {
    return false;
  }
  out = static_cast<int32_t>(u16be(b));
  return true;
}

bool BMP180::readRawPressure(int32_t& out) {
  uint8_t cmd = CMD_PRESSURE + (static_cast<uint8_t>(oss_) << 6);
  if (bus_.writeRegister(REG_CONTROL, cmd) != IMUStatus::Ok) {
    return false;
  }
  switch (oss_) {
    case OSS_ULTRA_LOW_POWER: delay(5); break;
    case OSS_STANDARD: delay(8); break;
    case OSS_HIGH_RES: delay(14); break;
    default: delay(26); break;
  }
  uint8_t b[3];
  if (bus_.readRegisters(REG_DATA, b, sizeof(b)) != IMUStatus::Ok) {
    return false;
  }
  out = (((static_cast<int32_t>(b[0]) << 16) |
          (static_cast<int32_t>(b[1]) << 8) | b[2]) >>
         (8 - static_cast<uint8_t>(oss_)));
  return true;
}

bool BMP180::update() {
  int32_t ut = 0;
  int32_t up = 0;
  if (!readRawTemperature(ut) || !readRawPressure(up)) {
    return false;
  }

  int32_t x1 = ((ut - static_cast<int32_t>(ac6_)) * static_cast<int32_t>(ac5_)) >> 15;
  int32_t x2 = (static_cast<int32_t>(mc_) << 11) / (x1 + md_);
  b5_ = x1 + x2;
  int32_t t = (b5_ + 8) >> 4;
  tempC_ = t * 0.1f;

  int32_t b6 = b5_ - 4000;
  x1 = (static_cast<int32_t>(b2_) * ((b6 * b6) >> 12)) >> 11;
  x2 = (static_cast<int32_t>(ac2_) * b6) >> 11;
  int32_t x3 = x1 + x2;
  int32_t b3 = (((static_cast<int32_t>(ac1_) * 4 + x3) <<
                 static_cast<uint8_t>(oss_)) + 2) >> 2;
  x1 = (static_cast<int32_t>(ac3_) * b6) >> 13;
  x2 = (static_cast<int32_t>(b1_) * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  uint32_t b4 = (static_cast<uint32_t>(ac4_) *
                 static_cast<uint32_t>(x3 + 32768)) >> 15;
  uint32_t b7 = (static_cast<uint32_t>(up - b3) *
                 static_cast<uint32_t>(50000UL >> static_cast<uint8_t>(oss_)));

  int32_t p = (b7 < 0x80000000UL) ? static_cast<int32_t>((b7 * 2) / b4)
                                  : static_cast<int32_t>((b7 / b4) * 2);
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  p = p + ((x1 + x2 + 3791) >> 4);
  pressPa_ = static_cast<float>(p);
  return true;
}

float BMP180::altitudeM() const {
  float ratio = (pressPa_ * 0.01f) / seaLevelHpa_;
  return 44330.0f * (1.0f - powf(ratio, 0.1902949572f));
}

bool BMP180::calibrateAltitude(float knownAltitudeM) {
  if (!update()) {
    return false;
  }
  float base = 1.0f - (knownAltitudeM / 44330.0f);
  if (base <= 0.0f) {
    return false;
  }
  seaLevelHpa_ = (pressPa_ * 0.01f) / powf(base, 5.2553f);
  return true;
}

}  // namespace nimu
