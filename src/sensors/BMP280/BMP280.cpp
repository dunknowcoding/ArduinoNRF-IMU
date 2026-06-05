#include "BMP280.h"

// Just one libm call (altitude uses a fractional power). Pulling in <math.h>
// here would drag in the C++ <limits>/<algorithm> headers, which clash with
// Arduino's min()/max() macros in sketches, so forward-declare the one symbol.
extern "C" float powf(float, float);

namespace nimu {
using namespace bmp280;

namespace {
inline uint16_t u16le(const uint8_t* p) {
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}
inline int16_t s16le(const uint8_t* p) {
  return static_cast<int16_t>(u16le(p));
}
}  // namespace

// ---------------------------------------------------------------- bring-up --

bool BMP280::begin(uint8_t address, TwoWire& wire) {
  bus_.beginI2C(wire, address, 400000);
  if (!isConnected()) {
    return false;
  }
  return reset() && readCalibration() && applyDefaultConfig();
}

bool BMP280::beginSPI(SPIClass& spi, uint8_t csPin) {
  // BMP280 SPI control byte: bit7 = 1 for read, 0 for write (same convention as
  // the IMUBus default read flag).
  bus_.beginSPI(spi, csPin, 1000000, 0x80);
  if (!isConnected()) {
    return false;
  }
  return reset() && readCalibration() && applyDefaultConfig();
}

uint8_t BMP280::chipId() {
  uint8_t id = 0;
  bus_.readRegister(REG_ID, id);
  return id;
}

bool BMP280::isConnected() { return chipId() == kChipId; }

bool BMP280::reset() {
  if (bus_.writeRegister(REG_RESET, RESET_WORD) != IMUStatus::Ok) {
    return false;
  }
  delay(10);  // power-on reset sequence; trim becomes readable after ~2 ms
  return true;
}

bool BMP280::readCalibration() {
  uint8_t b[24];
  if (bus_.readRegisters(REG_CALIB00, b, sizeof(b)) != IMUStatus::Ok) {
    return false;
  }
  digT1_ = u16le(&b[0]);
  digT2_ = s16le(&b[2]);
  digT3_ = s16le(&b[4]);
  digP1_ = u16le(&b[6]);
  digP2_ = s16le(&b[8]);
  digP3_ = s16le(&b[10]);
  digP4_ = s16le(&b[12]);
  digP5_ = s16le(&b[14]);
  digP6_ = s16le(&b[16]);
  digP7_ = s16le(&b[18]);
  digP8_ = s16le(&b[20]);
  digP9_ = s16le(&b[22]);
  return digT1_ != 0 && digP1_ != 0;  // all-zero trim => read failed
}

bool BMP280::applyDefaultConfig() {
  // Weather/indoor default: pressure x16, temp x2, IIR filter 16, normal mode.
  return setSampling(OSRS_X16, OSRS_X2, FILTER_16, MODE_NORMAL, STANDBY_0_5MS);
}

bool BMP280::setSampling(uint8_t pressureOsrs, uint8_t tempOsrs, uint8_t filter,
                         uint8_t mode, uint8_t standby) {
  uint8_t config = (standby << 5) | (filter << 2);
  if (bus_.writeRegister(REG_CONFIG, config) != IMUStatus::Ok) {
    return false;
  }
  uint8_t ctrl = (tempOsrs << 5) | (pressureOsrs << 2) | mode;
  if (bus_.writeRegister(REG_CTRL_MEAS, ctrl) != IMUStatus::Ok) {
    return false;
  }
  delay(10);
  return true;
}

// -------------------------------------------------------------- data read ---

bool BMP280::update() {
  uint8_t b[6];
  if (bus_.readRegisters(REG_PRESS_MSB, b, sizeof(b)) != IMUStatus::Ok) {
    return false;
  }
  int32_t adcP = (static_cast<int32_t>(b[0]) << 12) |
                 (static_cast<int32_t>(b[1]) << 4) | (b[2] >> 4);
  int32_t adcT = (static_cast<int32_t>(b[3]) << 12) |
                 (static_cast<int32_t>(b[4]) << 4) | (b[5] >> 4);

  // --- temperature compensation (datasheet, S32) -> sets tFine_, 0.01 degC ---
  int32_t var1 = ((((adcT >> 3) - (static_cast<int32_t>(digT1_) << 1))) *
                  static_cast<int32_t>(digT2_)) >> 11;
  int32_t var2 = (((((adcT >> 4) - static_cast<int32_t>(digT1_)) *
                    ((adcT >> 4) - static_cast<int32_t>(digT1_))) >> 12) *
                  static_cast<int32_t>(digT3_)) >> 14;
  tFine_ = var1 + var2;
  int32_t tHundredths = (tFine_ * 5 + 128) >> 8;
  tempC_ = tHundredths / 100.0f;

  // --- pressure compensation (datasheet, S64) -> Q24.8 Pa ---
  int64_t p1 = static_cast<int64_t>(tFine_) - 128000;
  int64_t p2 = p1 * p1 * static_cast<int64_t>(digP6_);
  p2 = p2 + ((p1 * static_cast<int64_t>(digP5_)) << 17);
  p2 = p2 + (static_cast<int64_t>(digP4_) << 35);
  p1 = ((p1 * p1 * static_cast<int64_t>(digP3_)) >> 8) +
       ((p1 * static_cast<int64_t>(digP2_)) << 12);
  p1 = (((static_cast<int64_t>(1) << 47) + p1) * static_cast<int64_t>(digP1_)) >> 33;
  if (p1 == 0) {
    pressPa_ = 0.0f;  // avoid divide-by-zero on a failed trim
    return true;
  }
  int64_t p = 1048576 - adcP;
  p = (((p << 31) - p2) * 3125) / p1;
  p1 = (static_cast<int64_t>(digP9_) * (p >> 13) * (p >> 13)) >> 25;
  p2 = (static_cast<int64_t>(digP8_) * p) >> 19;
  p = ((p + p1 + p2) >> 8) + (static_cast<int64_t>(digP7_) << 4);
  pressPa_ = static_cast<float>(p) / 256.0f;
  return true;
}

float BMP280::altitudeM() const {
  // International barometric formula: h = 44330 * (1 - (P/P0)^(1/5.255)).
  float ratio = (pressPa_ * 0.01f) / seaLevelHpa_;
  return 44330.0f * (1.0f - powf(ratio, 0.1902949572f));
}

bool BMP280::calibrateAltitude(float knownAltitudeM) {
  if (!update()) {
    return false;
  }
  // Invert the barometric formula to find the sea-level pressure that puts the
  // current reading at knownAltitudeM: P0 = P / (1 - h/44330)^5.255.
  float base = 1.0f - (knownAltitudeM / 44330.0f);
  if (base <= 0.0f) {
    return false;
  }
  seaLevelHpa_ = (pressPa_ * 0.01f) / powf(base, 5.2553f);
  return true;
}

}  // namespace nimu
