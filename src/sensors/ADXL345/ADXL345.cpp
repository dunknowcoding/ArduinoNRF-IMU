#include "ADXL345.h"

namespace nimu {
using namespace adxl345;

namespace {
inline int16_t le16(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}
}  // namespace

bool ADXL345::begin() {
  if (beginI2C(Wire, kAddrSDOLow)) {
    return true;
  }
  return beginI2C(Wire, kAddrSDOHigh);
}

bool ADXL345::beginI2C(TwoWire& wire, uint8_t address) {
  bus_.beginI2C(wire, address, clockHz_);
  bus_.recoverBus();
  if (!isConnected()) {
    return false;
  }
  return configureDefaults();
}

bool ADXL345::beginSPI(SPIClass& spi, uint8_t csPin) {
  bus_.beginSPI(spi, csPin, 1000000, 0xC0);
  if (!isConnected()) {
    return false;
  }
  return configureDefaults();
}

uint8_t ADXL345::whoAmI() {
  uint8_t id = 0;
  bus_.readRegister(DEVID, id);
  return id;
}

bool ADXL345::isConnected() {
  return whoAmI() == kDeviceId;
}

bool ADXL345::configureDefaults() {
  bool ok = true;
  ok &= setAccelRangeG(accelRangeG_);
  ok &= setSampleRateHz(sampleRateHz_);
  ok &= bus_.writeRegister(POWER_CTL, POWER_MEASURE) == IMUStatus::Ok;
  return ok;
}

bool ADXL345::readRaw(RawSample& out) {
  uint8_t b[6];
  if (bus_.readRegisters(DATAX0, b, sizeof(b)) != IMUStatus::Ok) {
    return false;
  }
  out.ax = le16(&b[0]);
  out.ay = le16(&b[2]);
  out.az = le16(&b[4]);
  return true;
}

bool ADXL345::update() {
  RawSample raw;
  if (!readRaw(raw)) {
    return false;
  }
  Vec3 a{raw.ax / accelLsbPerG_, raw.ay / accelLsbPerG_,
         raw.az / accelLsbPerG_};
  data_.accel = correct(a, cal_.accelBias, cal_.accelScale);
  data_.gyro = Vec3{0, 0, 0};
  data_.mag = Vec3{0, 0, 0};
  data_.temperature = 0.0f;
  data_.timestamp = micros();
  return true;
}

bool ADXL345::setAccelRangeG(uint16_t maxG) {
  uint8_t range;
  if (maxG <= 2) {
    range = 0x00; accelRangeG_ = 2;
  } else if (maxG <= 4) {
    range = 0x01; accelRangeG_ = 4;
  } else if (maxG <= 8) {
    range = 0x02; accelRangeG_ = 8;
  } else {
    range = 0x03; accelRangeG_ = 16;
  }
  return bus_.writeRegister(DATA_FORMAT, DATA_FULL_RES | range) == IMUStatus::Ok;
}

bool ADXL345::setGyroRangeDps(uint16_t maxDps) {
  (void)maxDps;
  return true;
}

uint8_t ADXL345::odrCodeForHz(uint16_t hz, uint16_t& actualHz) const {
  if (hz <= 25) { actualHz = 25; return 0x08; }
  if (hz <= 50) { actualHz = 50; return 0x09; }
  if (hz <= 100) { actualHz = 100; return 0x0A; }
  if (hz <= 200) { actualHz = 200; return 0x0B; }
  if (hz <= 400) { actualHz = 400; return 0x0C; }
  if (hz <= 800) { actualHz = 800; return 0x0D; }
  actualHz = 1600;
  return 0x0E;
}

bool ADXL345::setSampleRateHz(uint16_t hz) {
  if (hz == 0) {
    return false;
  }
  uint16_t actual = 0;
  uint8_t odr = odrCodeForHz(hz, actual);
  sampleRateHz_ = actual;
  return bus_.writeRegister(BW_RATE, odr) == IMUStatus::Ok;
}

bool ADXL345::setLowPassFilterHz(uint16_t hz) {
  return setSampleRateHz(hz * 2);
}

bool ADXL345::dataReady() {
  uint8_t source = 0;
  if (bus_.readRegister(INT_SOURCE, source) != IMUStatus::Ok) {
    return false;
  }
  return (source & INT_DATA_READY) != 0;
}

bool ADXL345::configureInterruptPolarity(bool activeLow) {
  return bus_.updateRegister(DATA_FORMAT, DATA_INT_INVERT,
                             activeLow ? DATA_INT_INVERT : 0) == IMUStatus::Ok;
}

bool ADXL345::routeInterrupt(uint8_t sources, uint8_t pin, bool enable) {
  if (pin != 1 && pin != 2) return false;
  uint8_t mapped = pin == 2 ? sources : 0;
  bool ok = bus_.updateRegister(INT_MAP, sources, mapped) == IMUStatus::Ok;
  ok &= bus_.updateRegister(INT_ENABLE, sources, enable ? sources : 0) ==
        IMUStatus::Ok;
  return ok;
}

uint8_t ADXL345::interruptSource() {
  uint8_t source = 0;
  bus_.readRegister(INT_SOURCE, source);
  return source;
}

void ADXL345::setBusClockHz(uint32_t hz) {
  clockHz_ = hz;
  bus_.setClockHz(hz);
}

}  // namespace nimu
