#include "MMA8452Q.h"

namespace nimu {
using namespace mma8452q;

namespace {
inline int16_t be12(const uint8_t* p) {
  int16_t v = static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
  return static_cast<int16_t>(v >> 4);
}
}  // namespace

bool MMA8452Q::begin() {
  if (beginI2C(Wire, kAddrSA0High)) {
    return true;
  }
  return beginI2C(Wire, kAddrSA0Low);
}

bool MMA8452Q::beginI2C(TwoWire& wire, uint8_t address) {
  bus_.beginI2C(wire, address, clockHz_);
  bus_.recoverBus();
  if (!isConnected()) {
    return false;
  }
  return configureDefaults();
}

bool MMA8452Q::beginSPI(SPIClass& spi, uint8_t csPin) {
  (void)spi;
  (void)csPin;
  return false;
}

uint8_t MMA8452Q::whoAmI() {
  uint8_t id = 0;
  bus_.readRegister(WHO_AM_I, id);
  return id;
}

bool MMA8452Q::isConnected() {
  return whoAmI() == kWhoAmI;
}

bool MMA8452Q::configureDefaults() {
  bool ok = true;
  ok &= setStandby(true);
  ok &= setAccelRangeG(accelRangeG_);
  ok &= setSampleRateHz(sampleRateHz_);
  ok &= setStandby(false);
  return ok;
}

bool MMA8452Q::setStandby(bool standby) {
  uint8_t value = 0;
  if (bus_.readRegister(CTRL_REG1, value) != IMUStatus::Ok) {
    return false;
  }
  if (standby) {
    value &= ~CTRL_ACTIVE;
  } else {
    value |= CTRL_ACTIVE;
  }
  return bus_.writeRegister(CTRL_REG1, value) == IMUStatus::Ok;
}

bool MMA8452Q::readRaw(RawSample& out) {
  uint8_t b[6];
  if (bus_.readRegisters(OUT_X_MSB, b, sizeof(b)) != IMUStatus::Ok) {
    return false;
  }
  out.ax = be12(&b[0]);
  out.ay = be12(&b[2]);
  out.az = be12(&b[4]);
  return true;
}

bool MMA8452Q::update() {
  RawSample raw;
  if (!readRaw(raw)) {
    return false;
  }

  Vec3 a{raw.ax / accelLsbPerG_, raw.ay / accelLsbPerG_,
         raw.az / accelLsbPerG_};
  data_.accel = correct(a, cal_.accelBias, cal_.accelScale);
  data_.gyro = Vec3{};
  data_.mag = Vec3{};
  data_.temperature = 0.0f;
  data_.timestamp = micros();
  return true;
}

bool MMA8452Q::setAccelRangeG(uint16_t maxG) {
  uint8_t fs;
  if (maxG <= 2) {
    fs = 0x00;
    accelLsbPerG_ = 1024.0f;
    accelRangeG_ = 2;
  } else if (maxG <= 4) {
    fs = 0x01;
    accelLsbPerG_ = 512.0f;
    accelRangeG_ = 4;
  } else {
    fs = 0x02;
    accelLsbPerG_ = 256.0f;
    accelRangeG_ = 8;
  }

  bool active = false;
  uint8_t ctrl = 0;
  if (bus_.readRegister(CTRL_REG1, ctrl) == IMUStatus::Ok) {
    active = (ctrl & CTRL_ACTIVE) != 0;
  }
  if (active && !setStandby(true)) {
    return false;
  }
  bool ok = bus_.updateRegister(XYZ_DATA_CFG, FS_MASK, fs) == IMUStatus::Ok;
  if (active) {
    ok &= setStandby(false);
  }
  return ok;
}

bool MMA8452Q::setGyroRangeDps(uint16_t maxDps) {
  (void)maxDps;
  return true;
}

uint8_t MMA8452Q::odrBitsForHz(uint16_t hz, uint16_t& actualHz) const {
  if (hz <= 2) { actualHz = 2; return 0x38; }
  if (hz <= 6) { actualHz = 6; return 0x30; }
  if (hz <= 13) { actualHz = 12; return 0x28; }
  if (hz <= 25) { actualHz = 25; return 0x20; }
  if (hz <= 50) { actualHz = 50; return 0x18; }
  if (hz <= 100) { actualHz = 100; return 0x10; }
  if (hz <= 200) { actualHz = 200; return 0x08; }
  actualHz = 800;
  return 0x00;
}

bool MMA8452Q::setSampleRateHz(uint16_t hz) {
  if (hz == 0) {
    return false;
  }
  uint16_t actual = 0;
  uint8_t odr = odrBitsForHz(hz, actual);
  sampleRateHz_ = actual;

  bool active = false;
  uint8_t ctrl = 0;
  if (bus_.readRegister(CTRL_REG1, ctrl) == IMUStatus::Ok) {
    active = (ctrl & CTRL_ACTIVE) != 0;
  }
  if (active && !setStandby(true)) {
    return false;
  }
  bool ok = bus_.updateRegister(CTRL_REG1, CTRL_ODR_MASK, odr) == IMUStatus::Ok;
  if (active) {
    ok &= setStandby(false);
  }
  return ok;
}

bool MMA8452Q::setLowPassFilterHz(uint16_t hz) {
  return setSampleRateHz(hz * 2);
}

bool MMA8452Q::dataReady() {
  uint8_t status = 0;
  if (bus_.readRegister(STATUS, status) != IMUStatus::Ok) {
    return false;
  }
  return (status & STATUS_ZYXDR) != 0;
}

bool MMA8452Q::configureInterruptPins(bool activeHigh, bool openDrain) {
  uint8_t value = (activeHigh ? INT_ACTIVE_HIGH : 0) |
                  (openDrain ? INT_OPEN_DRAIN : 0);
  return bus_.updateRegister(CTRL_REG3, INT_ACTIVE_HIGH | INT_OPEN_DRAIN,
                             value) == IMUStatus::Ok;
}

bool MMA8452Q::routeInterrupt(uint8_t sources, uint8_t pin, bool enable) {
  if (pin != 1 && pin != 2) return false;
  if (!setStandby(true)) return false;
  bool ok = bus_.updateRegister(CTRL_REG5, sources,
                                pin == 1 ? sources : 0) == IMUStatus::Ok;
  ok &= bus_.updateRegister(CTRL_REG4, sources, enable ? sources : 0) ==
        IMUStatus::Ok;
  return setStandby(false) && ok;
}

void MMA8452Q::setBusClockHz(uint32_t hz) {
  clockHz_ = hz;
  bus_.setClockHz(hz);
}

}  // namespace nimu
