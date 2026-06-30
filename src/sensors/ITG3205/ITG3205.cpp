#include "ITG3205.h"

namespace nimu {
using namespace itg3205;

namespace {
inline int16_t be16(const uint8_t* p) {
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) |
                              static_cast<uint16_t>(p[1]));
}
}  // namespace

bool ITG3205::begin() {
  if (beginI2C(Wire, kAddrAD0Low)) {
    return true;
  }
  return beginI2C(Wire, kAddrAD0High);
}

bool ITG3205::beginI2C(TwoWire& wire, uint8_t address) {
  bus_.beginI2C(wire, address, clockHz_);
  bus_.recoverBus();
  if (!isConnected()) {
    return false;
  }
  return reset() && configureDefaults();
}

bool ITG3205::beginSPI(SPIClass& spi, uint8_t csPin) {
  (void)spi;
  (void)csPin;
  return false;
}

uint8_t ITG3205::whoAmI() {
  uint8_t id = 0;
  bus_.readRegister(WHO_AM_I, id);
  return id;
}

bool ITG3205::isConnected() {
  return (whoAmI() & 0x7E) == kWhoAmIMasked;
}

bool ITG3205::reset() {
  if (bus_.writeRegister(PWR_MGM, H_RESET) != IMUStatus::Ok) {
    return false;
  }
  delay(20);
  return bus_.writeRegister(PWR_MGM, CLK_SEL_X_GYRO) == IMUStatus::Ok;
}

bool ITG3205::configureDefaults() {
  bool ok = true;
  ok &= setLowPassFilterHz(42);
  ok &= setSampleRateHz(sampleRateHz_);
  ok &= bus_.writeRegister(INT_CFG, RAW_RDY_EN) == IMUStatus::Ok;
  return ok;
}

bool ITG3205::readRaw(RawSample& out) {
  uint8_t b[8];
  if (bus_.readRegisters(TEMP_OUT_H, b, sizeof(b)) != IMUStatus::Ok) {
    return false;
  }
  out.temp = be16(&b[0]);
  out.gx = be16(&b[2]);
  out.gy = be16(&b[4]);
  out.gz = be16(&b[6]);
  return true;
}

bool ITG3205::update() {
  RawSample raw;
  if (!readRaw(raw)) {
    return false;
  }
  data_.accel = Vec3{0, 0, 0};
  Vec3 g{raw.gx / gyroLsbPerDps_, raw.gy / gyroLsbPerDps_,
         raw.gz / gyroLsbPerDps_};
  data_.gyro = correct(g, cal_.gyroBias, Vec3{1, 1, 1});
  data_.mag = Vec3{0, 0, 0};
  data_.temperature = 35.0f + ((raw.temp + 13200.0f) / 280.0f);
  data_.timestamp = micros();
  return true;
}

bool ITG3205::setAccelRangeG(uint16_t maxG) {
  (void)maxG;
  return true;
}

bool ITG3205::setGyroRangeDps(uint16_t maxDps) {
  (void)maxDps;
  return bus_.writeRegister(DLPF_FS, FS_SEL_2000 | dlpfCode_) == IMUStatus::Ok;
}

uint8_t ITG3205::dlpfCodeForHz(uint16_t hz) const {
  if (hz <= 5) return 0x06;
  if (hz <= 10) return 0x05;
  if (hz <= 20) return 0x04;
  if (hz <= 42) return 0x03;
  if (hz <= 98) return 0x02;
  if (hz <= 188) return 0x01;
  return 0x00;
}

bool ITG3205::setLowPassFilterHz(uint16_t hz) {
  dlpfCode_ = dlpfCodeForHz(hz);
  return bus_.writeRegister(DLPF_FS, FS_SEL_2000 | dlpfCode_) == IMUStatus::Ok;
}

bool ITG3205::setSampleRateHz(uint16_t hz) {
  if (hz == 0) {
    return false;
  }
  uint16_t baseHz = (dlpfCode_ == 0) ? 8000 : 1000;
  uint16_t divider = (hz >= baseHz) ? 0 : ((baseHz / hz) - 1);
  if (divider > 255) {
    divider = 255;
  }
  sampleRateHz_ = baseHz / (divider + 1);
  return bus_.writeRegister(SMPLRT_DIV, static_cast<uint8_t>(divider)) ==
         IMUStatus::Ok;
}

bool ITG3205::dataReady() {
  uint8_t status = 0;
  if (bus_.readRegister(INT_STATUS, status) != IMUStatus::Ok) {
    return false;
  }
  return (status & RAW_DATA_RDY) != 0;
}

bool ITG3205::configureInterrupt(bool activeLow, bool openDrain,
                                 bool latched, bool clearOnAnyRead) {
  uint8_t value = (activeLow ? INT_ACTIVE_LOW : 0) |
                  (openDrain ? INT_OPEN_DRAIN : 0) |
                  (latched ? INT_LATCH : 0) |
                  (clearOnAnyRead ? INT_CLEAR_ANY_READ : 0);
  return bus_.updateRegister(INT_CFG, INT_ACTIVE_LOW | INT_OPEN_DRAIN |
                                          INT_LATCH | INT_CLEAR_ANY_READ,
                             value) == IMUStatus::Ok;
}

bool ITG3205::setDataReadyInterrupt(bool enable) {
  return bus_.updateRegister(INT_CFG, RAW_RDY_EN,
                             enable ? RAW_RDY_EN : 0) == IMUStatus::Ok;
}

bool ITG3205::setPllReadyInterrupt(bool enable) {
  return bus_.updateRegister(INT_CFG, PLL_RDY_EN,
                             enable ? PLL_RDY_EN : 0) == IMUStatus::Ok;
}

void ITG3205::setBusClockHz(uint32_t hz) {
  clockHz_ = hz;
  bus_.setClockHz(hz);
}

}  // namespace nimu
