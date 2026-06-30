#include "L3G4200D.h"

namespace nimu {
using namespace l3g4200d;

namespace {
inline int16_t le16(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}
}  // namespace

bool L3G4200D::begin() {
  if (beginI2C(Wire, kAddrSDOHigh)) {
    return true;
  }
  return beginI2C(Wire, kAddrSDOLow);
}

bool L3G4200D::beginI2C(TwoWire& wire, uint8_t address) {
  bus_.beginI2C(wire, address, clockHz_);
  bus_.recoverBus();
  if (!isConnected()) {
    return false;
  }
  return configureDefaults();
}

bool L3G4200D::beginSPI(SPIClass& spi, uint8_t csPin) {
  bus_.beginSPI(spi, csPin, 1000000, 0xC0);
  if (!isConnected()) {
    return false;
  }
  return configureDefaults();
}

uint8_t L3G4200D::whoAmI() {
  uint8_t id = 0;
  bus_.readRegister(WHO_AM_I, id);
  return id;
}

bool L3G4200D::isConnected() {
  return whoAmI() == kWhoAmI;
}

bool L3G4200D::configureDefaults() {
  bool ok = true;
  ok &= bus_.writeRegister(CTRL_REG2, 0x00) == IMUStatus::Ok;
  ok &= bus_.writeRegister(CTRL_REG3, 0x00) == IMUStatus::Ok;
  ok &= bus_.writeRegister(CTRL_REG5, 0x00) == IMUStatus::Ok;
  ok &= setGyroRangeDps(gyroRangeDps_);
  ok &= setSampleRateHz(sampleRateHz_);
  ok &= setLowPassFilterHz(25);
  return ok;
}

bool L3G4200D::readRaw(RawSample& out) {
  uint8_t t = 0;
  uint8_t g[6];
  if (bus_.readRegister(OUT_TEMP, t) != IMUStatus::Ok ||
      bus_.readRegisters(OUT_X_L | AUTO_INCREMENT, g, sizeof(g)) !=
          IMUStatus::Ok) {
    return false;
  }
  out.temp = static_cast<int8_t>(t);
  out.gx = le16(&g[0]);
  out.gy = le16(&g[2]);
  out.gz = le16(&g[4]);
  return true;
}

bool L3G4200D::update() {
  RawSample raw;
  if (!readRaw(raw)) {
    return false;
  }
  data_.accel = Vec3{0, 0, 0};
  Vec3 g{raw.gx / gyroLsbPerDps_, raw.gy / gyroLsbPerDps_,
         raw.gz / gyroLsbPerDps_};
  data_.gyro = correct(g, cal_.gyroBias, Vec3{1, 1, 1});
  data_.mag = Vec3{0, 0, 0};
  data_.temperature = static_cast<float>(raw.temp);
  data_.timestamp = micros();
  return true;
}

bool L3G4200D::setAccelRangeG(uint16_t maxG) {
  (void)maxG;
  return true;
}

bool L3G4200D::setGyroRangeDps(uint16_t maxDps) {
  uint8_t fs;
  if (maxDps <= 250) {
    fs = FS_250; gyroLsbPerDps_ = 114.2857f; gyroRangeDps_ = 250;
  } else if (maxDps <= 500) {
    fs = FS_500; gyroLsbPerDps_ = 57.1429f; gyroRangeDps_ = 500;
  } else {
    fs = FS_2000; gyroLsbPerDps_ = 14.2857f; gyroRangeDps_ = 2000;
  }
  return bus_.updateRegister(CTRL_REG4, CTRL4_BDU | FS_MASK,
                             CTRL4_BDU | fs) == IMUStatus::Ok;
}

uint8_t L3G4200D::odrBitsForHz(uint16_t hz, uint16_t& actualHz) const {
  if (hz <= 100) { actualHz = 100; return 0x00; }
  if (hz <= 200) { actualHz = 200; return 0x40; }
  if (hz <= 400) { actualHz = 400; return 0x80; }
  actualHz = 800;
  return 0xC0;
}

bool L3G4200D::setSampleRateHz(uint16_t hz) {
  if (hz == 0) {
    return false;
  }
  uint16_t actual = 0;
  odrBits_ = odrBitsForHz(hz, actual);
  sampleRateHz_ = actual;
  return bus_.writeRegister(CTRL_REG1, odrBits_ | CTRL1_POWER_XYZ) ==
         IMUStatus::Ok;
}

bool L3G4200D::setLowPassFilterHz(uint16_t hz) {
  uint8_t bw = 0x00;
  if (hz >= 70) bw = 0x30;
  else if (hz >= 50) bw = 0x20;
  else if (hz >= 25) bw = 0x10;
  return bus_.writeRegister(CTRL_REG1, odrBits_ | bw | CTRL1_POWER_XYZ) ==
         IMUStatus::Ok;
}

bool L3G4200D::dataReady() {
  uint8_t status = 0;
  if (bus_.readRegister(STATUS_REG, status) != IMUStatus::Ok) {
    return false;
  }
  return (status & STATUS_ZYXDA) != 0;
}

bool L3G4200D::configureInterruptPins(bool activeLow, bool openDrain) {
  uint8_t value = (activeLow ? CTRL3_H_LACTIVE : 0) |
                  (openDrain ? CTRL3_PP_OD : 0);
  return bus_.updateRegister(CTRL_REG3, CTRL3_H_LACTIVE | CTRL3_PP_OD,
                             value) == IMUStatus::Ok;
}

bool L3G4200D::setDataReadyInterrupt(bool enable) {
  return bus_.updateRegister(CTRL_REG3, CTRL3_I2_DRDY,
                             enable ? CTRL3_I2_DRDY : 0) == IMUStatus::Ok;
}

bool L3G4200D::setThresholdInterrupt(bool enable) {
  return bus_.updateRegister(CTRL_REG3, CTRL3_I1_INT1,
                             enable ? CTRL3_I1_INT1 : 0) == IMUStatus::Ok;
}

void L3G4200D::setBusClockHz(uint32_t hz) {
  clockHz_ = hz;
  bus_.setClockHz(hz);
}

}  // namespace nimu
