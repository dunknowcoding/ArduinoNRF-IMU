#include "LSM6DS3.h"

namespace nimu {
using namespace lsm6ds3;

namespace {
inline int16_t le16(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}
}  // namespace

bool LSM6DS3::begin() {
  if (beginI2C(Wire, kAddrSA0High)) {
    return true;
  }
  return beginI2C(Wire, kAddrSA0Low);
}

bool LSM6DS3::beginI2C(TwoWire& wire, uint8_t address) {
  bus_.beginI2C(wire, address, clockHz_);
  bus_.recoverBus();
  if (!isConnected()) {
    return false;
  }
  return reset() && configureDefaults();
}

bool LSM6DS3::beginSPI(SPIClass& spi, uint8_t csPin) {
  // ST sensors use bit7 for read and bit6 for multi-byte auto-increment.
  bus_.beginSPI(spi, csPin, 1000000, 0xC0);
  if (!isConnected()) {
    return false;
  }
  return reset() && configureDefaults();
}

uint8_t LSM6DS3::whoAmI() {
  uint8_t id = 0;
  bus_.readRegister(WHO_AM_I, id);
  return id;
}

bool LSM6DS3::isConnected() {
  return whoAmI() == kWhoAmI;
}

bool LSM6DS3::reset() {
  if (bus_.writeRegister(CTRL3_C, CTRL3_SW_RESET) != IMUStatus::Ok) {
    return false;
  }
  delay(20);
  return bus_.writeRegister(CTRL3_C, CTRL3_BDU | CTRL3_IF_INC) == IMUStatus::Ok;
}

bool LSM6DS3::configureDefaults() {
  bool ok = true;
  ok &= setAccelRangeG(accelRangeG_);
  ok &= setGyroRangeDps(gyroRangeDps_);
  ok &= setSampleRateHz(sampleRateHz_);
  ok &= setLowPassFilterHz(50);
  return ok;
}

bool LSM6DS3::readRaw(RawSample& out) {
  uint8_t t[2];
  uint8_t g[6];
  uint8_t a[6];
  if (bus_.readRegisters(OUT_TEMP_L, t, sizeof(t)) != IMUStatus::Ok ||
      bus_.readRegisters(OUTX_L_G, g, sizeof(g)) != IMUStatus::Ok ||
      bus_.readRegisters(OUTX_L_XL, a, sizeof(a)) != IMUStatus::Ok) {
    return false;
  }
  out.temp = le16(t);
  out.gx = le16(&g[0]);
  out.gy = le16(&g[2]);
  out.gz = le16(&g[4]);
  out.ax = le16(&a[0]);
  out.ay = le16(&a[2]);
  out.az = le16(&a[4]);
  return true;
}

bool LSM6DS3::update() {
  RawSample raw;
  if (!readRaw(raw)) {
    return false;
  }

  Vec3 a{raw.ax / accelLsbPerG_, raw.ay / accelLsbPerG_,
         raw.az / accelLsbPerG_};
  data_.accel = correct(a, cal_.accelBias, cal_.accelScale);

  Vec3 g{raw.gx / gyroLsbPerDps_, raw.gy / gyroLsbPerDps_,
         raw.gz / gyroLsbPerDps_};
  data_.gyro = correct(g, cal_.gyroBias, Vec3{1, 1, 1});

  data_.mag = Vec3{0, 0, 0};
  data_.temperature = 25.0f + (raw.temp / 16.0f);
  data_.timestamp = micros();
  return true;
}

bool LSM6DS3::setAccelRangeG(uint16_t maxG) {
  uint8_t fs;
  if (maxG <= 2) {
    fs = XL_FS_2; accelLsbPerG_ = 16393.4426f; accelRangeG_ = 2;
  } else if (maxG <= 4) {
    fs = XL_FS_4; accelLsbPerG_ = 8196.7213f; accelRangeG_ = 4;
  } else if (maxG <= 8) {
    fs = XL_FS_8; accelLsbPerG_ = 4098.3607f; accelRangeG_ = 8;
  } else {
    fs = XL_FS_16; accelLsbPerG_ = 2049.1803f; accelRangeG_ = 16;
  }
  return bus_.updateRegister(CTRL1_XL, XL_FS_MASK, fs) == IMUStatus::Ok;
}

bool LSM6DS3::setGyroRangeDps(uint16_t maxDps) {
  uint8_t fs;
  if (maxDps <= 245) {
    fs = G_FS_245; gyroLsbPerDps_ = 114.2857f; gyroRangeDps_ = 245;
  } else if (maxDps <= 500) {
    fs = G_FS_500; gyroLsbPerDps_ = 57.1429f; gyroRangeDps_ = 500;
  } else if (maxDps <= 1000) {
    fs = G_FS_1000; gyroLsbPerDps_ = 28.5714f; gyroRangeDps_ = 1000;
  } else {
    fs = G_FS_2000; gyroLsbPerDps_ = 14.2857f; gyroRangeDps_ = 2000;
  }
  return bus_.updateRegister(CTRL2_G, G_FS_MASK, fs) == IMUStatus::Ok;
}

uint8_t LSM6DS3::odrBitsForHz(uint16_t hz, uint16_t& actualHz) const {
  if (hz <= 13) { actualHz = 12; return ODR_12_5; }
  if (hz <= 26) { actualHz = 26; return ODR_26; }
  if (hz <= 52) { actualHz = 52; return ODR_52; }
  if (hz <= 104) { actualHz = 104; return ODR_104; }
  if (hz <= 208) { actualHz = 208; return ODR_208; }
  if (hz <= 416) { actualHz = 416; return ODR_416; }
  if (hz <= 833) { actualHz = 833; return ODR_833; }
  actualHz = 1660;
  return ODR_1660;
}

bool LSM6DS3::setSampleRateHz(uint16_t hz) {
  if (hz == 0) {
    return false;
  }
  uint16_t actual = 0;
  odrBits_ = odrBitsForHz(hz, actual);
  sampleRateHz_ = actual;
  bool ok = true;
  ok &= bus_.updateRegister(CTRL1_XL, 0xF0, odrBits_) == IMUStatus::Ok;
  ok &= bus_.updateRegister(CTRL2_G, 0xF0, odrBits_) == IMUStatus::Ok;
  return ok;
}

bool LSM6DS3::setLowPassFilterHz(uint16_t hz) {
  (void)hz;
  // Keep the default ODR-derived bandwidth. The common API call is accepted so
  // sketches can stay portable; exact LSM6DS3 LPF routing can be added later.
  return true;
}

bool LSM6DS3::dataReady() {
  uint8_t status = 0;
  if (bus_.readRegister(STATUS_REG, status) != IMUStatus::Ok) {
    return false;
  }
  return (status & (STATUS_XLDA | STATUS_GDA)) == (STATUS_XLDA | STATUS_GDA);
}

bool LSM6DS3::configureInterruptPins(bool activeLow, bool openDrain) {
  uint8_t value = (activeLow ? CTRL3_H_LACTIVE : 0) |
                  (openDrain ? CTRL3_PP_OD : 0);
  return bus_.updateRegister(CTRL3_C, CTRL3_H_LACTIVE | CTRL3_PP_OD, value) ==
         IMUStatus::Ok;
}

bool LSM6DS3::routeInterrupt(uint8_t pin, uint8_t sources) {
  if (pin == 1) return bus_.writeRegister(INT1_CTRL, sources) == IMUStatus::Ok;
  if (pin == 2) return bus_.writeRegister(INT2_CTRL, sources) == IMUStatus::Ok;
  return false;
}

bool LSM6DS3::setDataReadyInterrupt(uint8_t pin, bool accel, bool gyro) {
  uint8_t sources = (accel ? INT_DRDY_XL : 0) | (gyro ? INT_DRDY_G : 0);
  return routeInterrupt(pin, sources);
}

bool LSM6DS3::embeddedAccess(bool enable) {
  return bus_.updateRegister(FUNC_CFG_ACCESS, FUNC_CFG_EN,
                             enable ? FUNC_CFG_EN : 0) == IMUStatus::Ok;
}

bool LSM6DS3::enablePedometer(uint8_t interruptPin) {
  if (interruptPin != 1 && interruptPin != 2) return false;
  if (!setSampleRateHz(26) || !setAccelRangeG(2) || !embeddedAccess(true))
    return false;
  bool ok = bus_.updateRegister(PEDO_THS_MIN, 0x1F, 0x17) == IMUStatus::Ok;
  ok &= embeddedAccess(false);
  ok &= bus_.updateRegister(CTRL10_C, CTRL10_FUNC_EN | CTRL10_PEDO_EN,
                            CTRL10_FUNC_EN | CTRL10_PEDO_EN) == IMUStatus::Ok;
  uint8_t route = interruptPin == 1 ? INT1_CTRL : INT2_CTRL;
  ok &= bus_.updateRegister(route, INT_STEP, INT_STEP) == IMUStatus::Ok;
  return ok;
}

bool LSM6DS3::disablePedometer() {
  bool ok = bus_.updateRegister(INT1_CTRL, INT_STEP, 0) == IMUStatus::Ok;
  ok &= bus_.updateRegister(INT2_CTRL, INT_STEP, 0) == IMUStatus::Ok;
  ok &= bus_.updateRegister(CTRL10_C, CTRL10_PEDO_EN, 0) == IMUStatus::Ok;
  return ok;
}

uint16_t LSM6DS3::stepCount() {
  uint8_t value[2] = {0, 0};
  if (bus_.readRegisters(STEP_COUNTER_L, value, sizeof(value)) != IMUStatus::Ok)
    return 0;
  return static_cast<uint16_t>(value[0]) |
         (static_cast<uint16_t>(value[1]) << 8);
}

bool LSM6DS3::resetStepCount() {
  if (bus_.updateRegister(CTRL10_C, CTRL10_PEDO_RST_STEP,
                          CTRL10_PEDO_RST_STEP) != IMUStatus::Ok) return false;
  delay(10);
  return bus_.updateRegister(CTRL10_C, CTRL10_PEDO_RST_STEP, 0) == IMUStatus::Ok;
}

bool LSM6DS3::stepDetected() {
  uint8_t status = 0;
  return bus_.readRegister(FUNC_SRC, status) == IMUStatus::Ok &&
         (status & FUNC_STEP_DETECTED) != 0;
}

bool LSM6DS3::enableTiltDetection(uint8_t interruptPin) {
  if (interruptPin != 1 && interruptPin != 2) return false;
  if (!setSampleRateHz(26) || !setAccelRangeG(2)) return false;
  bool ok = bus_.updateRegister(CTRL10_C, CTRL10_FUNC_EN,
                                CTRL10_FUNC_EN) == IMUStatus::Ok;
  ok &= bus_.updateRegister(TAP_CFG, TAP_TILT_ENABLE,
                            TAP_TILT_ENABLE) == IMUStatus::Ok;
  uint8_t route = interruptPin == 1 ? MD1_CFG : MD2_CFG;
  ok &= bus_.updateRegister(route, MD_TILT, MD_TILT) == IMUStatus::Ok;
  return ok;
}

bool LSM6DS3::disableTiltDetection() {
  bool ok = bus_.updateRegister(MD1_CFG, MD_TILT, 0) == IMUStatus::Ok;
  ok &= bus_.updateRegister(MD2_CFG, MD_TILT, 0) == IMUStatus::Ok;
  ok &= bus_.updateRegister(TAP_CFG, TAP_TILT_ENABLE, 0) == IMUStatus::Ok;
  return ok;
}

bool LSM6DS3::tiltDetected() {
  uint8_t status = 0;
  return bus_.readRegister(FUNC_SRC, status) == IMUStatus::Ok &&
         (status & FUNC_TILT_DETECTED) != 0;
}

bool LSM6DS3::enableSignificantMotion(uint8_t interruptPin) {
  if (interruptPin != 1 && interruptPin != 2) return false;
  if (!setSampleRateHz(26) || !setAccelRangeG(2)) return false;
  bool ok = bus_.updateRegister(CTRL10_C,
                                CTRL10_FUNC_EN | CTRL10_SIGN_MOTION_EN,
                                CTRL10_FUNC_EN | CTRL10_SIGN_MOTION_EN) ==
            IMUStatus::Ok;
  uint8_t route = interruptPin == 1 ? INT1_CTRL : INT2_CTRL;
  ok &= bus_.updateRegister(route, INT_SIGN_MOT, INT_SIGN_MOT) == IMUStatus::Ok;
  return ok;
}

bool LSM6DS3::significantMotionDetected() {
  uint8_t status = 0;
  return bus_.readRegister(FUNC_SRC, status) == IMUStatus::Ok &&
         (status & FUNC_SIGN_MOTION_DETECTED) != 0;
}

bool LSM6DS3::enableSensorHubPullups(bool enable) {
  return bus_.updateRegister(MASTER_CONFIG, SENSOR_HUB_PULL_UP,
                             enable ? SENSOR_HUB_PULL_UP : 0) == IMUStatus::Ok;
}

bool LSM6DS3::setSensorHubPassThrough(bool enable) {
  if (enable) {
    if (bus_.updateRegister(MASTER_CONFIG, MASTER_ON, 0) != IMUStatus::Ok)
      return false;
  }
  return bus_.updateRegister(MASTER_CONFIG, PASS_THRU_MODE,
                             enable ? PASS_THRU_MODE : 0) == IMUStatus::Ok;
}

bool LSM6DS3::waitSensorHub(uint16_t timeoutMs) {
  uint32_t start = millis();
  uint8_t status = 0;
  do {
    if (bus_.readRegister(FUNC_SRC, status) != IMUStatus::Ok) return false;
    if ((status & FUNC_SENS_HUB_END) != 0) return true;
  } while (static_cast<uint32_t>(millis() - start) < timeoutMs);
  return false;
}

bool LSM6DS3::sensorHubRead(uint8_t address, uint8_t reg, uint8_t* data,
                            uint8_t length, uint16_t timeoutMs) {
  if (data == nullptr || length == 0 || length > 7 || !embeddedAccess(true))
    return false;
  bool ok = bus_.writeRegister(SLV0_ADD, (address << 1) | 0x01) == IMUStatus::Ok;
  ok &= bus_.writeRegister(SLV0_SUBADD, reg) == IMUStatus::Ok;
  ok &= bus_.updateRegister(SLAVE0_CONFIG, 0x07, length) == IMUStatus::Ok;
  ok &= embeddedAccess(false);
  ok &= bus_.updateRegister(CTRL10_C, CTRL10_FUNC_EN,
                            CTRL10_FUNC_EN) == IMUStatus::Ok;
  ok &= bus_.updateRegister(MASTER_CONFIG, MASTER_ON, MASTER_ON) == IMUStatus::Ok;
  if (!ok || !waitSensorHub(timeoutMs)) return false;
  ok = bus_.readRegisters(SENSORHUB1, data, length) == IMUStatus::Ok;
  ok &= bus_.updateRegister(MASTER_CONFIG, MASTER_ON, 0) == IMUStatus::Ok;
  return ok;
}

bool LSM6DS3::sensorHubWrite(uint8_t address, uint8_t reg, uint8_t value,
                             uint16_t timeoutMs) {
  if (!embeddedAccess(true)) return false;
  bool ok = bus_.writeRegister(SLV0_ADD, address << 1) == IMUStatus::Ok;
  ok &= bus_.writeRegister(SLV0_SUBADD, reg) == IMUStatus::Ok;
  ok &= bus_.writeRegister(DATAWRITE_SLV0, value) == IMUStatus::Ok;
  ok &= embeddedAccess(false);
  ok &= bus_.updateRegister(CTRL10_C, CTRL10_FUNC_EN,
                            CTRL10_FUNC_EN) == IMUStatus::Ok;
  ok &= bus_.updateRegister(MASTER_CONFIG, MASTER_ON, MASTER_ON) == IMUStatus::Ok;
  if (!ok || !waitSensorHub(timeoutMs)) return false;
  return bus_.updateRegister(MASTER_CONFIG, MASTER_ON, 0) == IMUStatus::Ok;
}

void LSM6DS3::setBusClockHz(uint32_t hz) {
  clockHz_ = hz;
  bus_.setClockHz(hz);
}

}  // namespace nimu
