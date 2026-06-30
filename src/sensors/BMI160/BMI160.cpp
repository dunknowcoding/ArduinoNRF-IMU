#include "BMI160.h"

namespace nimu {
using namespace bmi160;

namespace {
inline int16_t le16(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}
}  // namespace

bool BMI160::begin() {
  if (beginI2C(Wire, kAddrSDOLow)) {
    return true;
  }
  return beginI2C(Wire, kAddrSDOHigh);
}

bool BMI160::beginI2C(TwoWire& wire, uint8_t address) {
  bus_.beginI2C(wire, address, clockHz_);
  bus_.recoverBus();
  if (!isConnected()) {
    return false;
  }
  return reset() && configureDefaults();
}

bool BMI160::beginSPI(SPIClass& spi, uint8_t csPin) {
  bus_.beginSPI(spi, csPin, 1000000, 0x80);
  // BMI160 needs one dummy read after power-up/SPI selection on many boards.
  whoAmI();
  if (!isConnected()) {
    return false;
  }
  return reset() && configureDefaults();
}

uint8_t BMI160::whoAmI() {
  uint8_t id = 0;
  bus_.readRegister(CHIP_ID, id);
  return id;
}

bool BMI160::isConnected() {
  return whoAmI() == kChipId;
}

bool BMI160::reset() {
  if (bus_.writeRegister(CMD, CMD_SOFT_RESET) != IMUStatus::Ok) {
    return false;
  }
  delay(100);
  return true;
}

bool BMI160::configureDefaults() {
  if (bus_.writeRegister(CMD, CMD_ACC_NORMAL) != IMUStatus::Ok) {
    return false;
  }
  delay(5);
  if (bus_.writeRegister(CMD, CMD_GYR_NORMAL) != IMUStatus::Ok) {
    return false;
  }
  delay(80);
  bool ok = true;
  ok &= setAccelRangeG(accelRangeG_);
  ok &= setGyroRangeDps(gyroRangeDps_);
  ok &= setSampleRateHz(sampleRateHz_);
  ok &= setLowPassFilterHz(50);
  return ok;
}

bool BMI160::readRaw(RawSample& out) {
  uint8_t g[6];
  uint8_t a[6];
  uint8_t t[2];
  if (bus_.readRegisters(DATA_GYRO_X_L, g, sizeof(g)) != IMUStatus::Ok ||
      bus_.readRegisters(DATA_ACCEL_X_L, a, sizeof(a)) != IMUStatus::Ok ||
      bus_.readRegisters(DATA_TEMP_L, t, sizeof(t)) != IMUStatus::Ok) {
    return false;
  }
  out.gx = le16(&g[0]);
  out.gy = le16(&g[2]);
  out.gz = le16(&g[4]);
  out.ax = le16(&a[0]);
  out.ay = le16(&a[2]);
  out.az = le16(&a[4]);
  out.temp = le16(t);
  return true;
}

bool BMI160::update() {
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
  data_.temperature = 23.0f + (raw.temp / 512.0f);
  data_.timestamp = micros();
  return true;
}

bool BMI160::setAccelRangeG(uint16_t maxG) {
  uint8_t range;
  if (maxG <= 2) {
    range = 0x03; accelLsbPerG_ = 16384.0f; accelRangeG_ = 2;
  } else if (maxG <= 4) {
    range = 0x05; accelLsbPerG_ = 8192.0f; accelRangeG_ = 4;
  } else if (maxG <= 8) {
    range = 0x08; accelLsbPerG_ = 4096.0f; accelRangeG_ = 8;
  } else {
    range = 0x0C; accelLsbPerG_ = 2048.0f; accelRangeG_ = 16;
  }
  return bus_.writeRegister(ACC_RANGE, range) == IMUStatus::Ok;
}

bool BMI160::setGyroRangeDps(uint16_t maxDps) {
  uint8_t range;
  if (maxDps <= 125) {
    range = 0x04; gyroLsbPerDps_ = 262.4f; gyroRangeDps_ = 125;
  } else if (maxDps <= 250) {
    range = 0x03; gyroLsbPerDps_ = 131.2f; gyroRangeDps_ = 250;
  } else if (maxDps <= 500) {
    range = 0x02; gyroLsbPerDps_ = 65.6f; gyroRangeDps_ = 500;
  } else if (maxDps <= 1000) {
    range = 0x01; gyroLsbPerDps_ = 32.8f; gyroRangeDps_ = 1000;
  } else {
    range = 0x00; gyroLsbPerDps_ = 16.4f; gyroRangeDps_ = 2000;
  }
  return bus_.writeRegister(GYR_RANGE, range) == IMUStatus::Ok;
}

uint8_t BMI160::odrCodeForHz(uint16_t hz, uint16_t& actualHz) const {
  if (hz <= 25) { actualHz = 25; return 0x06; }
  if (hz <= 50) { actualHz = 50; return 0x07; }
  if (hz <= 100) { actualHz = 100; return 0x08; }
  if (hz <= 200) { actualHz = 200; return 0x09; }
  if (hz <= 400) { actualHz = 400; return 0x0A; }
  if (hz <= 800) { actualHz = 800; return 0x0B; }
  actualHz = 1600;
  return 0x0C;
}

bool BMI160::setSampleRateHz(uint16_t hz) {
  if (hz == 0) {
    return false;
  }
  uint16_t actual = 0;
  uint8_t odr = odrCodeForHz(hz, actual);
  sampleRateHz_ = actual;
  // Normal averaging/filter mode with ODR in low nibble.
  bool ok = true;
  ok &= bus_.writeRegister(ACC_CONF, 0x20 | odr) == IMUStatus::Ok;
  ok &= bus_.writeRegister(GYR_CONF, 0x20 | odr) == IMUStatus::Ok;
  return ok;
}

bool BMI160::setLowPassFilterHz(uint16_t hz) {
  (void)hz;
  return true;
}

bool BMI160::dataReady() {
  uint8_t status = 0;
  if (bus_.readRegister(STATUS, status) != IMUStatus::Ok) {
    return false;
  }
  return (status & (STATUS_DRDY_ACC | STATUS_DRDY_GYR)) ==
         (STATUS_DRDY_ACC | STATUS_DRDY_GYR);
}

bool BMI160::configureInterruptPin(uint8_t pin, bool activeHigh,
                                   bool openDrain, bool latched) {
  if (pin != 1 && pin != 2) return false;
  uint8_t shift = pin == 1 ? 0 : 4;
  uint8_t value = static_cast<uint8_t>((0x08 |
                                        (openDrain ? 0x04 : 0) |
                                        (activeHigh ? 0x02 : 0)) << shift);
  bool ok = bus_.updateRegister(INT_OUT_CTRL, static_cast<uint8_t>(0x0F << shift),
                                value) == IMUStatus::Ok;
  ok &= bus_.updateRegister(INT_LATCH, 0x0F, latched ? 0x0F : 0x00) ==
        IMUStatus::Ok;
  return ok;
}

bool BMI160::routeDataReadyInterrupt(uint8_t pin, bool enable) {
  if (pin != 1 && pin != 2) return false;
  uint8_t map = pin == 1 ? 0x80 : 0x08;
  bool ok = bus_.updateRegister(INT_MAP_1, 0x88, enable ? map : 0) ==
            IMUStatus::Ok;
  ok &= bus_.updateRegister(INT_EN_1, INT_DATA_READY_ENABLE,
                            enable ? INT_DATA_READY_ENABLE : 0) == IMUStatus::Ok;
  return ok;
}

void BMI160::setBusClockHz(uint32_t hz) {
  clockHz_ = hz;
  bus_.setClockHz(hz);
}

}  // namespace nimu
