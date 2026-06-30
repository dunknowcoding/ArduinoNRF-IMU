#include "ICM42688P.h"

namespace nimu {
using namespace icm42688p;

namespace {
inline int16_t be16(const uint8_t* p) {
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) |
                              static_cast<uint16_t>(p[1]));
}
}  // namespace

bool ICM42688P::begin() {
  if (beginI2C(Wire, kAddrAD0Low)) {
    return true;
  }
  return beginI2C(Wire, kAddrAD0High);
}

bool ICM42688P::beginI2C(TwoWire& wire, uint8_t address) {
  bus_.beginI2C(wire, address, clockHz_);
  bus_.recoverBus();
  if (!isConnected()) {
    return false;
  }
  return reset() && configureDefaults();
}

bool ICM42688P::beginSPI(SPIClass& spi, uint8_t csPin) {
  bus_.beginSPI(spi, csPin, 1000000, 0x80);
  if (!isConnected()) {
    return false;
  }
  return reset() && configureDefaults();
}

uint8_t ICM42688P::whoAmI() {
  uint8_t id = 0;
  bus_.readRegister(WHO_AM_I, id);
  return id;
}

bool ICM42688P::isConnected() {
  return whoAmI() == kWhoAmI;
}

bool ICM42688P::reset() {
  if (bus_.writeRegister(DEVICE_CONFIG, DEVICE_SOFT_RESET) != IMUStatus::Ok) {
    return false;
  }
  delay(10);
  return bus_.writeRegister(REG_BANK_SEL, 0x00) == IMUStatus::Ok;
}

bool ICM42688P::configureDefaults() {
  bool ok = true;
  ok &= setAccelRangeG(accelRangeG_);
  ok &= setGyroRangeDps(gyroRangeDps_);
  ok &= setSampleRateHz(sampleRateHz_);
  ok &= setLowPassFilterHz(50);
  ok &= bus_.updateRegister(INT_CONFIG1, 0x10, 0x00) == IMUStatus::Ok;
  ok &= bus_.writeRegister(PWR_MGMT0, PWR_GYRO_LN | PWR_ACCEL_LN) ==
        IMUStatus::Ok;
  delay(50);
  return ok;
}

bool ICM42688P::readRaw(RawSample& out) {
  uint8_t data[14];
  if (bus_.readRegisters(TEMP_DATA1, data, sizeof(data)) != IMUStatus::Ok) {
    return false;
  }
  out.temp = be16(&data[0]);
  out.ax = be16(&data[2]);
  out.ay = be16(&data[4]);
  out.az = be16(&data[6]);
  out.gx = be16(&data[8]);
  out.gy = be16(&data[10]);
  out.gz = be16(&data[12]);
  return true;
}

bool ICM42688P::update() {
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
  data_.temperature = (raw.temp / 132.48f) + 25.0f;
  data_.timestamp = micros();
  return true;
}

bool ICM42688P::setAccelRangeG(uint16_t maxG) {
  if (maxG <= 2) {
    accelFsCode_ = 0x03; accelLsbPerG_ = 16384.0f; accelRangeG_ = 2;
  } else if (maxG <= 4) {
    accelFsCode_ = 0x02; accelLsbPerG_ = 8192.0f; accelRangeG_ = 4;
  } else if (maxG <= 8) {
    accelFsCode_ = 0x01; accelLsbPerG_ = 4096.0f; accelRangeG_ = 8;
  } else {
    accelFsCode_ = 0x00; accelLsbPerG_ = 2048.0f; accelRangeG_ = 16;
  }
  return bus_.writeRegister(ACCEL_CONFIG0, (accelFsCode_ << 5) | odrCode_) ==
         IMUStatus::Ok;
}

bool ICM42688P::setGyroRangeDps(uint16_t maxDps) {
  if (maxDps <= 16) {
    gyroFsCode_ = 0x07; gyroLsbPerDps_ = 2097.0f; gyroRangeDps_ = 16;
  } else if (maxDps <= 31) {
    gyroFsCode_ = 0x06; gyroLsbPerDps_ = 1048.0f; gyroRangeDps_ = 31;
  } else if (maxDps <= 63) {
    gyroFsCode_ = 0x05; gyroLsbPerDps_ = 524.0f; gyroRangeDps_ = 63;
  } else if (maxDps <= 125) {
    gyroFsCode_ = 0x04; gyroLsbPerDps_ = 262.0f; gyroRangeDps_ = 125;
  } else if (maxDps <= 250) {
    gyroFsCode_ = 0x03; gyroLsbPerDps_ = 131.0f; gyroRangeDps_ = 250;
  } else if (maxDps <= 500) {
    gyroFsCode_ = 0x02; gyroLsbPerDps_ = 65.5f; gyroRangeDps_ = 500;
  } else if (maxDps <= 1000) {
    gyroFsCode_ = 0x01; gyroLsbPerDps_ = 32.8f; gyroRangeDps_ = 1000;
  } else {
    gyroFsCode_ = 0x00; gyroLsbPerDps_ = 16.4f; gyroRangeDps_ = 2000;
  }
  return bus_.writeRegister(GYRO_CONFIG0, (gyroFsCode_ << 5) | odrCode_) ==
         IMUStatus::Ok;
}

uint8_t ICM42688P::odrCodeForHz(uint16_t hz, uint16_t& actualHz) const {
  if (hz <= 13) { actualHz = 12; return 0x0B; }
  if (hz <= 25) { actualHz = 25; return 0x0A; }
  if (hz <= 50) { actualHz = 50; return 0x09; }
  if (hz <= 100) { actualHz = 100; return 0x08; }
  if (hz <= 200) { actualHz = 200; return 0x07; }
  if (hz <= 500) { actualHz = 500; return 0x0F; }
  if (hz <= 1000) { actualHz = 1000; return 0x06; }
  if (hz <= 2000) { actualHz = 2000; return 0x05; }
  if (hz <= 4000) { actualHz = 4000; return 0x04; }
  if (hz <= 8000) { actualHz = 8000; return 0x03; }
  if (hz <= 16000) { actualHz = 16000; return 0x02; }
  actualHz = 32000;
  return 0x01;
}

bool ICM42688P::setSampleRateHz(uint16_t hz) {
  if (hz == 0) {
    return false;
  }
  uint16_t actual = 0;
  odrCode_ = odrCodeForHz(hz, actual);
  sampleRateHz_ = actual;
  bool ok = true;
  ok &= setAccelRangeG(accelRangeG_);
  ok &= setGyroRangeDps(gyroRangeDps_);
  return ok;
}

bool ICM42688P::setLowPassFilterHz(uint16_t hz) {
  uint8_t code = 1;
  if (hz == 0) {
    code = 0;
  } else if (sampleRateHz_ > 0) {
    uint16_t ratio = sampleRateHz_ / hz;
    if (ratio <= 2) code = 0;
    else if (ratio <= 4) code = 1;
    else if (ratio <= 5) code = 2;
    else if (ratio <= 8) code = 3;
    else if (ratio <= 10) code = 4;
    else if (ratio <= 16) code = 5;
    else if (ratio <= 20) code = 6;
    else code = 7;
  }
  return bus_.writeRegister(GYRO_ACCEL_CONFIG0, (code << 4) | code) ==
         IMUStatus::Ok;
}

bool ICM42688P::dataReady() {
  uint8_t status = 0;
  if (bus_.readRegister(INT_STATUS, status) != IMUStatus::Ok) {
    return false;
  }
  return (status & INT_STATUS_DATA_READY) != 0;
}

bool ICM42688P::configureInterruptPin(uint8_t pin, bool activeHigh,
                                      bool openDrain, bool latched) {
  if (pin != 1 && pin != 2) return false;
  uint8_t shift = pin == 1 ? 0 : 3;
  uint8_t mask = static_cast<uint8_t>(0x07 << shift);
  uint8_t value = static_cast<uint8_t>(((latched ? 1 : 0) << 2 |
                                        (openDrain ? 1 : 0) << 1 |
                                        (activeHigh ? 1 : 0)) << shift);
  return bus_.updateRegister(INT_CONFIG, mask, value) == IMUStatus::Ok;
}

bool ICM42688P::routeDataReadyInterrupt(uint8_t pin, bool enable) {
  uint8_t reg = pin == 1 ? INT_SOURCE0 : pin == 2 ? INT_SOURCE3 : 0;
  return reg != 0 && bus_.updateRegister(reg, INT_UI_DRDY,
                                          enable ? INT_UI_DRDY : 0) ==
                         IMUStatus::Ok;
}

bool ICM42688P::routeFsyncInterrupt(uint8_t pin, bool enable) {
  uint8_t reg = pin == 1 ? INT_SOURCE0 : pin == 2 ? INT_SOURCE3 : 0;
  return reg != 0 && bus_.updateRegister(reg, INT_UI_FSYNC,
                                          enable ? INT_UI_FSYNC : 0) ==
                         IMUStatus::Ok;
}

bool ICM42688P::setPin9Function(Pin9Function function) {
  if (bus_.writeRegister(REG_BANK_SEL, 1) != IMUStatus::Ok) return false;
  bool ok = bus_.updateRegister(INTF_CONFIG5_B1, 0x06,
                                static_cast<uint8_t>(function) << 1) ==
            IMUStatus::Ok;
  ok &= bus_.writeRegister(REG_BANK_SEL, 0) == IMUStatus::Ok;
  return ok;
}

void ICM42688P::setBusClockHz(uint32_t hz) {
  clockHz_ = hz;
  bus_.setClockHz(hz);
}

}  // namespace nimu
