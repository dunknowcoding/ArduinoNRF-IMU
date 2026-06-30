#include "LSM6DSV.h"

namespace nimu {
using namespace lsm6dsv;

namespace {
int16_t le16(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}
}

bool LSM6DSV::begin() {
  return beginI2C(Wire, kAddrSA0Low) || beginI2C(Wire, kAddrSA0High);
}

bool LSM6DSV::beginI2C(TwoWire& wire, uint8_t address) {
  bus_.beginI2C(wire, address, 400000);
  bus_.recoverBus();
  if (!isConnected() || !reset()) return false;
  return setAccelRangeG(4) && setGyroRangeDps(1000) &&
         setSampleRateHz(120);
}

bool LSM6DSV::beginSPI(SPIClass& spi, uint8_t csPin) {
  bus_.beginSPI(spi, csPin, 1000000, 0x80);
  if (!isConnected() || !reset()) return false;
  return setAccelRangeG(4) && setGyroRangeDps(1000) &&
         setSampleRateHz(120);
}

uint8_t LSM6DSV::whoAmI() {
  uint8_t id = 0;
  bus_.readRegister(WHO_AM_I, id);
  return id;
}

bool LSM6DSV::isConnected() { return whoAmI() == kWhoAmI; }

bool LSM6DSV::reset() {
  if (bus_.writeRegister(CTRL3, CTRL3_SW_RESET) != IMUStatus::Ok) return false;
  delay(20);
  return bus_.writeRegister(CTRL3, CTRL3_BDU | CTRL3_IF_INC) == IMUStatus::Ok;
}

bool LSM6DSV::readRaw(RawSample& out) {
  uint8_t raw[14];
  if (bus_.readRegisters(OUT_TEMP_L, raw, sizeof(raw)) != IMUStatus::Ok)
    return false;
  out.temp = le16(&raw[0]);
  out.gx = le16(&raw[2]);
  out.gy = le16(&raw[4]);
  out.gz = le16(&raw[6]);
  out.ax = le16(&raw[8]);
  out.ay = le16(&raw[10]);
  out.az = le16(&raw[12]);
  return true;
}

bool LSM6DSV::update() {
  RawSample raw;
  if (!readRaw(raw)) return false;
  data_.accel = correct(Vec3{raw.ax / accelLsbPerG_, raw.ay / accelLsbPerG_,
                              raw.az / accelLsbPerG_},
                        cal_.accelBias, cal_.accelScale);
  data_.gyro = correct(Vec3{raw.gx / gyroLsbPerDps_, raw.gy / gyroLsbPerDps_,
                             raw.gz / gyroLsbPerDps_},
                       cal_.gyroBias, Vec3{1, 1, 1});
  if (qmcEnabled_) {
    Vec3 mag;
    if (readQMC6309(mag)) data_.mag = correct(mag, cal_.magBias, cal_.magScale);
  }
  data_.temperature = 25.0f + raw.temp / 256.0f;
  data_.timestamp = micros();
  return true;
}

bool LSM6DSV::setAccelRangeG(uint16_t maxG) {
  uint8_t code;
  uint16_t range;
  if (maxG <= 2) { code = 0; range = 2; }
  else if (maxG <= 4) { code = 1; range = 4; }
  else if (maxG <= 8) { code = 2; range = 8; }
  else { code = 3; range = 16; }
  accelLsbPerG_ = 32768.0f / range;
  return bus_.updateRegister(CTRL8, 0x03, code) == IMUStatus::Ok;
}

bool LSM6DSV::setGyroRangeDps(uint16_t maxDps) {
  uint8_t code;
  uint16_t range;
  if (maxDps <= 125) { code = 0; range = 125; }
  else if (maxDps <= 250) { code = 1; range = 250; }
  else if (maxDps <= 500) { code = 2; range = 500; }
  else if (maxDps <= 1000) { code = 3; range = 1000; }
  else if (maxDps <= 2000) { code = 4; range = 2000; }
  else { code = 12; range = 4000; }
  gyroLsbPerDps_ = 32768.0f / range;
  return bus_.updateRegister(CTRL6, 0x0F, code) == IMUStatus::Ok;
}

bool LSM6DSV::setSampleRateHz(uint16_t hz) {
  if (hz == 0) return false;
  if (hz <= 2) odrCode_ = 1;
  else if (hz <= 8) odrCode_ = 2;
  else if (hz <= 15) odrCode_ = 3;
  else if (hz <= 30) odrCode_ = 4;
  else if (hz <= 60) odrCode_ = 5;
  else if (hz <= 120) odrCode_ = 6;
  else if (hz <= 240) odrCode_ = 7;
  else if (hz <= 480) odrCode_ = 8;
  else if (hz <= 960) odrCode_ = 9;
  else if (hz <= 1920) odrCode_ = 10;
  else if (hz <= 3840) odrCode_ = 11;
  else odrCode_ = 12;
  return bus_.updateRegister(CTRL1, 0x0F, odrCode_) == IMUStatus::Ok &&
         bus_.updateRegister(CTRL2, 0x0F, odrCode_) == IMUStatus::Ok;
}

bool LSM6DSV::setLowPassFilterHz(uint16_t hz) {
  (void)hz;
  return true;
}

bool LSM6DSV::dataReady() {
  uint8_t status = 0;
  return bus_.readRegister(STATUS_REG, status) == IMUStatus::Ok &&
         (status & (STATUS_XLDA | STATUS_GDA)) ==
             (STATUS_XLDA | STATUS_GDA);
}

bool LSM6DSV::configureInterruptPins(bool activeLow, bool openDrain) {
  uint8_t value = (activeLow ? IF_H_LACTIVE : 0) |
                  (openDrain ? IF_PP_OD : 0);
  return bus_.updateRegister(IF_CFG, IF_H_LACTIVE | IF_PP_OD, value) ==
         IMUStatus::Ok;
}

bool LSM6DSV::routeInterrupt(uint8_t pin, uint8_t sources) {
  uint8_t reg = pin == 1 ? INT1_CTRL : pin == 2 ? INT2_CTRL : 0;
  return reg != 0 && bus_.writeRegister(reg, sources) == IMUStatus::Ok;
}

bool LSM6DSV::setDataReadyInterrupt(uint8_t pin, bool accel, bool gyro) {
  return routeInterrupt(pin, (accel ? INT_DRDY_XL : 0) |
                                 (gyro ? INT_DRDY_G : 0));
}

bool LSM6DSV::configureOisInterface(bool accel, bool gyro) {
  if (bus_.updateRegister(FUNC_CFG_ACCESS, OIS_CTRL_FROM_UI,
                          OIS_CTRL_FROM_UI) != IMUStatus::Ok) return false;
  uint8_t enable = (gyro ? OIS_G_ENABLE : 0) |
                   (accel ? OIS_XL_ENABLE : 0);
  return bus_.updateRegister(UI_CTRL1_OIS, OIS_G_ENABLE | OIS_XL_ENABLE,
                             enable) == IMUStatus::Ok;
}

bool LSM6DSV::oisDataReady() {
  uint8_t status = 0;
  return bus_.readRegister(UI_STATUS_REG_OIS, status) == IMUStatus::Ok &&
         (status & (OIS_XL_DATA_READY | OIS_G_DATA_READY)) != 0;
}

bool LSM6DSV::sensorHubBank(bool enable) {
  return bus_.updateRegister(FUNC_CFG_ACCESS, SHUB_ACCESS,
                             enable ? SHUB_ACCESS : 0) == IMUStatus::Ok;
}

bool LSM6DSV::embeddedBank(bool enable) {
  return bus_.updateRegister(FUNC_CFG_ACCESS, SHUB_ACCESS | EMBED_ACCESS,
                             enable ? EMBED_ACCESS : 0) == IMUStatus::Ok;
}

bool LSM6DSV::waitSensorHub(uint16_t timeoutMs) {
  uint32_t start = millis();
  uint8_t status = 0;
  do {
    if (bus_.readRegister(STATUS_MASTER, status) != IMUStatus::Ok) return false;
    if ((status & STATUS_SHUB_DONE) != 0) return true;
  } while (static_cast<uint32_t>(millis() - start) < timeoutMs);
  return false;
}

bool LSM6DSV::enableSensorHubPullups(bool enable) {
  return bus_.updateRegister(IF_CFG, IF_SHUB_PU_EN,
                             enable ? IF_SHUB_PU_EN : 0) == IMUStatus::Ok;
}

bool LSM6DSV::sensorHubRead(uint8_t address, uint8_t reg, uint8_t* data,
                            uint8_t length, uint16_t timeoutMs) {
  if (data == nullptr || length == 0 || length > 7 || !sensorHubBank(true))
    return false;
  bool ok = bus_.writeRegister(SLV0_ADD, (address << 1) | 0x01) == IMUStatus::Ok;
  ok &= bus_.writeRegister(SLV0_SUBADD, reg) == IMUStatus::Ok;
  ok &= bus_.updateRegister(SLV0_CONFIG, 0x07, length) == IMUStatus::Ok;
  ok &= bus_.updateRegister(MASTER_CONFIG, MASTER_ON, MASTER_ON) == IMUStatus::Ok;
  ok &= sensorHubBank(false);
  if (!ok || !waitSensorHub(timeoutMs) || !sensorHubBank(true)) return false;
  ok = bus_.readRegisters(SENSOR_HUB_1, data, length) == IMUStatus::Ok;
  ok &= bus_.updateRegister(MASTER_CONFIG, MASTER_ON, 0) == IMUStatus::Ok;
  ok &= sensorHubBank(false);
  return ok;
}

bool LSM6DSV::sensorHubWrite(uint8_t address, uint8_t reg, uint8_t value,
                             uint16_t timeoutMs) {
  if (!sensorHubBank(true)) return false;
  bool ok = bus_.writeRegister(SLV0_ADD, address << 1) == IMUStatus::Ok;
  ok &= bus_.writeRegister(SLV0_SUBADD, reg) == IMUStatus::Ok;
  ok &= bus_.writeRegister(DATAWRITE_SLV0, value) == IMUStatus::Ok;
  ok &= bus_.updateRegister(MASTER_CONFIG, MASTER_ON, MASTER_ON) == IMUStatus::Ok;
  ok &= sensorHubBank(false);
  if (!ok || !waitSensorHub(timeoutMs) || !sensorHubBank(true)) return false;
  ok = bus_.updateRegister(MASTER_CONFIG, MASTER_ON, 0) == IMUStatus::Ok;
  ok &= sensorHubBank(false);
  return ok;
}

bool LSM6DSV::configureQMC6309() {
  if (!enableSensorHubPullups()) return false;
  uint8_t id = 0;
  if (!sensorHubRead(0x7C, 0x00, &id, 1) || id != 0x90) return false;
  if (!sensorHubWrite(0x7C, 0x0B, 0x80) ||
      !sensorHubWrite(0x7C, 0x0B, 0x00)) return false;
  delay(10);
  qmcEnabled_ = sensorHubWrite(0x7C, 0x0B, 0x48) &&
                sensorHubWrite(0x7C, 0x0A, 0x21);
  hasMag_ = qmcEnabled_;
  return qmcEnabled_;
}

bool LSM6DSV::readQMC6309(Vec3& magUT) {
  uint8_t raw[6];
  if (!sensorHubRead(0x7C, 0x01, raw, sizeof(raw))) return false;
  constexpr float scale = 100.0f / 4000.0f;
  magUT = Vec3{le16(&raw[0]) * scale, le16(&raw[2]) * scale,
               le16(&raw[4]) * scale};
  return true;
}

bool LSM6DSV::enableSflp(uint16_t rateHz) {
  uint8_t rateCode;
  if (rateHz <= 15) rateCode = 0;
  else if (rateHz <= 30) rateCode = 1;
  else if (rateHz <= 60) rateCode = 2;
  else if (rateHz <= 120) rateCode = 3;
  else if (rateHz <= 240) rateCode = 4;
  else rateCode = 5;

  if (!setAccelRangeG(4) || !setGyroRangeDps(2000) ||
      !setSampleRateHz(rateHz) || !embeddedBank(true)) return false;
  bool ok = bus_.updateRegister(EMB_FUNC_FIFO_EN_A, SFLP_GAME_ENABLE,
                                SFLP_GAME_ENABLE) == IMUStatus::Ok;
  ok &= bus_.updateRegister(EMB_FUNC_EN_A, SFLP_GAME_ENABLE,
                            SFLP_GAME_ENABLE) == IMUStatus::Ok;
  ok &= bus_.updateRegister(SFLP_ODR, 0x38, rateCode << 3) == IMUStatus::Ok;
  ok &= embeddedBank(false);
  ok &= bus_.updateRegister(FIFO_CTRL4, 0x07, FIFO_STREAM_MODE) == IMUStatus::Ok;
  return ok;
}

bool LSM6DSV::disableSflp() {
  if (!embeddedBank(true)) return false;
  bool ok = bus_.updateRegister(EMB_FUNC_FIFO_EN_A, SFLP_GAME_ENABLE, 0) ==
            IMUStatus::Ok;
  ok &= bus_.updateRegister(EMB_FUNC_EN_A, SFLP_GAME_ENABLE, 0) == IMUStatus::Ok;
  ok &= embeddedBank(false);
  ok &= bus_.updateRegister(FIFO_CTRL4, 0x07, 0) == IMUStatus::Ok;
  return ok;
}

uint16_t LSM6DSV::fifoSamples() {
  uint8_t status[2] = {0, 0};
  if (bus_.readRegisters(FIFO_STATUS1, status, sizeof(status)) != IMUStatus::Ok)
    return 0;
  return static_cast<uint16_t>(status[0]) |
         (static_cast<uint16_t>(status[1] & 0x01) << 8);
}

float LSM6DSV::halfToFloat(uint16_t value) {
  uint32_t sign = static_cast<uint32_t>(value & 0x8000) << 16;
  uint32_t exponent = (value >> 10) & 0x1F;
  uint32_t mantissa = value & 0x03FF;
  uint32_t bits;
  if (exponent == 0) {
    if (mantissa == 0) {
      bits = sign;
    } else {
      int shift = 0;
      while ((mantissa & 0x0400) == 0) {
        mantissa <<= 1;
        ++shift;
      }
      mantissa &= 0x03FF;
      bits = sign | static_cast<uint32_t>(127 - 14 - shift) << 23 |
             mantissa << 13;
    }
  } else if (exponent == 0x1F) {
    bits = sign | 0x7F800000UL | mantissa << 13;
  } else {
    bits = sign | (exponent + 112) << 23 | mantissa << 13;
  }
  union {
    uint32_t u;
    float f;
  } converted = {bits};
  return converted.f;
}

bool LSM6DSV::readSflpQuaternion(SflpQuaternion& quaternion) {
  uint16_t samples = fifoSamples();
  while (samples-- > 0) {
    uint8_t tag = 0;
    uint8_t raw[6];
    if (bus_.readRegister(FIFO_DATA_OUT_TAG, tag) != IMUStatus::Ok ||
        bus_.readRegisters(FIFO_DATA_OUT_X_L, raw, sizeof(raw)) != IMUStatus::Ok)
      return false;
    if ((tag >> 3) != SFLP_GAME_TAG) continue;
    quaternion.x = halfToFloat(static_cast<uint16_t>(raw[0]) |
                               (static_cast<uint16_t>(raw[1]) << 8));
    quaternion.y = halfToFloat(static_cast<uint16_t>(raw[2]) |
                               (static_cast<uint16_t>(raw[3]) << 8));
    quaternion.z = halfToFloat(static_cast<uint16_t>(raw[4]) |
                               (static_cast<uint16_t>(raw[5]) << 8));
    float sum = quaternion.x * quaternion.x +
                quaternion.y * quaternion.y +
                quaternion.z * quaternion.z;
    if (sum > 1.0f) {
      float scale = 1.0f / sqrtf(sum);
      quaternion.x *= scale;
      quaternion.y *= scale;
      quaternion.z *= scale;
      sum = 1.0f;
    }
    quaternion.w = sqrtf(1.0f - sum);
    return true;
  }
  return false;
}

}  // namespace nimu
