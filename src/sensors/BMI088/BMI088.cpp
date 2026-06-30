#include "BMI088.h"

namespace nimu {
using namespace bmi088;

namespace {
inline int16_t le16(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}
}  // namespace

bool BMI088::begin() {
  const uint8_t accelAddresses[] = {kAccelAddrLow, kAccelAddrHigh};
  const uint8_t gyroAddresses[] = {kGyroAddrLow, kGyroAddrHigh};
  for (uint8_t a : accelAddresses) {
    for (uint8_t g : gyroAddresses) {
      if (beginI2C(Wire, a, g)) return true;
    }
  }
  return false;
}

bool BMI088::beginI2C(TwoWire& wire, uint8_t address) {
  uint8_t gyroAddress = (address == kAccelAddrHigh) ? kGyroAddrHigh : kGyroAddrLow;
  return beginI2C(wire, address, gyroAddress);
}

bool BMI088::beginI2C(TwoWire& wire, uint8_t accelAddress,
                      uint8_t gyroAddress) {
  bus_.beginI2C(wire, accelAddress, 400000);
  gyroBus_.beginI2C(wire, gyroAddress, 400000);
  if (!isConnected()) return false;
  return reset() && configureDefaults();
}

bool BMI088::beginSPI(SPIClass& spi, uint8_t csPin) {
  (void)spi;
  (void)csPin;
  return false;
}

bool BMI088::beginSPI(SPIClass& spi, uint8_t accelCsPin, uint8_t gyroCsPin) {
  bus_.beginSPI(spi, accelCsPin, 1000000, 0x80, 1);
  gyroBus_.beginSPI(spi, gyroCsPin, 1000000, 0x80);
  if (!isConnected()) return false;
  return reset() && configureDefaults();
}

uint8_t BMI088::whoAmI() {
  uint8_t id = 0;
  bus_.readRegister(ACC_CHIP_ID, id);
  return id;
}

uint8_t BMI088::gyroWhoAmI() {
  uint8_t id = 0;
  gyroBus_.readRegister(GYR_CHIP_ID, id);
  return id;
}

bool BMI088::isConnected() {
  return whoAmI() == kAccelId && gyroWhoAmI() == kGyroId;
}

bool BMI088::reset() {
  bool ok = bus_.writeRegister(ACC_SOFT_RESET, SOFT_RESET) == IMUStatus::Ok;
  ok &= gyroBus_.writeRegister(GYR_SOFT_RESET, SOFT_RESET) == IMUStatus::Ok;
  delay(30);
  return ok && isConnected();
}

bool BMI088::configureDefaults() {
  bool ok = bus_.writeRegister(ACC_PWR_CONF, 0x00) == IMUStatus::Ok;
  delay(5);
  ok &= bus_.writeRegister(ACC_PWR_CTRL, 0x04) == IMUStatus::Ok;
  ok &= gyroBus_.writeRegister(GYR_POWER, 0x00) == IMUStatus::Ok;
  delay(50);
  ok &= setAccelRangeG(accelRangeG_);
  ok &= setGyroRangeDps(gyroRangeDps_);
  ok &= setSampleRateHz(sampleRateHz_);
  return ok;
}

bool BMI088::readRaw(RawSample& out) {
  uint8_t accel[6], gyro[6], temp[2];
  if (bus_.readRegisters(ACC_DATA, accel, sizeof(accel)) != IMUStatus::Ok ||
      gyroBus_.readRegisters(GYR_DATA, gyro, sizeof(gyro)) != IMUStatus::Ok ||
      bus_.readRegisters(ACC_TEMP, temp, sizeof(temp)) != IMUStatus::Ok) {
    return false;
  }
  out.ax = le16(&accel[0]); out.ay = le16(&accel[2]); out.az = le16(&accel[4]);
  out.gx = le16(&gyro[0]); out.gy = le16(&gyro[2]); out.gz = le16(&gyro[4]);
  int16_t t = static_cast<int16_t>((static_cast<uint16_t>(temp[0]) << 3) |
                                   (temp[1] >> 5));
  if (t > 1023) t -= 2048;
  out.temp = t;
  return true;
}

bool BMI088::update() {
  RawSample raw;
  if (!readRaw(raw)) return false;
  data_.accel = correct(Vec3{raw.ax / accelLsbPerG_, raw.ay / accelLsbPerG_,
                             raw.az / accelLsbPerG_},
                        cal_.accelBias, cal_.accelScale);
  data_.gyro = correct(Vec3{raw.gx / gyroLsbPerDps_, raw.gy / gyroLsbPerDps_,
                            raw.gz / gyroLsbPerDps_},
                       cal_.gyroBias, Vec3{1, 1, 1});
  data_.mag = Vec3{0, 0, 0};
  data_.temperature = raw.temp * 0.125f + 23.0f;
  data_.timestamp = micros();
  return true;
}

bool BMI088::setAccelRangeG(uint16_t maxG) {
  uint8_t code;
  if (maxG <= 3) { code = 0; accelRangeG_ = 3; accelLsbPerG_ = 10920.0f; }
  else if (maxG <= 6) { code = 1; accelRangeG_ = 6; accelLsbPerG_ = 5460.0f; }
  else if (maxG <= 12) { code = 2; accelRangeG_ = 12; accelLsbPerG_ = 2730.0f; }
  else { code = 3; accelRangeG_ = 24; accelLsbPerG_ = 1365.0f; }
  return bus_.writeRegister(ACC_RANGE, code) == IMUStatus::Ok;
}

bool BMI088::setGyroRangeDps(uint16_t maxDps) {
  uint8_t code;
  if (maxDps <= 125) { code = 4; gyroRangeDps_ = 125; gyroLsbPerDps_ = 262.144f; }
  else if (maxDps <= 250) { code = 3; gyroRangeDps_ = 250; gyroLsbPerDps_ = 131.072f; }
  else if (maxDps <= 500) { code = 2; gyroRangeDps_ = 500; gyroLsbPerDps_ = 65.536f; }
  else if (maxDps <= 1000) { code = 1; gyroRangeDps_ = 1000; gyroLsbPerDps_ = 32.768f; }
  else { code = 0; gyroRangeDps_ = 2000; gyroLsbPerDps_ = 16.384f; }
  return gyroBus_.writeRegister(GYR_RANGE, code) == IMUStatus::Ok;
}

uint8_t BMI088::accelOdrForHz(uint16_t hz, uint16_t& actualHz) const {
  if (hz <= 13) { actualHz = 12; return 0x05; }
  if (hz <= 25) { actualHz = 25; return 0x06; }
  if (hz <= 50) { actualHz = 50; return 0x07; }
  if (hz <= 100) { actualHz = 100; return 0x08; }
  if (hz <= 200) { actualHz = 200; return 0x09; }
  if (hz <= 400) { actualHz = 400; return 0x0A; }
  if (hz <= 800) { actualHz = 800; return 0x0B; }
  actualHz = 1600; return 0x0C;
}

uint8_t BMI088::gyroBandwidthForHz(uint16_t hz, uint16_t& actualHz) const {
  if (hz <= 100) { actualHz = 100; return 0x07; }
  if (hz <= 200) { actualHz = 200; return 0x06; }
  if (hz <= 400) { actualHz = 400; return 0x03; }
  if (hz <= 1000) { actualHz = 1000; return 0x02; }
  actualHz = 2000; return 0x01;
}

bool BMI088::setSampleRateHz(uint16_t hz) {
  if (hz == 0) return false;
  uint16_t accelHz, gyroHz;
  accelOdr_ = accelOdrForHz(hz, accelHz);
  gyroBandwidth_ = gyroBandwidthForHz(hz, gyroHz);
  sampleRateHz_ = (accelHz < gyroHz) ? accelHz : gyroHz;
  bool ok = bus_.writeRegister(ACC_CONF, 0xA0 | accelOdr_) == IMUStatus::Ok;
  ok &= gyroBus_.writeRegister(GYR_BANDWIDTH, gyroBandwidth_) == IMUStatus::Ok;
  return ok;
}

bool BMI088::setLowPassFilterHz(uint16_t hz) {
  return setSampleRateHz(hz == 0 ? sampleRateHz_ : hz * 2);
}

bool BMI088::dataReady() {
  uint8_t accelStatus = 0, gyroStatus = 0;
  if (bus_.readRegister(ACC_STATUS, accelStatus) != IMUStatus::Ok ||
      gyroBus_.readRegister(GYR_INT_STATUS, gyroStatus) != IMUStatus::Ok) {
    return false;
  }
  return (accelStatus & 0x80) && (gyroStatus & 0x80);
}

bool BMI088::configureAccelInterruptPin(uint8_t pin, bool activeHigh,
                                        bool openDrain) {
  uint8_t reg = pin == 1 ? ACC_INT1_IO_CTRL : pin == 2 ? ACC_INT2_IO_CTRL : 0;
  if (reg == 0) return false;
  uint8_t value = 0x08 | (openDrain ? 0x04 : 0) |
                  (activeHigh ? 0x02 : 0);
  return bus_.writeRegister(reg, value) == IMUStatus::Ok;
}

bool BMI088::routeAccelDataReadyInterrupt(uint8_t pin, bool enable) {
  uint8_t mask = pin == 1 ? 0x04 : pin == 2 ? 0x40 : 0;
  return mask != 0 && bus_.updateRegister(ACC_INT_MAP_DATA, mask,
                                          enable ? mask : 0) == IMUStatus::Ok;
}

bool BMI088::configureGyroInterruptPin(uint8_t pin, bool activeHigh,
                                       bool openDrain) {
  if (pin != 3 && pin != 4) return false;
  uint8_t shift = pin == 3 ? 0 : 2;
  uint8_t mask = static_cast<uint8_t>(0x03 << shift);
  uint8_t value = static_cast<uint8_t>(((openDrain ? 1 : 0) << 1 |
                                        (activeHigh ? 1 : 0)) << shift);
  return gyroBus_.updateRegister(GYR_INT3_INT4_IO_CONF, mask, value) ==
         IMUStatus::Ok;
}

bool BMI088::routeGyroDataReadyInterrupt(uint8_t pin, bool enable) {
  uint8_t mask = pin == 3 ? 0x01 : pin == 4 ? 0x80 : 0;
  if (mask == 0) return false;
  bool ok = gyroBus_.updateRegister(GYR_INT3_INT4_IO_MAP, mask,
                                     enable ? mask : 0) == IMUStatus::Ok;
  ok &= gyroBus_.updateRegister(GYR_INT_CTRL, 0x80,
                                enable ? 0x80 : 0) == IMUStatus::Ok;
  return ok;
}

}  // namespace nimu
