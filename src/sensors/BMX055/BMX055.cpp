#include "BMX055.h"

namespace nimu {
namespace {
constexpr uint8_t BMA_CHIP_ID = 0x00;
constexpr uint8_t BMA_DATA = 0x02;
constexpr uint8_t BMA_TEMP = 0x08;
constexpr uint8_t BMA_STATUS = 0x0A;
constexpr uint8_t BMA_RANGE = 0x0F;
constexpr uint8_t BMA_BW = 0x10;
constexpr uint8_t BMA_RESET = 0x14;
constexpr uint8_t BMA_ID = 0xFB;

constexpr uint8_t BMG_CHIP_ID = 0x00;
constexpr uint8_t BMG_DATA = 0x02;
constexpr uint8_t BMG_STATUS = 0x0A;
constexpr uint8_t BMG_RANGE = 0x0F;
constexpr uint8_t BMG_BW = 0x10;
constexpr uint8_t BMG_RESET = 0x14;
constexpr uint8_t BMG_ID = 0x0F;

constexpr uint8_t BMM_CHIP_ID = 0x40;
constexpr uint8_t BMM_DATA = 0x42;
constexpr uint8_t BMM_POWER = 0x4B;
constexpr uint8_t BMM_OPMODE = 0x4C;
constexpr uint8_t BMM_REP_XY = 0x51;
constexpr uint8_t BMM_REP_Z = 0x52;
constexpr uint8_t BMM_ID = 0x32;

int16_t le16(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}
uint16_t ule16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0]) |
         (static_cast<uint16_t>(p[1]) << 8);
}
}  // namespace

bool BMX055::begin() {
  return beginI2C(Wire, kAccelAddr, kGyroAddr, kMagAddr);
}

bool BMX055::beginI2C(TwoWire& wire, uint8_t accelAddress) {
  return beginI2C(wire, accelAddress, kGyroAddr, kMagAddr);
}

bool BMX055::beginI2C(TwoWire& wire, uint8_t accelAddress,
                      uint8_t gyroAddress, uint8_t magAddress) {
  accelBus_.beginI2C(wire, accelAddress, 400000);
  gyroBus_.beginI2C(wire, gyroAddress, 400000);
  magBus_.beginI2C(wire, magAddress, 400000);
  if (!isConnected()) return false;
  return configureDefaults();
}

bool BMX055::beginSPI(SPIClass& spi, uint8_t csPin) {
  (void)spi;
  (void)csPin;
  return false;  // BMX055 is three chips and cannot use a one-CS signature.
}

bool BMX055::beginSPI(SPIClass& spi, uint8_t accelCsPin,
                      uint8_t gyroCsPin, uint8_t magCsPin) {
  // The BMA280 accelerometer consumes one dummy byte after its SPI read
  // address. The BMG160 and BMM150 return data immediately.
  accelBus_.beginSPI(spi, accelCsPin, 1000000, 0x80, 1);
  gyroBus_.beginSPI(spi, gyroCsPin, 1000000, 0x80);
  magBus_.beginSPI(spi, magCsPin, 1000000, 0x80);
  if (!isConnected()) return false;
  return configureDefaults();
}

uint8_t BMX055::whoAmI() {
  uint8_t id = 0;
  accelBus_.readRegister(BMA_CHIP_ID, id);
  return id;
}

bool BMX055::accelConnected() { return whoAmI() == BMA_ID; }
bool BMX055::gyroConnected() {
  uint8_t id = 0;
  return gyroBus_.readRegister(BMG_CHIP_ID, id) == IMUStatus::Ok && id == BMG_ID;
}
bool BMX055::magConnected() {
  magBus_.writeRegister(BMM_POWER, 0x01);
  delay(3);
  uint8_t id = 0;
  return magBus_.readRegister(BMM_CHIP_ID, id) == IMUStatus::Ok && id == BMM_ID;
}
bool BMX055::isConnected() {
  return accelConnected() && gyroConnected() && magConnected();
}

bool BMX055::configureDefaults() {
  if (accelBus_.writeRegister(BMA_RESET, 0xB6) != IMUStatus::Ok ||
      gyroBus_.writeRegister(BMG_RESET, 0xB6) != IMUStatus::Ok) return false;
  delay(30);
  bool ok = setAccelRangeG(accelRangeG_) && setGyroRangeDps(gyroRangeDps_);
  ok &= setSampleRateHz(100);
  ok &= magBus_.writeRegister(BMM_POWER, 0x01) == IMUStatus::Ok;
  delay(3);
  ok &= magBus_.writeRegister(BMM_REP_XY, 0x17) == IMUStatus::Ok;
  ok &= magBus_.writeRegister(BMM_REP_Z, 0x52) == IMUStatus::Ok;
  ok &= magBus_.writeRegister(BMM_OPMODE, 0x00) == IMUStatus::Ok;
  return ok && readMagTrim();
}

bool BMX055::readMagTrim() {
  uint8_t a[2], b[4], c[10];
  if (magBus_.readRegisters(0x5D, a, sizeof(a)) != IMUStatus::Ok ||
      magBus_.readRegisters(0x62, b, sizeof(b)) != IMUStatus::Ok ||
      magBus_.readRegisters(0x68, c, sizeof(c)) != IMUStatus::Ok) return false;
  trim_.x1 = static_cast<int8_t>(a[0]);
  trim_.y1 = static_cast<int8_t>(a[1]);
  trim_.z4 = le16(&b[0]);
  trim_.x2 = static_cast<int8_t>(b[2]);
  trim_.y2 = static_cast<int8_t>(b[3]);
  trim_.z2 = le16(&c[0]);
  trim_.z1 = ule16(&c[2]);
  trim_.xyz1 = ule16(&c[4]) & 0x7FFF;
  trim_.z3 = le16(&c[6]);
  trim_.xy2 = static_cast<int8_t>(c[8]);
  trim_.xy1 = c[9];
  return true;
}

bool BMX055::readRaw(RawSample& out) {
  uint8_t a[6], g[6], m[8], t = 0;
  if (accelBus_.readRegisters(BMA_DATA, a, sizeof(a)) != IMUStatus::Ok ||
      accelBus_.readRegister(BMA_TEMP, t) != IMUStatus::Ok ||
      gyroBus_.readRegisters(BMG_DATA, g, sizeof(g)) != IMUStatus::Ok ||
      magBus_.readRegisters(BMM_DATA, m, sizeof(m)) != IMUStatus::Ok)
    return false;
  out.ax = le16(&a[0]) >> 2; out.ay = le16(&a[2]) >> 2;
  out.az = le16(&a[4]) >> 2;
  out.gx = le16(&g[0]); out.gy = le16(&g[2]); out.gz = le16(&g[4]);
  out.mx = le16(&m[0]) >> 3; out.my = le16(&m[2]) >> 3;
  out.mz = le16(&m[4]) >> 1; out.rhall = ule16(&m[6]) >> 2;
  out.temp = static_cast<int8_t>(t);
  return true;
}

float BMX055::compensateMagX(int16_t raw, uint16_t rhall) const {
  if (raw == -4096 || rhall == 0 || trim_.xyz1 == 0) return 0.0f;
  float x = (static_cast<float>(trim_.xyz1) * 16384.0f / rhall) - 16384.0f;
  float gain = ((trim_.xy2 * (x * x / 268435456.0f)) +
                (x * trim_.xy1 / 16384.0f) + 256.0f) *
               (trim_.x2 + 160.0f);
  return ((raw * gain) / 8192.0f + trim_.x1 * 8.0f) / 16.0f;
}

float BMX055::compensateMagY(int16_t raw, uint16_t rhall) const {
  if (raw == -4096 || rhall == 0 || trim_.xyz1 == 0) return 0.0f;
  float x = (static_cast<float>(trim_.xyz1) * 16384.0f / rhall) - 16384.0f;
  float gain = ((trim_.xy2 * (x * x / 268435456.0f)) +
                (x * trim_.xy1 / 16384.0f) + 256.0f) *
               (trim_.y2 + 160.0f);
  return ((raw * gain) / 8192.0f + trim_.y1 * 8.0f) / 16.0f;
}

float BMX055::compensateMagZ(int16_t raw, uint16_t rhall) const {
  if (raw == -16384 || trim_.z2 == 0 || trim_.z1 == 0 || rhall == 0)
    return 0.0f;
  float numerator = (raw - trim_.z4) * 131072.0f -
                    trim_.z3 * (static_cast<float>(rhall) - trim_.xyz1);
  float denominator = (trim_.z2 +
                       trim_.z1 * static_cast<float>(rhall) / 32768.0f) * 4.0f;
  return (numerator / denominator) / 16.0f;
}

bool BMX055::update() {
  RawSample raw;
  if (!readRaw(raw)) return false;
  data_.accel = correct(Vec3{raw.ax / accelLsbPerG_, raw.ay / accelLsbPerG_,
                              raw.az / accelLsbPerG_},
                        cal_.accelBias, cal_.accelScale);
  data_.gyro = correct(Vec3{raw.gx / gyroLsbPerDps_, raw.gy / gyroLsbPerDps_,
                             raw.gz / gyroLsbPerDps_},
                       cal_.gyroBias, Vec3{1, 1, 1});
  data_.mag = correct(Vec3{compensateMagX(raw.mx, raw.rhall),
                            compensateMagY(raw.my, raw.rhall),
                            compensateMagZ(raw.mz, raw.rhall)},
                      cal_.magBias, cal_.magScale);
  data_.temperature = 23.0f + raw.temp * 0.5f;
  data_.timestamp = micros();
  return true;
}

bool BMX055::setAccelRangeG(uint16_t maxG) {
  uint8_t value;
  if (maxG <= 2) { value = 0x03; accelLsbPerG_ = 4096; accelRangeG_ = 2; }
  else if (maxG <= 4) { value = 0x05; accelLsbPerG_ = 2048; accelRangeG_ = 4; }
  else if (maxG <= 8) { value = 0x08; accelLsbPerG_ = 1024; accelRangeG_ = 8; }
  else { value = 0x0C; accelLsbPerG_ = 512; accelRangeG_ = 16; }
  return accelBus_.writeRegister(BMA_RANGE, value) == IMUStatus::Ok;
}

bool BMX055::setGyroRangeDps(uint16_t maxDps) {
  uint8_t value;
  if (maxDps <= 125) { value = 0x04; gyroLsbPerDps_ = 262.4f; gyroRangeDps_ = 125; }
  else if (maxDps <= 250) { value = 0x03; gyroLsbPerDps_ = 131.2f; gyroRangeDps_ = 250; }
  else if (maxDps <= 500) { value = 0x02; gyroLsbPerDps_ = 65.6f; gyroRangeDps_ = 500; }
  else if (maxDps <= 1000) { value = 0x01; gyroLsbPerDps_ = 32.8f; gyroRangeDps_ = 1000; }
  else { value = 0x00; gyroLsbPerDps_ = 16.4f; gyroRangeDps_ = 2000; }
  return gyroBus_.writeRegister(BMG_RANGE, value) == IMUStatus::Ok;
}

bool BMX055::setSampleRateHz(uint16_t hz) {
  if (hz == 0) return false;
  uint8_t accelBw = hz <= 16 ? 0x08 : hz <= 32 ? 0x09 :
                    hz <= 63 ? 0x0A : hz <= 125 ? 0x0B : 0x0C;
  uint8_t gyroBw = hz <= 100 ? 0x07 : hz <= 200 ? 0x06 : 0x02;
  return accelBus_.writeRegister(BMA_BW, accelBw) == IMUStatus::Ok &&
         gyroBus_.writeRegister(BMG_BW, gyroBw) == IMUStatus::Ok;
}

bool BMX055::setLowPassFilterHz(uint16_t hz) { return setSampleRateHz(hz * 2); }

bool BMX055::dataReady() {
  uint8_t a = 0, g = 0;
  return accelBus_.readRegister(BMA_STATUS, a) == IMUStatus::Ok &&
         gyroBus_.readRegister(BMG_STATUS, g) == IMUStatus::Ok &&
         (a & 0x80) != 0 && (g & 0x80) != 0;
}

}  // namespace nimu
