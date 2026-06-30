#include "QMI8658.h"

namespace nimu {
using namespace qmi8658;

namespace {
inline int16_t le16(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}
}  // namespace

bool QMI8658::begin() {
  if (beginI2C(Wire, kAddrHigh)) {
    return true;
  }
  return beginI2C(Wire, kAddrLow);
}

bool QMI8658::beginI2C(TwoWire& wire, uint8_t address) {
  bus_.beginI2C(wire, address, clockHz_);
  bus_.recoverBus();
  if (!isConnected()) {
    return false;
  }
  return configureDefaults();
}

bool QMI8658::beginSPI(SPIClass& spi, uint8_t csPin) {
  bus_.beginSPI(spi, csPin, 1000000, 0x80);
  if (!isConnected()) {
    return false;
  }
  return configureDefaults();
}

uint8_t QMI8658::whoAmI() {
  uint8_t id = 0;
  bus_.readRegister(WHO_AM_I, id);
  return id;
}

bool QMI8658::isConnected() {
  return whoAmI() == kWhoAmI;
}

bool QMI8658::configureDefaults() {
  if (bus_.writeRegister(CTRL1, CTRL1_SPI_AI) != IMUStatus::Ok) {
    return false;
  }
  bool ok = true;
  ok &= setAccelRangeG(accelRangeG_);
  ok &= setGyroRangeDps(gyroRangeDps_);
  ok &= setSampleRateHz(sampleRateHz_);
  ok &= bus_.writeRegister(CTRL7, ENABLE_ACCEL | ENABLE_GYRO) == IMUStatus::Ok;
  return ok;
}

bool QMI8658::readRaw(RawSample& out) {
  uint8_t t[2];
  uint8_t a[6];
  uint8_t g[6];
  if (bus_.readRegisters(TEMP_L, t, sizeof(t)) != IMUStatus::Ok ||
      bus_.readRegisters(AX_L, a, sizeof(a)) != IMUStatus::Ok ||
      bus_.readRegisters(GX_L, g, sizeof(g)) != IMUStatus::Ok) {
    return false;
  }
  out.temp = le16(t);
  out.ax = le16(&a[0]);
  out.ay = le16(&a[2]);
  out.az = le16(&a[4]);
  out.gx = le16(&g[0]);
  out.gy = le16(&g[2]);
  out.gz = le16(&g[4]);
  return true;
}

bool QMI8658::update() {
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
  data_.temperature = raw.temp / 256.0f;
  data_.timestamp = micros();
  return true;
}

bool QMI8658::setAccelRangeG(uint16_t maxG) {
  uint8_t range;
  if (maxG <= 2) {
    range = 0x00; accelLsbPerG_ = 16384.0f; accelRangeG_ = 2;
  } else if (maxG <= 4) {
    range = 0x01; accelLsbPerG_ = 8192.0f; accelRangeG_ = 4;
  } else if (maxG <= 8) {
    range = 0x02; accelLsbPerG_ = 4096.0f; accelRangeG_ = 8;
  } else {
    range = 0x03; accelLsbPerG_ = 2048.0f; accelRangeG_ = 16;
  }
  return bus_.writeRegister(CTRL2, (range << 4) | accelOdr_) == IMUStatus::Ok;
}

bool QMI8658::setGyroRangeDps(uint16_t maxDps) {
  uint8_t range;
  if (maxDps <= 32) {
    range = 0x00; gyroLsbPerDps_ = 1024.0f; gyroRangeDps_ = 32;
  } else if (maxDps <= 64) {
    range = 0x01; gyroLsbPerDps_ = 512.0f; gyroRangeDps_ = 64;
  } else if (maxDps <= 128) {
    range = 0x02; gyroLsbPerDps_ = 256.0f; gyroRangeDps_ = 128;
  } else if (maxDps <= 256) {
    range = 0x03; gyroLsbPerDps_ = 128.0f; gyroRangeDps_ = 256;
  } else if (maxDps <= 512) {
    range = 0x04; gyroLsbPerDps_ = 64.0f; gyroRangeDps_ = 512;
  } else if (maxDps <= 1024) {
    range = 0x05; gyroLsbPerDps_ = 32.0f; gyroRangeDps_ = 1024;
  } else if (maxDps <= 2048) {
    range = 0x06; gyroLsbPerDps_ = 16.0f; gyroRangeDps_ = 2048;
  } else {
    range = 0x07; gyroLsbPerDps_ = 8.0f; gyroRangeDps_ = 4096;
  }
  return bus_.writeRegister(CTRL3, (range << 4) | gyroOdr_) == IMUStatus::Ok;
}

uint8_t QMI8658::odrCodeForHz(uint16_t hz, uint16_t& actualHz) const {
  if (hz <= 31) { actualHz = 31; return 0x08; }
  if (hz <= 63) { actualHz = 62; return 0x07; }
  if (hz <= 125) { actualHz = 125; return 0x06; }
  if (hz <= 250) { actualHz = 250; return 0x05; }
  if (hz <= 500) { actualHz = 500; return 0x04; }
  if (hz <= 1000) { actualHz = 1000; return 0x03; }
  if (hz <= 2000) { actualHz = 2000; return 0x02; }
  if (hz <= 4000) { actualHz = 4000; return 0x01; }
  actualHz = 8000;
  return 0x00;
}

bool QMI8658::setSampleRateHz(uint16_t hz) {
  if (hz == 0) {
    return false;
  }
  uint16_t actual = 0;
  accelOdr_ = odrCodeForHz(hz, actual);
  gyroOdr_ = accelOdr_;
  sampleRateHz_ = actual;
  bool ok = true;
  ok &= setAccelRangeG(accelRangeG_);
  ok &= setGyroRangeDps(gyroRangeDps_);
  return ok;
}

bool QMI8658::setLowPassFilterHz(uint16_t hz) {
  (void)hz;
  return true;
}

bool QMI8658::dataReady() {
  uint8_t status = 0;
  if (bus_.readRegister(STATUS0, status) != IMUStatus::Ok) {
    return false;
  }
  return (status & STATUS_DATA_READY) != 0;
}

bool QMI8658::setSynchronizedSampleMode(bool enable) {
  return bus_.updateRegister(CTRL7, CTRL7_SYNC_SAMPLE,
                             enable ? CTRL7_SYNC_SAMPLE : 0) == IMUStatus::Ok;
}

uint8_t QMI8658::interruptStatus() {
  uint8_t status = 0;
  bus_.readRegister(STATUS_INT, status);
  return status;
}

void QMI8658::setBusClockHz(uint32_t hz) {
  clockHz_ = hz;
  bus_.setClockHz(hz);
}

}  // namespace nimu
