#include "BNO08x.h"

/*
  SHTP packet flow follows CEVA's SH-2 documentation and the MIT-licensed
  SparkFun BNO080 Arduino library by Nathan Seidle, substantially rewritten
  for ArduinoNRF-IMU's unified interface and bounded buffers.
*/

namespace nimu {
using namespace bno08x;

int16_t BNO08x::le16(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}

uint32_t BNO08x::le32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

float BNO08x::qToFloat(int16_t value, uint8_t qPoint) {
  return static_cast<float>(value) / static_cast<float>(1UL << qPoint);
}

bool BNO08x::begin() {
  if (beginI2C(Wire, kAddrDefault)) return true;
  return beginI2C(Wire, kAddrAlternate);
}

bool BNO08x::beginI2C(TwoWire& wire, uint8_t address) {
  wire_ = &wire;
  address_ = address;
  productValid_ = false;
  resetSeen_ = false;
  for (uint8_t& sequence : txSequence_) sequence = 0;
  wire_->begin();
  wire_->setClock(400000);

  if (resetPin_ >= 0 && !hardwareReset()) return false;
  if (!softReset() || !requestProductId()) return false;
  bool ok = true;
  ok &= enableReport(SENSOR_ACCELEROMETER, reportIntervalUs_);
  ok &= enableReport(SENSOR_GYROSCOPE, reportIntervalUs_);
  ok &= enableReport(SENSOR_MAGNETIC_FIELD, reportIntervalUs_);
  ok &= enableReport(SENSOR_ROTATION_VECTOR, reportIntervalUs_);
  return ok;
}

void BNO08x::configurePins(int8_t interruptPin, int8_t resetPin,
                           int8_t wakePin) {
  interruptPin_ = interruptPin;
  resetPin_ = resetPin;
  wakePin_ = wakePin;
  if (interruptPin_ >= 0) pinMode(interruptPin_, INPUT_PULLUP);
  if (wakePin_ >= 0) {
    pinMode(wakePin_, OUTPUT);
    digitalWrite(wakePin_, LOW);  // WAKE is active low.
  }
  if (resetPin_ >= 0) {
    pinMode(resetPin_, OUTPUT);
    digitalWrite(resetPin_, HIGH);  // RST is active low.
  }
}

bool BNO08x::hardwareReset(uint16_t bootDelayMs) {
  if (resetPin_ < 0) return false;
  if (wakePin_ >= 0) digitalWrite(wakePin_, LOW);
  digitalWrite(resetPin_, LOW);
  delay(10);
  digitalWrite(resetPin_, HIGH);
  delay(bootDelayMs);
  productValid_ = false;
  resetSeen_ = true;
  return true;
}

bool BNO08x::interruptAsserted() const {
  return interruptPin_ >= 0 && digitalRead(interruptPin_) == LOW;
}

void BNO08x::setAwake(bool awake) {
  if (wakePin_ >= 0) digitalWrite(wakePin_, awake ? LOW : HIGH);
}

bool BNO08x::beginSPI(SPIClass& spi, uint8_t csPin) {
  (void)spi;
  (void)csPin;
  return false;  // BNO08x SPI also requires INT, RST and WAKE/PS0 pins.
}

bool BNO08x::softReset() {
  const uint8_t reset = 1;
  if (!sendPacket(CHANNEL_EXECUTABLE, &reset, 1)) return false;
  delay(100);
  for (uint8_t i = 0; i < 8; ++i) {
    if (!receivePacket()) break;
  }
  delay(20);
  return true;
}

bool BNO08x::sendPacket(uint8_t channel, const uint8_t* payload,
                        uint8_t length) {
  if (wire_ == nullptr || channel >= 6 || length > 28) return false;
  const uint16_t packetLength = static_cast<uint16_t>(length) + 4;
  wire_->beginTransmission(address_);
  wire_->write(static_cast<uint8_t>(packetLength));
  wire_->write(static_cast<uint8_t>(packetLength >> 8));
  wire_->write(channel);
  wire_->write(txSequence_[channel]++);
  if (length != 0) wire_->write(payload, length);
  return wire_->endTransmission() == 0;
}

bool BNO08x::readPayload(uint16_t length) {
  uint16_t offset = 0;
  while (offset < length) {
    uint8_t chunk = static_cast<uint8_t>(length - offset);
    if (chunk > 28) chunk = 28;
    uint8_t requested = static_cast<uint8_t>(chunk + 4);
    if (wire_->requestFrom(address_, requested, true) != requested) return false;
    for (uint8_t i = 0; i < 4; ++i) (void)wire_->read();
    for (uint8_t i = 0; i < chunk; ++i) {
      uint8_t value = static_cast<uint8_t>(wire_->read());
      if (offset + i < kPacketCapacity) payload_[offset + i] = value;
    }
    offset += chunk;
  }
  payloadLength_ = length < kPacketCapacity ? length : kPacketCapacity;
  return true;
}

bool BNO08x::receivePacket() {
  if (wire_ == nullptr) return false;
  if (wire_->requestFrom(address_, static_cast<uint8_t>(4), true) != 4) return false;
  for (uint8_t i = 0; i < 4; ++i) header_[i] = static_cast<uint8_t>(wire_->read());
  uint16_t packetLength = (static_cast<uint16_t>(header_[1]) << 8) | header_[0];
  packetLength &= 0x7FFF;
  if (packetLength < 4 || packetLength > 512) return false;
  if (!readPayload(packetLength - 4)) return false;
  if (header_[2] == CHANNEL_EXECUTABLE && payloadLength_ > 0 && payload_[0] == 1) {
    resetSeen_ = true;
  }
  return true;
}

bool BNO08x::requestProductId(uint16_t timeoutMs) {
  const uint8_t request[2] = {REPORT_PRODUCT_ID_REQUEST, 0};
  if (!sendPacket(CHANNEL_CONTROL, request, sizeof(request))) return false;
  uint32_t started = millis();
  while (millis() - started < timeoutMs) {
    if (!receivePacket()) {
      delay(2);
      continue;
    }
    if (header_[2] != CHANNEL_CONTROL || payloadLength_ < 14 ||
        payload_[0] != REPORT_PRODUCT_ID_RESPONSE) {
      continue;
    }
    product_.resetReason = payload_[1];
    product_.versionMajor = payload_[2];
    product_.versionMinor = payload_[3];
    product_.partNumber = le32(&payload_[4]);
    product_.buildNumber = le32(&payload_[8]);
    product_.versionPatch = static_cast<uint16_t>(payload_[12]) |
                            (static_cast<uint16_t>(payload_[13]) << 8);
    productValid_ = true;
    return true;
  }
  return false;
}

uint8_t BNO08x::whoAmI() {
  if (!productValid_) requestProductId(150);
  return productValid_ ? 0x08 : 0x00;
}

bool BNO08x::isConnected() { return requestProductId(250); }

bool BNO08x::enableReport(uint8_t reportId, uint32_t intervalUs) {
  uint8_t command[17] = {0};
  command[0] = REPORT_SET_FEATURE;
  command[1] = reportId;
  command[5] = static_cast<uint8_t>(intervalUs);
  command[6] = static_cast<uint8_t>(intervalUs >> 8);
  command[7] = static_cast<uint8_t>(intervalUs >> 16);
  command[8] = static_cast<uint8_t>(intervalUs >> 24);
  return sendPacket(CHANNEL_CONTROL, command, sizeof(command));
}

bool BNO08x::parseSensorReport(const uint8_t* report, size_t length) {
  if (length < 4) return false;
  uint8_t id = report[0];
  uint8_t accuracy = report[2] & 0x03;

  if (id == SENSOR_STABILITY_CLASSIFIER && length >= 5) {
    stabilityClass_ = report[4];
    data_.timestamp = micros();
    return true;
  }
  if (id == SENSOR_PERSONAL_ACTIVITY_CLASSIFIER && length >= 14) {
    activityClass_ = report[4];
    for (uint8_t i = 0; i < 9; ++i) activityConfidence_[i] = report[5 + i];
    data_.timestamp = micros();
    return true;
  }
  if (id == SENSOR_TAP_DETECTOR && length >= 5) {
    tapCode_ = report[4];
    data_.timestamp = micros();
    return true;
  }
  if (length < 10) return false;
  int16_t x = le16(&report[4]);
  int16_t y = le16(&report[6]);
  int16_t z = le16(&report[8]);

  if (id == SENSOR_ACCELEROMETER) {
    Vec3 value{qToFloat(x, 8) / kGravityMs2,
               qToFloat(y, 8) / kGravityMs2,
               qToFloat(z, 8) / kGravityMs2};
    data_.accel = correct(value, cal_.accelBias, cal_.accelScale);
    accelAccuracy_ = accuracy;
  } else if (id == SENSOR_GYROSCOPE) {
    constexpr float kRadToDeg = 57.29577951308232f;
    Vec3 value{qToFloat(x, 9) * kRadToDeg,
               qToFloat(y, 9) * kRadToDeg,
               qToFloat(z, 9) * kRadToDeg};
    data_.gyro = correct(value, cal_.gyroBias, Vec3{1, 1, 1});
    gyroAccuracy_ = accuracy;
  } else if (id == SENSOR_MAGNETIC_FIELD) {
    Vec3 value{qToFloat(x, 4), qToFloat(y, 4), qToFloat(z, 4)};
    data_.mag = correct(value, cal_.magBias, cal_.magScale);
    magAccuracy_ = accuracy;
  } else if (id == SENSOR_LINEAR_ACCELERATION) {
    linearAccelMs2_ = Vec3{qToFloat(x, 8), qToFloat(y, 8), qToFloat(z, 8)};
  } else if (id == SENSOR_GRAVITY) {
    gravityMs2_ = Vec3{qToFloat(x, 8), qToFloat(y, 8), qToFloat(z, 8)};
  } else if (id == SENSOR_ROTATION_VECTOR || id == SENSOR_GAME_ROTATION_VECTOR ||
             id == SENSOR_GEOMAGNETIC_ROTATION_VECTOR) {
    if (length < 12) return false;
    Quaternion* q = &quaternion_;
    if (id == SENSOR_GAME_ROTATION_VECTOR) q = &gameQuaternion_;
    if (id == SENSOR_GEOMAGNETIC_ROTATION_VECTOR) q = &geomagneticQuaternion_;
    q->x = qToFloat(x, 14);
    q->y = qToFloat(y, 14);
    q->z = qToFloat(z, 14);
    q->w = qToFloat(le16(&report[10]), 14);
    q->accuracy = accuracy;
    if (id == SENSOR_ROTATION_VECTOR && length >= 14) {
      q->accuracyRad = qToFloat(le16(&report[12]), 12);
    }
  } else if (id == SENSOR_STEP_COUNTER) {
    stepCount_ = static_cast<uint16_t>(z);
  } else {
    return false;
  }
  data_.timestamp = micros();
  return true;
}

bool BNO08x::parsePacket() {
  if (header_[2] == CHANNEL_REPORTS && payloadLength_ >= 15 &&
      payload_[0] == REPORT_BASE_TIMESTAMP) {
    return parseSensorReport(&payload_[5], payloadLength_ - 5);
  }
  return false;
}

bool BNO08x::update() {
  bool parsed = false;
  for (uint8_t i = 0; i < 8; ++i) {
    if (!receivePacket()) break;
    parsed |= parsePacket();
  }
  return parsed;
}

bool BNO08x::setAccelRangeG(uint16_t maxG) { return maxG <= 8; }
bool BNO08x::setGyroRangeDps(uint16_t maxDps) { return maxDps <= 2000; }
bool BNO08x::setLowPassFilterHz(uint16_t hz) {
  (void)hz;
  return true;  // SH-2 owns filtering for calibrated reports.
}

bool BNO08x::setSampleRateHz(uint16_t hz) {
  // This unified snapshot enables accel, gyro, mag and rotation together;
  // 100 Hz is the highest common requested rate because of the magnetometer.
  if (hz == 0 || hz > 100) return false;
  reportIntervalUs_ = 1000000UL / hz;
  bool ok = true;
  ok &= enableReport(SENSOR_ACCELEROMETER, reportIntervalUs_);
  ok &= enableReport(SENSOR_GYROSCOPE, reportIntervalUs_);
  ok &= enableReport(SENSOR_MAGNETIC_FIELD, reportIntervalUs_);
  ok &= enableReport(SENSOR_ROTATION_VECTOR, reportIntervalUs_);
  return ok;
}

bool BNO08x::sendTare(uint8_t subcommand, uint8_t axes) {
  uint8_t request[12] = {0};
  request[0] = REPORT_COMMAND_REQUEST;
  request[1] = commandSequence_++;
  request[2] = COMMAND_TARE;
  request[3] = subcommand;
  request[4] = axes;
  request[5] = 0;  // Rotation-vector basis.
  return sendPacket(CHANNEL_CONTROL, request, sizeof(request));
}

bool BNO08x::tareNow(bool zAxisOnly) {
  return sendTare(0, zAxisOnly ? 0x04 : 0x07);
}

bool BNO08x::saveTare() { return sendTare(1, 0x07); }

bool BNO08x::sendCommand(uint8_t command, const uint8_t* parameters,
                         uint8_t parameterCount, bool waitForResponse) {
  if (parameterCount > 9) return false;
  uint8_t request[12] = {0};
  request[0] = REPORT_COMMAND_REQUEST;
  request[1] = commandSequence_++;
  request[2] = command;
  for (uint8_t i = 0; i < parameterCount; ++i) request[3 + i] = parameters[i];
  lastCommandStatus_ = 0xFF;
  if (!sendPacket(CHANNEL_CONTROL, request, sizeof(request))) return false;
  return !waitForResponse || waitForCommandResponse(command);
}

bool BNO08x::waitForCommandResponse(uint8_t command, uint16_t timeoutMs) {
  uint32_t started = millis();
  while (millis() - started < timeoutMs) {
    if (!receivePacket()) {
      delay(2);
      continue;
    }
    if (header_[2] == CHANNEL_CONTROL && payloadLength_ >= 6 &&
        payload_[0] == REPORT_COMMAND_RESPONSE && payload_[2] == command) {
      lastCommandStatus_ = payload_[5];
      return lastCommandStatus_ == 0;
    }
    parsePacket();
  }
  return false;
}

bool BNO08x::beginCalibration(bool accel, bool gyro, bool mag,
                              bool planarAccel, bool onTable) {
  uint8_t parameters[5] = {static_cast<uint8_t>(accel),
                           static_cast<uint8_t>(gyro),
                           static_cast<uint8_t>(mag),
                           static_cast<uint8_t>(planarAccel),
                           static_cast<uint8_t>(onTable)};
  return sendCommand(COMMAND_ME_CALIBRATION, parameters, sizeof(parameters));
}

bool BNO08x::endCalibration() {
  uint8_t parameters[5] = {0, 0, 0, 0, 0};
  return sendCommand(COMMAND_ME_CALIBRATION, parameters, sizeof(parameters));
}

bool BNO08x::calibrationComplete() const {
  return accelAccuracy_ == 3 && gyroAccuracy_ == 3 && magAccuracy_ == 3;
}

bool BNO08x::saveCalibration() {
  return sendCommand(COMMAND_SAVE_DCD);
}

bool BNO08x::setPeriodicCalibrationSave(bool enable) {
  uint8_t parameter = enable ? 0x00 : 0x01;
  return sendCommand(COMMAND_DCD_PERIODIC_SAVE, &parameter, 1, false);
}

bool BNO08x::wasReset() {
  bool result = resetSeen_;
  resetSeen_ = false;
  return result;
}

}  // namespace nimu
