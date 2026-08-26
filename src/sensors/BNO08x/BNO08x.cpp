#include "BNO08x.h"
#include "../../imu/I2CRecovery.h"

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

  // Callers using custom pins must configure the supplied TwoWire instance
  // before beginI2C(). A zero busClockHz_ preserves their selected clock.
  wire_->begin();
  if (busClockHz_ != 0) wire_->setClock(busClockHz_);

  // Retry complete reset/product/report bring-up within one bounded budget.
  const uint32_t deadline = millis() + 3500;
  sawResponse_ = false;
  invalidHeaderCount_ = 0;
  payloadReadFailureCount_ = 0;
  lastUpdateResult_ = IMUUpdateResult::NoData;
  for (uint8_t attempt = 1; attempt <= 10; ++attempt) {
    if (attempt != 1) {
      if (debug_ != nullptr) {
        debug_->print(F("  [b] attempt "));
        debug_->print(attempt);
        debug_->println(F(" - retrying bounded bring-up"));
      }
      // Let the reset the previous attempt provoked finish.
      delay(120);
    }

    if (attemptBringUp()) return true;

    // Three silent attempts are enough to try the alternate address without
    // spending the complete time budget.
    if (!sawResponse_ && attempt >= 3) {
      if (debug_ != nullptr) {
        debug_->println(F("  [b] no response at this address"));
      }
      return false;
    }
    if (millis() >= deadline) break;
  }
  return false;
}

bool BNO08x::attemptBringUp() {
  productValid_ = false;
  resetSeen_ = false;
  shtpErrorCount_ = 0;
  lastShtpError_ = 0;
  lastError_ = Error::None;
  for (uint8_t& sequence : txSequence_) sequence = 0;

  // Give the controller a bounded settling interval before the first frame.
  delay(10);

  if (resetPin_ >= 0 && !hardwareReset()) {
    lastError_ = Error::ResetFailed;
    return false;
  }

  // Reset before draining so a prior partial SHTP stream is replaced by a
  // fresh boot sequence.
  if (!softReset()) {
    lastError_ = Error::NoResponse;
    return false;
  }

  if (!requestProductId()) {
    lastError_ = Error::NoProductId;
    return false;
  }
  // The product-ID exchange has only just finished; let it settle before
  // sending configuration.
  delay(20);

  return enableDefaultReports();
}

bool BNO08x::enableDefaultReports() {
  // An accepted Set Feature frame is not proof that a report was scheduled.
  // Require report traffic and retry the command set once within the budget.
  for (uint8_t round = 0; round < 2; ++round) {
    bool ok = true;
    ok &= enableReport(SENSOR_ACCELEROMETER, reportIntervalUs_);
    ok &= enableReport(SENSOR_GYROSCOPE, reportIntervalUs_);
    ok &= enableReport(SENSOR_MAGNETIC_FIELD, reportIntervalUs_);
    ok &= enableReport(SENSOR_ROTATION_VECTOR, reportIntervalUs_);
    if (!ok) {
      lastError_ = Error::ReportEnableFailed;
      return false;
    }
    if (waitForReports(300)) return true;
    if (debug_ != nullptr) {
      debug_->println(F("  [b] enabled but silent - sending Set Feature again"));
    }
  }
  lastError_ = Error::NoReports;
  return false;
}

bool BNO08x::waitForReports(uint16_t budgetMs) {
  const uint32_t deadline = millis() + budgetMs;
  while (millis() < deadline) {
    // Any packet on the report channel will do. The Get Feature response the
    // chip sends back on the control channel is a promise; a report is the
    // thing itself.
    if (receivePacket() && header_[2] == CHANNEL_REPORTS) return true;
    delay(2);
  }
  return false;
}

const char* BNO08x::lastErrorText() const {
  switch (lastError_) {
    case Error::None:               return "no error";
    case Error::ResetFailed:        return "the hardware reset pin did not work";
    case Error::NoResponse:
      switch (lastWireError_) {
        case 2:  return "nothing acknowledged that address - wrong address, or "
                        "PS0/PS1 not both LOW so it is not in I2C mode";
        case 3:  return "the address answered but the data was rejected - it is "
                        "there but not accepting SHTP";
        case 4:  return "the I2C peripheral reported a bus fault, not a refusal";
        case 5:  return "the I2C transfer timed out";
        default: return "no acknowledgement";
      }
    case Error::NoProductId:        return "acknowledged but sent no product ID - "
                                           "on the bus but not speaking SHTP";
    case Error::ReportEnableFailed: return "opened, but a sensor report would not enable";
    case Error::NoReports:          return "opened and configured, but no report ever "
                                           "arrived - answering on the bus, yet not "
                                           "measuring";
  }
  return "unknown";
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

// Reads packets until the queue stays empty or the budget runs out. Returns
// how many whole packets were consumed. A rejected header - a length that is
// absurd because the stream is out of step - counts as progress too, because
// the read still advanced the device's pointer.
uint8_t BNO08x::drainQueue(uint16_t budgetMs) {
  const uint32_t deadline = millis() + budgetMs;
  uint8_t consumed = 0;
  uint8_t quiet = 0;
  while (millis() < deadline && quiet < 3 && consumed < 24) {
    if (receivePacket()) {
      ++consumed;
      quiet = 0;
    } else {
      ++quiet;
      delay(2);
    }
  }
  return consumed;
}

bool BNO08x::softReset() {
  const uint8_t reset = 1;

  // Clear the flag first: the chip announced a reset-complete during its
  // power-on boot as well, and mistaking that one for ours would mean
  // carrying on before this reset had even started.
  resetSeen_ = false;
  if (!sendPacket(CHANNEL_EXECUTABLE, &reset, 1)) return false;
  delay(100);

  // Poll for the reset-complete report instead of relying on a fixed delay.
  // Empty reads during reboot remain within this bounded deadline.
  const uint32_t deadline = millis() + 800;
  while (millis() < deadline && !resetSeen_) {
    if (!receivePacket()) delay(5);
  }
  if (debug_ != nullptr) {
    debug_->println(resetSeen_ ? F("  [b] reset complete seen")
                               : F("  [b] no reset-complete - carrying on"));
  }

  // Drain the bounded boot announcement tail, then explicitly request the
  // product ID. A control-channel announcement is not required by every SH-2
  // firmware build, so its absence cannot be used as a bring-up gate.
  drainQueue(30);
  if (debug_ != nullptr) debug_->println(F("  [b] post-reset drain done"));
  delay(20);
  return true;
}

bool BNO08x::sendPacket(uint8_t channel, const uint8_t* payload,
                        uint8_t length) {
  if (wire_ == nullptr || channel >= 6 || length > 28) return false;
  const uint16_t packetLength = static_cast<uint16_t>(length) + 4;

  // Commit the sequence only after the frame is accepted.
  const uint8_t sequence = txSequence_[channel];

  // Retry bounded transient controller errors with the same sequence number.
  for (uint8_t attempt = 0; attempt < 4; ++attempt) {
    if (attempt != 0) delay(4);
    wire_->beginTransmission(address_);
    wire_->write(static_cast<uint8_t>(packetLength));
    wire_->write(static_cast<uint8_t>(packetLength >> 8));
    wire_->write(channel);
    wire_->write(sequence);
    if (length != 0) wire_->write(payload, length);
    // Retain the raw controller code for diagnostics.
    lastWireError_ = wire_->endTransmission();
    if (lastWireError_ == 0) {
      txSequence_[channel] = static_cast<uint8_t>(sequence + 1);
      sawResponse_ = true;
      return true;
    }
    detail::resetI2CControllerAfterError(*wire_);
  }
  if (debug_ != nullptr) {
    debug_->print(F("    tx ch"));
    debug_->print(channel);
    debug_->print(F(" refused, wire code "));
    debug_->println(lastWireError_);
  }
  return false;
}

// SH-2 I2C framing is defined in 32-byte transactions. Every continuation
// repeats the four-byte SHTP header, leaving 28 payload bytes. A larger host
// Wire buffer does not increase the sensor-side frame size; requesting 128
// bytes consumed later packet headers as payload and left the stream
// misaligned.
static uint8_t bno08xMaxChunk() {
  return 28;
}

bool BNO08x::readPayload(uint16_t length) {
  const uint8_t maxChunk = bno08xMaxChunk();
  uint16_t offset = 0;
  while (offset < length) {
    // Cap in the wide type before narrowing; 256-byte boundaries must never
    // turn into a zero-sized chunk.
    const uint16_t remaining = length - offset;
    const uint8_t chunk = (remaining > maxChunk) ? maxChunk
                                                 : static_cast<uint8_t>(remaining);
    const uint8_t requested = static_cast<uint8_t>(chunk + 4);

    // A partial packet cannot be abandoned without losing SHTP alignment.
    // Retry each continuation within a fixed count, then fail closed.
    bool got = false;
    for (uint8_t retry = 0; retry < 6 && !got; ++retry) {
      got = (wire_->requestFrom(address_, requested,
                                static_cast<uint8_t>(true)) == requested);
      if (!got && retry != 5) delay(1);
    }
    if (!got) {
      receiveFault_ = true;
      ++payloadReadFailureCount_;
      detail::resetI2CControllerAfterError(*wire_);
      if (debug_ != nullptr) {
        debug_->print(F("    rx chunk failed, "));
        debug_->print(length - offset);
        debug_->print(F(" of "));
        debug_->print(length);
        debug_->println(F(" bytes unread - packet abandoned"));
      }
      return false;
    }

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
  receiveFault_ = false;
  // A NACK here just means the queue is empty right now; callers poll, so one
  // short retry is enough to ride out a busy moment without stalling them.
  // No retry on the header: an empty queue is the common case and callers
  // poll, so paying an I2C timeout here just to ask twice slows every caller
  // down for nothing.
  if (wire_->requestFrom(address_, static_cast<uint8_t>(4),
                         static_cast<uint8_t>(true)) != 4) {
    return false;   // queue empty; too common to be worth tracing
  }
  sawResponse_ = true;
  for (uint8_t i = 0; i < 4; ++i) header_[i] = static_cast<uint8_t>(wire_->read());
  uint16_t packetLength = (static_cast<uint16_t>(header_[1]) << 8) | header_[0];
  packetLength &= 0x7FFF;
  // SH-2 returns an all-zero length header when no packet is queued. Polling
  // without the optional interrupt pin therefore reaches this path normally;
  // it is NoData, not a malformed frame or a transport fault.
  if (packetLength == 0) return false;
  if (packetLength < 4 || packetLength > 512) {
    receiveFault_ = true;
    ++invalidHeaderCount_;
    if (debug_ != nullptr && packetLength != 0) {
      debug_->print(F("    rx REJECTED length "));
      debug_->print(packetLength);
      debug_->print(F(" header="));
      for (uint8_t i = 0; i < 4; ++i) {
        if (header_[i] < 0x10) debug_->print('0');
        debug_->print(header_[i], HEX);
        if (i != 3) debug_->print(':');
      }
      debug_->println();
    }
    return false;
  }
  if (!readPayload(packetLength - 4)) return false;
  if (header_[2] == CHANNEL_EXECUTABLE && payloadLength_ > 0 && payload_[0] == 1) {
    resetSeen_ = true;
  }

  // The command channel carries SHTP's own error list: report id 0x01
  // followed by one byte per error. The packets grow as errors accumulate,
  // which is why a boot shows lengths of 6, then 7, then 8. Reading them and
  // dropping them on the floor threw away the one place the part explains
  // itself, so keep the count and the most recent code.
  if (header_[2] == CHANNEL_COMMAND && payloadLength_ >= 2 && payload_[0] == 0x01) {
    shtpErrorCount_ = static_cast<uint16_t>(payloadLength_ - 1);
    lastShtpError_ = payload_[payloadLength_ - 1];
    if (debug_ != nullptr) {
      debug_->print(F("    shtp errors now "));
      debug_->print(shtpErrorCount_);
      debug_->print(F(", latest 0x"));
      debug_->println(lastShtpError_, HEX);
    }
  }
  // A BNO085 emits advertisement, reset-complete and product ID unprompted
  // after every reset. Capturing the product ID here rather than only inside
  // requestProductId() means a drain loop can no longer discard it.
  if (header_[2] == CHANNEL_REPORTS && payloadLength_ >= 6 &&
      payload_[0] == REPORT_BASE_TIMESTAMP) {
    lastReportId_ = payload_[5];
  }
  const bool wasProductId = captureProductId();
  if (debug_ != nullptr) {
    debug_->print(F("    rx ch"));
    debug_->print(header_[2]);
    debug_->print(F(" seq"));
    debug_->print(header_[3]);
    debug_->print(F(" len"));
    debug_->print(packetLength);
    debug_->print(F(" first "));
    debug_->print(payloadLength_ > 0 ? payload_[0] : 0, HEX);
    // On the report channel the first byte is only the timestamp wrapper; the
    // sensor's own report id sits five bytes in, and that is the number you
    // actually want when working out which features a part is delivering.
    if (header_[2] == CHANNEL_REPORTS && payloadLength_ >= 6 &&
        payload_[0] == REPORT_BASE_TIMESTAMP) {
      debug_->print(F(" report 0x"));
      debug_->print(payload_[5], HEX);
    }
    if (wasProductId) debug_->print(F("   <-- product ID captured"));
    debug_->println();
  }
  return true;
}

bool BNO08x::requestProductId(uint16_t timeoutMs) {
  // The chip volunteers its product ID during boot, so by the time softReset()
  // has drained the reset traffic we usually have it already. Asking a second
  // time is not harmless: the answer to a repeated request is not another
  // 0xF1, so the old code sat waiting for a report that was never coming
  // again - having thrown the first one away moments earlier.
  if (productValid_) {
    if (debug_ != nullptr) debug_->println(F("  [b] product ID already captured"));
    return true;
  }
  if (debug_ != nullptr) debug_->println(F("  [b] asking for product ID"));

  // Give the controller a moment after the drain loop before writing.
  delay(4);
  const uint8_t request[2] = {REPORT_PRODUCT_ID_REQUEST, 0};
  if (!sendPacket(CHANNEL_CONTROL, request, sizeof(request))) {
    if (debug_ != nullptr) debug_->println(F("  [b] request would not send"));
    return false;
  }
  uint32_t started = millis();
  while (millis() - started < timeoutMs) {
    if (!receivePacket()) {
      delay(2);
      continue;
    }
    // receivePacket() captures a product ID wherever it appears, so there is
    // nothing to parse here - just wait for it to have landed.
    if (productValid_) return true;
  }
  return false;
}

// Fills product_ if the packet currently in the buffer is a product-ID
// response. Called from receivePacket(), because the BNO085 volunteers this
// report during boot rather than only when asked - see the note there.
bool BNO08x::captureProductId() {
  if (header_[2] != CHANNEL_CONTROL || payloadLength_ < 14 ||
      payload_[0] != REPORT_PRODUCT_ID_RESPONSE) {
    return false;
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
  if (!sendPacket(CHANNEL_CONTROL, command, sizeof(command))) return false;

  // Give the chip a moment to act on it.
  //
  // Set Feature is a command, not a register write: the part has to schedule
  // the report before it will honour the next one. Firing several back to back
  // with no gap meant later ones were dropped - and since begin() enables four
  // in a row, a freshly opened sensor delivered nothing at all until the
  // application happened to call setSampleRateHz() later, by which time the
  // chip had settled. "begin() succeeded but update() never returns true" is a
  // miserable thing to debug, and this is the whole of it.
  delay(5);
  return true;
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
  // Event detectors carry no measurement, only the fact that it happened.
  if (id == SENSOR_STEP_DETECTOR) {
    ++stepEvents_;
    data_.timestamp = micros();
    return true;
  }
  if (id == SENSOR_STABILITY_DETECTOR) {
    ++stabilityEvents_;
    data_.timestamp = micros();
    return true;
  }
  if (id == SENSOR_SHAKE_DETECTOR && length >= 6) {
    // Bits 0..2 of a 16-bit field say which axes the shake was along.
    shakeAxes_ = static_cast<uint8_t>(le16(&report[4]) & 0x07);
    ++shakeEvents_;
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
  } else if (id == SENSOR_GYROSCOPE_UNCALIBRATED) {
    // Rate then drift, six Q9 values in radians per second.
    if (length < 16) return false;
    constexpr float kRadToDeg = 57.29577951308232f;
    gyroUncal_ = Vec3{qToFloat(x, 9) * kRadToDeg, qToFloat(y, 9) * kRadToDeg,
                      qToFloat(z, 9) * kRadToDeg};
    gyroDrift_ = Vec3{qToFloat(le16(&report[10]), 9) * kRadToDeg,
                      qToFloat(le16(&report[12]), 9) * kRadToDeg,
                      qToFloat(le16(&report[14]), 9) * kRadToDeg};
  } else if (id == SENSOR_MAGNETIC_FIELD_UNCALIBRATED) {
    // Field then hard-iron offset, six Q4 values in microtesla.
    if (length < 16) return false;
    magUncal_ = Vec3{qToFloat(x, 4), qToFloat(y, 4), qToFloat(z, 4)};
    magHardIron_ = Vec3{qToFloat(le16(&report[10]), 4),
                        qToFloat(le16(&report[12]), 4),
                        qToFloat(le16(&report[14]), 4)};
  } else if (id == SENSOR_ARVR_ROTATION_VECTOR ||
             id == SENSOR_ARVR_GAME_ROTATION_VECTOR) {
    // Same Q14 quaternion layout as the plain rotation vectors.
    if (length < 12) return false;
    Quaternion* q = (id == SENSOR_ARVR_ROTATION_VECTOR) ? &arvrQuaternion_
                                                        : &arvrGameQuaternion_;
    q->x = qToFloat(x, 14);
    q->y = qToFloat(y, 14);
    q->z = qToFloat(z, 14);
    q->w = qToFloat(le16(&report[10]), 14);
    q->accuracy = accuracy;
    if (id == SENSOR_ARVR_ROTATION_VECTOR && length >= 14) {
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
  bool fault = false;
  for (uint8_t i = 0; i < 8; ++i) {
    if (!receivePacket()) {
      fault = receiveFault_;
      break;
    }
    parsed |= parsePacket();
  }
  lastUpdateResult_ = parsed ? IMUUpdateResult::Sample
                             : (fault ? IMUUpdateResult::Error
                                      : IMUUpdateResult::NoData);
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
