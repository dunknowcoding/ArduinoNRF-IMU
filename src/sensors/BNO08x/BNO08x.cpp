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

void BNO08x::nudgeBus() {
  if (wire_ == nullptr) return;
  wire_->beginTransmission(address_);
  (void)wire_->endTransmission();
}

bool BNO08x::beginI2C(TwoWire& wire, uint8_t address) {
  wire_ = &wire;
  address_ = address;

  // begin() is safe to repeat - every core treats a second call on a live bus
  // as a no-op - but it takes no pins, so it can only ever open the DEFAULT
  // SDA/SCL. A sketch using any other pins must call Wire.begin(sda, scl)
  // itself before getting here; passing a TwoWire& is what says "I have set
  // this up". Deliberately no setClock() any more: forcing 400 kHz threw away
  // a caller's deliberate choice of a slower clock for long wires or a
  // marginal bus. Use setBusClockHz() if you want this driver to set it.
  wire_->begin();
  if (busClockHz_ != 0) wire_->setClock(busClockHz_);

  // Try the whole bring-up more than once before giving up.
  //
  // Measured on an ESP32 with a BNO085 that a scanner finds on every pass:
  // bring-ups alternated pass, fail, pass, fail for twenty runs in a row. The
  // failing ones were refused at their very first write with Wire code 4, and
  // an identical write issued by hand at that same moment was refused too, so
  // it is genuine bus state rather than a mistake in how this driver calls
  // Wire.
  //
  // The useful part is what the trace showed next: a failing attempt resets
  // the sensor anyway. The write reaches the part, the controller reports an
  // error, and the following attempt finds fresh boot traffic waiting and
  // succeeds. The alternation was never two different outcomes - it was one
  // outcome that needed two passes.
  //
  // Three attempts, because a failing one costs about 35 ms while giving up
  // on a healthy sensor costs the whole application. A genuinely absent chip
  // still reports absent, just 70 ms later.
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    if (attempt != 0) {
      if (debug_ != nullptr) {
        debug_->print(F("  [b] attempt "));
        debug_->print(attempt + 1);
        debug_->println(F(" - the last one reset the part, so it should be ready"));
      }
      // Let the reset the previous attempt provoked finish, then spend one
      // discarded transaction putting the controller back in step.
      delay(120);
      nudgeBus();
      delay(5);
    }
    if (attemptBringUp(attempt == 2)) return true;
  }
  return false;
}

bool BNO08x::attemptBringUp(bool lastChance) {
  productValid_ = false;
  resetSeen_ = false;
  lastError_ = Error::None;
  for (uint8_t& sequence : txSequence_) sequence = 0;

  // Let the controller settle, then spend one throwaway transaction on it.
  //
  // The first transfer after Wire.begin() is unreliable on ESP32 - it comes
  // back NACKed or with a bus fault on a bus that is demonstrably fine a
  // millisecond later. A sketch that happens to do something else first never
  // notices; one that calls begin() immediately gets an intermittent "not
  // found" on a healthy sensor, which is a miserable thing to debug.
  //
  // The driver cannot rely on the caller having warmed the bus, so it warms
  // it here. An address probe costs microseconds and is discarded.
  delay(10);
  nudgeBus();
  delay(5);

  if (resetPin_ >= 0 && !hardwareReset()) {
    lastError_ = Error::ResetFailed;
    return false;
  }

  // Reset first and drain afterwards, never the other way round.
  //
  // This used to drain before resetting, to clear a stream left half-read by
  // an MCU reboot - a real problem, which shows up as a header with a
  // nonsense length in it (26856 bytes, in one measured case). But draining
  // here cost more than it saved: against a chip that had been streaming, the
  // reads that come back empty leave the next write refused with code 4, and
  // no probe or delay rescues it. That is the whole reason the first attempt
  // of every single bring-up was failing.
  //
  // The measurements are unambiguous. A write to a streaming chip with a
  // probe immediately before it was accepted 4 times out of 4; the same write
  // with reads in between was refused every time; and settling for up to
  // 200 ms in place of the probe did not help once in twenty.
  //
  // Resetting first loses nothing, because the reset is what resynchronises
  // the stream in the first place - the part reboots and starts a fresh one,
  // announcing itself on channels 0, 1 and 2. A desynchronised read pointer
  // cannot spoil a write, which begins with its own START.
  if (!softReset()) {
    // softReset() only fails when the very first write is not acknowledged,
    // so at this point the chip is absent, at another address, or not in I2C
    // mode at all - on the BNO085 that means PS0/PS1 are not both low.
    lastError_ = Error::NoResponse;
    return false;
  }
  // If the boot did not finish, stop here and let the caller try again.
  //
  // Traced over eight bring-ups, this one line predicts the outcome every
  // time: each attempt that saw the control channel come up went on to work,
  // and the one that did not had both rounds of Set Feature ignored and then
  // failed. Pressing on from here is not merely futile, it is expensive - it
  // spends 600 ms proving the chip is not listening, and by the time the next
  // attempt starts the sensor has moved on again.
  //
  // The last attempt carries on regardless. Not every part in this family is
  // guaranteed to announce on channel 2, and refusing to open a sensor that
  // would otherwise have worked is worse than the wasted time.
  if (!controlSeen_ && !lastChance) {
    if (debug_ != nullptr) {
      debug_->println(F("  [b] boot incomplete - abandoning this attempt"));
    }
    lastError_ = Error::NoResponse;
    return false;
  }

  if (!requestProductId()) {
    // It acknowledged the write but never returned a product ID, so something
    // is on the bus and talking, just not SHTP.
    lastError_ = Error::NoProductId;
    return false;
  }
  // The product-ID exchange has only just finished; let it settle before
  // sending configuration.
  delay(20);

  return enableDefaultReports();
}

bool BNO08x::enableDefaultReports() {
  // Enable, then prove it took.
  //
  // A Set Feature command is acknowledged on the wire long before the chip
  // has scheduled anything, so "the write succeeded" says nothing about
  // whether reports will arrive. Two runs in twenty came up with every write
  // accepted, a valid product ID, and then complete silence - no Get Feature
  // response, no sensor reports, nothing. From the caller's side that is
  // indistinguishable from a broken sensor, and it is the worst way for a
  // bring-up to fail because it fails later, somewhere else.
  //
  // So the enables are followed by a wait for real traffic, and if none comes
  // they are sent again. Once is enough - a part that ignores two rounds of
  // Set Feature has something else wrong with it, and saying so is more use
  // than trying for ever.
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
    // 300 ms is not a guess and not a budget to be widened. Doubling it was
    // tried: at 100 kHz and above both settings pass every run, and at 50 kHz
    // both fail about one run in six, so the failures there are not reports
    // arriving late. The wait returns the instant the first report lands, so
    // the number only matters when something is actually wrong - and a longer
    // one just makes a genuinely mute part take longer to say so.
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
  controlSeen_ = false;
  if (!sendPacket(CHANNEL_EXECUTABLE, &reset, 1)) return false;
  delay(100);

  // Wait for the chip to say it is back, rather than guessing at a delay.
  //
  // While it reboots it NACKs every read, so an empty queue means "still
  // booting", not "nothing left to read". Treating a few empty reads as
  // "drained" returned within milliseconds and left the caller talking to a
  // chip that was not listening yet: the next write came back refused with
  // wire code 4 and bring-up failed, with the device perfectly healthy and
  // about to announce itself.
  //
  // The reset-complete report on the executable channel is that
  // announcement. Typical is well under 200 ms; the deadline is generous
  // because failing here costs the whole bring-up.
  const uint32_t deadline = millis() + 800;
  while (millis() < deadline && !resetSeen_) {
    if (!receivePacket()) delay(5);
  }
  if (debug_ != nullptr) {
    debug_->println(resetSeen_ ? F("  [b] reset complete seen")
                               : F("  [b] no reset-complete - carrying on"));
  }

  // Then wait for the control channel, because the reset-complete report is
  // not the end of the boot.
  //
  // A BNO085 announces itself on three channels after a reset: the SHTP
  // advertisement on channel 0, the reset-complete on channel 1, and an
  // unsolicited response on channel 2. Only the first two were being waited
  // for, and the third routinely arrives a little later.
  //
  // Carrying on without it looked harmless and was not. Traced over a full
  // period, every bring-up whose post-reset drain caught all three packets
  // went on to work, and every one that saw only channels 0 and 1 had its
  // Set Feature commands ignored - no Get Feature response, no reports, twice
  // in a row - and then failed. The part was still finishing its boot and was
  // not yet accepting configuration.
  //
  // That is the whole of the "every fifth bring-up fails" pattern: a race
  // that the sensor won four times out of five. Waiting for channel 2 costs
  // nothing when it has already arrived, which is the common case.
  const uint32_t controlDeadline = millis() + 600;
  while (millis() < controlDeadline && !controlSeen_) {
    if (!receivePacket()) delay(2);
  }
  if (debug_ != nullptr) {
    debug_->println(controlSeen_ ? F("  [b] control channel up - boot finished")
                                 : F("  [b] no control-channel packet - carrying on"));
  }

  // Then take whatever else the reboot queued, so the caller starts on a
  // packet boundary rather than mid-stream.
  drainQueue(200);
  if (debug_ != nullptr) debug_->println(F("  [b] post-reset drain done"));
  delay(20);
  return true;
}

bool BNO08x::sendPacket(uint8_t channel, const uint8_t* payload,
                        uint8_t length) {
  if (wire_ == nullptr || channel >= 6 || length > 28) return false;
  const uint16_t packetLength = static_cast<uint16_t>(length) + 4;

  // The sequence number is read here and only committed once the frame has
  // actually gone out. It used to be incremented inside the write, which
  // meant every refused attempt burned a number the device never saw: three
  // retries advanced the channel's sequence by four for one delivered packet,
  // and the gaps showed up on the device's side as lost frames.
  const uint8_t sequence = txSequence_[channel];

  // A write issued straight after a run of NACKed reads is refused with code
  // 4 - the controller has not settled yet. That is exactly the position
  // requestProductId() is in, arriving immediately after a drain loop, and a
  // single refusal there failed the whole bring-up while the chip sat waiting
  // to answer. Rebuild the frame and try again rather than giving up on it.
  //
  // Every write is preceded by a discarded address-only transaction, with
  // nothing at all in between.
  //
  // Against a chip that has been streaming and then left alone for a moment -
  // which is the state every re-open starts from - a plain write is refused
  // with code 4. Waiting does not help it: measured at 0, 50, 100, 150 and
  // 200 ms of settling, the write was refused 20 times out of 20. Sending a
  // bare probe first and then writing immediately was accepted 4 times out of
  // 4, and putting so much as 50 ms between the probe and the write broke it
  // again, 0 out of 15.
  //
  // So it is adjacency that matters, not time, and the probe has to be the
  // transaction directly before the one that counts. It comes back NACKed
  // every time and that is fine - it is not there to succeed, it is there to
  // put the controller and the sensor back in step.
  for (uint8_t attempt = 0; attempt < 4; ++attempt) {
    if (attempt != 0) delay(4);
    nudgeBus();
    wire_->beginTransmission(address_);
    wire_->write(static_cast<uint8_t>(packetLength));
    wire_->write(static_cast<uint8_t>(packetLength >> 8));
    wire_->write(channel);
    wire_->write(sequence);
    if (length != 0) wire_->write(payload, length);
    // Keep the raw code. "begin() failed" is not a diagnosis; 2 (nobody at
    // that address) and 3 (there, but rejecting the data) send you to
    // opposite ends of the bench, and 4 means the peripheral is unhappy.
    lastWireError_ = wire_->endTransmission();
    if (lastWireError_ == 0) {
      txSequence_[channel] = static_cast<uint8_t>(sequence + 1);
      return true;
    }
  }
  if (debug_ != nullptr) {
    debug_->print(F("    tx ch"));
    debug_->print(channel);
    debug_->print(F(" refused, wire code "));
    debug_->println(lastWireError_);
  }
  return false;
}

// Largest payload slice we can ask for in one transaction. Every read returns
// a repeated 4-byte header first, so the slice is the buffer minus those four.
//
// This used to be hardcoded to 28, the AVR-safe figure. On a core with a 128
// byte buffer that turned the 276-byte boot advertisement into ten
// transactions instead of three - and every extra transaction is another
// chance to hit the empty-queue NACK below.
static uint8_t bno08xMaxChunk() {
#if defined(I2C_BUFFER_LENGTH)
  const uint16_t buffer = I2C_BUFFER_LENGTH;
#elif defined(BUFFER_LENGTH)
  const uint16_t buffer = BUFFER_LENGTH;
#elif defined(SERIAL_BUFFER_SIZE)
  const uint16_t buffer = 32;
#else
  const uint16_t buffer = 32;
#endif
  if (buffer <= 8) return 4;
  const uint16_t usable = buffer - 4;
  return usable > 124 ? 124 : static_cast<uint8_t>(usable);
}

bool BNO08x::readPayload(uint16_t length) {
  const uint8_t maxChunk = bno08xMaxChunk();
  uint16_t offset = 0;
  while (offset < length) {
    // Narrow AFTER capping, never before.
    //
    // This used to read
    //     uint8_t chunk = (uint8_t)(length - offset);
    //     if (chunk > maxChunk) chunk = maxChunk;
    // which truncates the subtraction before the cap can act. Whenever the
    // bytes remaining were an exact multiple of 256 the cast produced 0, the
    // cap left it at 0, offset advanced by nothing, and the loop spun for
    // ever.
    //
    // The BNO085's boot advertisement is 276 bytes, so a 272-byte payload
    // steps 272 -> (uint8_t)272 = 16, then 256 -> (uint8_t)256 = 0, and hangs
    // on the second iteration - every time, on the very first packet the chip
    // sends. It only escaped notice because the old read path abandoned the
    // packet on the first empty-queue NACK and so rarely got this far.
    const uint16_t remaining = length - offset;
    const uint8_t chunk = (remaining > maxChunk) ? maxChunk
                                                 : static_cast<uint8_t>(remaining);
    const uint8_t requested = static_cast<uint8_t>(chunk + 4);

    // The BNO085 NACKs a read when its output queue is momentarily empty,
    // which is normal SHTP behaviour rather than a fault - measured at 26 in
    // 200 attempts on a healthy bus, arriving in runs of up to thirteen.
    //
    // Giving up here abandoned the packet halfway through and left the
    // device's read pointer mid-stream, so nothing afterwards parsed either.
    // With a 276-byte advertisement needing several transactions, that made a
    // clean drain unlikely and bring-up failed for reasons that looked like
    // anything but this. Wait for the device instead.
    // Retries are not free: a failed requestFrom() can burn the core's whole
    // I2C timeout, 50 ms by default on ESP32. Eight of those per chunk turned
    // a three-chunk packet into a second and a half, and two drain loops into
    // a bring-up that looked like a hang. Three is enough to ride out a busy
    // moment without that.
    bool got = false;
    for (uint8_t retry = 0; retry < 3 && !got; ++retry) {
      got = (wire_->requestFrom(address_, requested, true) == requested);
    }
    if (!got) {
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
  // A NACK here just means the queue is empty right now; callers poll, so one
  // short retry is enough to ride out a busy moment without stalling them.
  // No retry on the header: an empty queue is the common case and callers
  // poll, so paying an I2C timeout here just to ask twice slows every caller
  // down for nothing.
  if (wire_->requestFrom(address_, static_cast<uint8_t>(4), true) != 4) {
    return false;   // queue empty; too common to be worth tracing
  }
  for (uint8_t i = 0; i < 4; ++i) header_[i] = static_cast<uint8_t>(wire_->read());
  uint16_t packetLength = (static_cast<uint16_t>(header_[1]) << 8) | header_[0];
  packetLength &= 0x7FFF;
  if (packetLength < 4 || packetLength > 512) {
    if (debug_ != nullptr && packetLength != 0) {
      debug_->print(F("    rx REJECTED length "));
      debug_->print(packetLength);
      debug_->println(F(" - stream out of step"));
    }
    return false;
  }
  if (!readPayload(packetLength - 4)) return false;
  if (header_[2] == CHANNEL_EXECUTABLE && payloadLength_ > 0 && payload_[0] == 1) {
    resetSeen_ = true;
  }
  if (header_[2] == CHANNEL_CONTROL) controlSeen_ = true;
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
