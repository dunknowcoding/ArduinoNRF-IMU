#if !defined(__AVR__)  // ST LSM6DSV320X driver is not AVR/C++11-portable
#include "LSM6DSV320X.h"

namespace nimu {
using namespace lsm6dsv;
namespace v320x = lsm6dsv320x;

namespace {
int16_t le16(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}
}  // namespace

LSM6DSV320X::LSM6DSV320X() {
  name_ = "LSM6DSV320X";
  stContext_.write_reg = stWrite;
  stContext_.read_reg = stRead;
  stContext_.mdelay = stDelay;
  stContext_.handle = this;
  stContext_.priv_data = nullptr;
}

int32_t LSM6DSV320X::stWrite(void* handle, uint8_t reg, uint8_t* data,
                             uint16_t length) {
  if (handle == nullptr || data == nullptr || length == 0) return -1;
  LSM6DSV320X* sensor = static_cast<LSM6DSV320X*>(handle);
  return sensor->bus_.writeRegisters(reg, data, length) == IMUStatus::Ok
             ? 0
             : -1;
}

int32_t LSM6DSV320X::stRead(void* handle, uint8_t reg, uint8_t* data,
                            uint16_t length) {
  if (handle == nullptr || data == nullptr || length == 0) return -1;
  LSM6DSV320X* sensor = static_cast<LSM6DSV320X*>(handle);
  return sensor->bus_.readRegisters(reg, data, length) == IMUStatus::Ok
             ? 0
             : -1;
}

void LSM6DSV320X::stDelay(uint32_t milliseconds) { delay(milliseconds); }

bool LSM6DSV320X::begin() {
  return beginI2C(Wire, kAddrSA0Low) || beginI2C(Wire, kAddrSA0High);
}

bool LSM6DSV320X::beginI2C(TwoWire& wire, uint8_t address) {
  bus_.beginI2C(wire, address, 400000);
  bus_.recoverBus();
  if (!isConnected() || !reset()) return false;
  return setAccelRangeG(4) && setGyroRangeDps(1000) && setSampleRateHz(120);
}

bool LSM6DSV320X::beginSPI(SPIClass& spi, uint8_t csPin) {
  bus_.beginSPI(spi, csPin, 1000000, 0x80);
  if (!isConnected() || !reset()) return false;
  return setAccelRangeG(4) && setGyroRangeDps(1000) && setSampleRateHz(120);
}

bool LSM6DSV320X::isConnected() { return whoAmI() == v320x::kWhoAmI; }

bool LSM6DSV320X::setGyroRangeDps(uint16_t maxDps) {
  uint8_t code;
  uint16_t range;
  if (maxDps <= 250) { code = 1; range = 250; }
  else if (maxDps <= 500) { code = 2; range = 500; }
  else if (maxDps <= 1000) { code = 3; range = 1000; }
  else if (maxDps <= 2000) { code = 4; range = 2000; }
  else { code = 5; range = 4000; }
  gyroLsbPerDps_ = 32768.0f / range;
  return bus_.updateRegister(v320x::CTRL6, 0x07, code) == IMUStatus::Ok;
}

bool LSM6DSV320X::setHighGRangeG(uint16_t maxG) {
  uint8_t code;
  if (maxG <= 32) { code = 0; highGLsbPerG_ = 1.0f / 0.000976f; }
  else if (maxG <= 64) { code = 1; highGLsbPerG_ = 1.0f / 0.001952f; }
  else if (maxG <= 128) { code = 2; highGLsbPerG_ = 1.0f / 0.003904f; }
  else if (maxG <= 256) { code = 3; highGLsbPerG_ = 1.0f / 0.007808f; }
  else { code = 4; highGLsbPerG_ = 1.0f / 0.010417f; }
  return bus_.updateRegister(v320x::CTRL1_XL_HG, 0x07, code) == IMUStatus::Ok;
}

bool LSM6DSV320X::setHighGSampleRateHz(uint16_t hz) {
  if (hz == 0) return false;
  uint8_t code;
  if (hz <= 480) code = 3;
  else if (hz <= 960) code = 4;
  else if (hz <= 1920) code = 5;
  else if (hz <= 3840) code = 6;
  else code = 7;
  return bus_.updateRegister(v320x::CTRL1_XL_HG, 0xB8,
                             static_cast<uint8_t>((code << 3) | 0x80)) ==
         IMUStatus::Ok;
}

bool LSM6DSV320X::loadUcfExtended(const UcfLineExtended* configuration,
                                  size_t count, uint16_t pollTimeoutMs) {
  if (configuration == nullptr || count == 0) return false;
  for (size_t i = 0; i < count; ++i) {
    const UcfLineExtended& line = configuration[i];
    if (line.operation == UcfOperation::Write) {
      if (bus_.writeRegister(line.address, line.data) != IMUStatus::Ok)
        return false;
    } else if (line.operation == UcfOperation::Read) {
      uint8_t ignored = 0;
      if (bus_.readRegister(line.address, ignored) != IMUStatus::Ok)
        return false;
    } else if (line.operation == UcfOperation::Delay) {
      delay(line.data);
    } else if (line.operation == UcfOperation::PollSet ||
               line.operation == UcfOperation::PollReset) {
      const uint32_t start = millis();
      bool matched = false;
      do {
        uint8_t value = 0;
        if (bus_.readRegister(line.address, value) != IMUStatus::Ok)
          return false;
        matched = line.operation == UcfOperation::PollSet
                      ? (value & line.data) == line.data
                      : (value & line.data) == 0;
        if (matched) break;
      } while (static_cast<uint32_t>(millis() - start) < pollTimeoutMs);
      if (!matched) return false;
    } else {
      return false;
    }
  }
  return true;
}

bool LSM6DSV320X::routeHighGDataReady(uint8_t pin, bool enable) {
  uint8_t mask = pin == 1 ? v320x::HG_INT1_DRDY :
                 pin == 2 ? v320x::HG_INT2_DRDY : 0;
  return mask != 0 &&
         bus_.updateRegister(v320x::CTRL7, mask, enable ? mask : 0) ==
             IMUStatus::Ok;
}

bool LSM6DSV320X::configureHighGWakeup(uint16_t thresholdMg,
                                       uint8_t duration,
                                       uint8_t interruptPin) {
  if (interruptPin != 1 && interruptPin != 2) return false;
  uint16_t rangeMg = static_cast<uint16_t>(32768.0f / highGLsbPerG_ * 1000.0f);
  uint32_t scaled = static_cast<uint32_t>(thresholdMg) * 256UL / rangeMg;
  if (scaled > 255) scaled = 255;
  uint8_t route = interruptPin == 1 ? v320x::HG_INT1_WAKE :
                                      v320x::HG_INT2_WAKE;
  bool ok = bus_.writeRegister(v320x::HG_WAKE_UP_THS,
                               static_cast<uint8_t>(scaled)) == IMUStatus::Ok;
  ok &= bus_.writeRegister(v320x::HG_FUNCTIONS_ENABLE,
                           v320x::HG_INTERRUPTS_ENABLE | route |
                               (duration & 0x0F)) == IMUStatus::Ok;
  return ok;
}

bool LSM6DSV320X::highGWakeupDetected() {
  uint8_t status = 0;
  return bus_.readRegister(v320x::HG_WAKE_UP_SRC, status) == IMUStatus::Ok &&
         (status & v320x::HG_WAKE_EVENT) != 0;
}

bool LSM6DSV320X::highGShockState() {
  uint8_t status = 0;
  return bus_.readRegister(v320x::HG_WAKE_UP_SRC, status) == IMUStatus::Ok &&
         (status & v320x::HG_SHOCK_STATE) != 0;
}

bool LSM6DSV320X::readHighG(Vec3& accelG) {
  uint8_t raw[6];
  if (bus_.readRegisters(v320x::OUTX_L_A_HG, raw, sizeof(raw)) != IMUStatus::Ok)
    return false;
  accelG = Vec3{le16(&raw[0]) / highGLsbPerG_,
                le16(&raw[2]) / highGLsbPerG_,
                le16(&raw[4]) / highGLsbPerG_};
  return true;
}

bool LSM6DSV320X::setMlcEnabled(bool enable) {
  if (!embeddedBank(true)) return false;
  bool ok = bus_.updateRegister(v320x::EMB_FUNC_EN_B,
                                v320x::MLC_CORE_ENABLE,
                                enable ? v320x::MLC_CORE_ENABLE : 0) ==
            IMUStatus::Ok;
  ok &= embeddedBank(false);
  return ok;
}

bool LSM6DSV320X::setFsmEnabled(uint8_t programMask) {
  if (!embeddedBank(true)) return false;
  bool ok = bus_.writeRegister(v320x::FSM_ENABLE, programMask) == IMUStatus::Ok;
  ok &= bus_.updateRegister(v320x::EMB_FUNC_EN_B,
                            v320x::FSM_CORE_ENABLE,
                            programMask ? v320x::FSM_CORE_ENABLE : 0) ==
        IMUStatus::Ok;
  ok &= embeddedBank(false);
  return ok;
}

bool LSM6DSV320X::readMlcOutput(uint8_t index, uint8_t& value) {
  if (index >= 8 || !embeddedBank(true)) return false;
  bool ok = bus_.readRegister(static_cast<uint8_t>(v320x::MLC1_SRC + index),
                              value) == IMUStatus::Ok;
  ok &= embeddedBank(false);
  return ok;
}

bool LSM6DSV320X::readFsmOutput(uint8_t index, uint8_t& value) {
  if (index >= 8 || !embeddedBank(true)) return false;
  bool ok = bus_.readRegister(static_cast<uint8_t>(v320x::FSM_OUTS1 + index),
                              value) == IMUStatus::Ok;
  ok &= embeddedBank(false);
  return ok;
}

bool LSM6DSV320X::enableAdaptiveSelfConfiguration(bool enable) {
  return bus_.updateRegister(FUNC_CFG_ACCESS, v320x::ASC_FSM_WRITE_CTRL,
                             enable ? v320x::ASC_FSM_WRITE_CTRL : 0) ==
         IMUStatus::Ok;
}

bool LSM6DSV320X::adaptiveSelfConfigurationActive() {
  uint8_t status = 0;
  return bus_.readRegister(0x1A, status) == IMUStatus::Ok &&
         (status & 0x04) != 0;
}

bool LSM6DSV320X::setI3CInterrupts(bool enable) {
  return bus_.updateRegister(v320x::CTRL5, v320x::I3C_INTERRUPT_ENABLE,
                             enable ? v320x::I3C_INTERRUPT_ENABLE : 0) ==
         IMUStatus::Ok;
}

}  // namespace nimu

#endif  // !__AVR__
