# Changelog

## 0.4.1

Reliable BMI270 and BNO08x operation on shared and recovery-sensitive I2C
buses.

- BMI270 now bundles Bosch Sensortec's standard configuration image with its
  license, performs bounded initialization and recovery, reports exact status,
  and exposes the advanced feature controls through the public driver.
- BNO085/BNO086 transport now follows bounded SHTP framing and continuation
  rules, distinguishes an ordinary no-report poll from a transport error, and
  exposes additional fusion, motion, event, calibration, and diagnostic data.
- Common I2C recovery preserves caller-selected pins and clock settings and no
  longer tears down a healthy shared bus.
- MPU6050 accepts known register-compatible clone identities without weakening
  the normal identity check.

## 0.4.0

Cross-MCU portability: the sensor and bus layers now build on every major
Arduino architecture, not just the ArduinoNRF nRF52840 core.

- **Verified building** on AVR (Nano/Uno), megaAVR (Nano Every), ESP32/ESP32-S3,
  ESP8266, RP2040, SAMD21 (Arduino Zero), Renesas RA4M1 (UNO R4), and nRF52.
- `Vec3` gains explicit constructors so `Vec3{}` and `Vec3{x, y, z}` compile
  under C++11 (the AVR core's default); a struct with default member
  initializers is not an aggregate before C++14.
- `IMUBus::recoverBus()` uses each core's portable default `SDA`/`SCL` pin
  macros instead of the `TwoWire::pinSDA()/pinSCL()` accessors, which exist on
  only a few cores (they broke ESP32-S3, RP2040, SAMD, and Renesas). Cores that
  expose neither fall back to a plain I2C re-init, and `TwoWire::end()` (absent
  on ESP8266) is called through a portable helper.
- `powf` forward-declarations in the BMP180/BMP280/MS5611 drivers no longer
  conflict with avr-libc `<math.h>` on AVR.
- The ST **LSM6DSV320X** advanced driver is compiled out on AVR (its generated
  register layer needs C++14+ and more flash/RAM than an ATmega328P offers).
  It is unchanged on all other architectures.

No public API changes; existing nRF52 builds are unaffected.

## 0.3.0

Initial public sensor catalog on the ArduinoNRF nRF52840 core.
