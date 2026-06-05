# NiusIMU

A **unified, friendly IMU library** for the [ArduinoNRF](../ArduinoNRF) board
package. One common API across every IMU, with chip-specific extras where a
sensor offers something unique — so your code reads the same whether the board
carries an MPU-9250 today or something else tomorrow.

It doubles as a **bring-up and debugging tool for the ArduinoNRF core**: all
sensor traffic flows through one small bus layer, so the I2C (`Wire`) and SPI
drivers get exercised by real devices, not just loopback tests.

## Highlights

- **Same API for every sensor.** `update()`, then `accelG()`, `gyroDps()`,
  `magUT()`, `temperatureC()` — identical no matter the chip.
- **Engineering units, always.** g, deg/s, µT, °C. Conversions to m/s² and rad/s
  are one call away.
- **Calibrate once, restore forever.** Gyro, accel and magnetometer calibration
  produce a struct you print and paste back — no waving the board at every boot.
- **Whole boards in one object.** `<GY91.h>` brings up the MPU + BMP280 together
  (`accelG()`, `gyroDps()`, `pressureHpa()`, `altitudeM()`); `<GY9250.h>` is the
  9-axis board. Or use the bare chips directly.
- **Pluggable sensors as folders.** Drop a driver in `src/sensors/<NAME>/`, add a
  one-line forwarding header, and `#include <NAME.h>` works. See
  [docs/ADDING_A_SENSOR.md](docs/ADDING_A_SENSOR.md).
- **Chip-specific power kept available.** The MPU-9250 driver still exposes its
  magnetometer control, raw DLPF codes, data-ready interrupt and raw registers.

## Supported sensors and boards

Chips live in `src/sensors/`; multi-chip breakout boards in `src/boards/`. Pick
whichever name matches what you bought — `<GY91.h>` for the board, or the
individual chip headers if you wired chips up yourself.

| Include | Device | Accel | Gyro | Mag | Pressure |
| ------- | ------ | ----- | ---- | --- | -------- |
| `<GY91.h>` | GY-91 board (MPU-9250/6500 + BMP280) | ✅ | ✅ | ✅† | ✅ |
| `<GY9250.h>` | GY-9250 board (MPU-9250) | ✅ | ✅ | ✅ (AK8963) | — |
| `<MPU9250.h>` | MPU-9250 chip (9-axis) | ✅ | ✅ | ✅ (AK8963) | — |
| `<MPU6500.h>` | MPU-6500 chip (6-axis) | ✅ | ✅ | — | — |
| `<BMP280.h>` | BMP280 barometer | — | — | — | ✅ (+ temp) |

† Many GY-91 boards actually carry a 6-axis MPU-6500 (`WHO_AM_I` 0x70, no
magnetometer). The MPU-9250 driver auto-detects this and runs as a 6-axis part;
`hasMagnetometer()` tells you which one you have.

The MPU-9250 is an MPU-6500 plus an AK8963, so its driver simply **extends**
`MPU6500`. Over I2C the AK8963 is reached through the MPU's internal I2C master
(not bypass), so a dead/absent magnetometer can never jam the accel/gyro bus. On
SPI only accel + gyro are read.

## Install

This library lives next to the ArduinoNRF package. Either copy this folder into
your Arduino `libraries/` directory, or compile an example directly:

```sh
arduino-cli compile \
  --fqbn arduinonrf:nrf52:promicro_nrf52840 \
  --library <path-to>/ArduinoNRF-IMU \
  <path-to>/ArduinoNRF-IMU/examples/GY91/GY91_Basic
```

## Wiring (ProMicro nRF52840, default I2C)

| Sensor | Board pad | nRF52 pin |
| ------ | --------- | --------- |
| SDA | `SDA` (D6) | P1.00 |
| SCL | `SCL` (D7) | P0.11 |
| VCC | `3V3` | — |
| GND | `GND` | — |

## Quick start

```cpp
#include <MPU9250.h>
MPU9250 imu;

void setup() {
  Serial.begin(115200);
  imu.begin();            // Wire @ 0x68, sensible defaults
  imu.calibrateGyro();    // hold still ~1 s
}

void loop() {
  imu.update();
  Vec3 a = imu.accelG();  // g
  Vec3 g = imu.gyroDps(); // deg/s
  Vec3 m = imu.magUT();   // µT
  delay(50);
}
```

## Examples

| Example | What it shows |
| ------- | ------------- |
| [`I2C_Scanner`](examples/I2C_Scanner) | Find devices on the bus; first thing to run on a new board |
| [`GY91/GY91_Basic`](examples/GY91/GY91_Basic) | Whole GY-91 (IMU + barometer) in one quick read loop |
| [`GY91/GY91_Advanced`](examples/GY91/GY91_Advanced) | Full config + guided calibration of every sensor, with on-screen hints |
| [`GY9250/GY9250_Basic`](examples/GY9250/GY9250_Basic) | Minimal 9-axis read loop |
| [`GY9250/GY9250_Calibration`](examples/GY9250/GY9250_Calibration) | Guided gyro/accel/mag calibration that prints paste-ready code |
| [`GY9250/GY9250_Advanced`](examples/GY9250/GY9250_Advanced) | Custom ranges, DLPF, sample rate, data-ready, raw registers |
| [`GY9250/GY9250_SPI`](examples/GY9250/GY9250_SPI) | Read over SPI: configure at 1 MHz, then burst data at 8 MHz |
| [`GY9250/GY9250_DataReadyInterrupt`](examples/GY9250/GY9250_DataReadyInterrupt) | Read on the INT pin (D0) — one read per fresh sample, no polling |
| [`GY9250/GY9250_FSYNC`](examples/GY9250/GY9250_FSYNC) | Time-stamp an external event on FSYNC (D1) into the sample stream |
| [`MPU6500/MPU6500_Basic`](examples/MPU6500/MPU6500_Basic) | Bare 6-axis accelerometer + gyroscope |
| [`BMP280/BMP280_Basic`](examples/BMP280/BMP280_Basic) | Pressure, temperature and altitude, with sea-level reference |

## Interfaces: SPI, INT and FSYNC (MPU-6500 / MPU-9250)

The IMU drivers run over **I2C or SPI** and expose the chip's interrupt and
external-sync pins. All three were hardware-verified on a GY-9250 over SPI:

- **SPI.** `beginSPI(SPI, SS)` (the part is pinned to SPI via `I2C_IF_DIS`).
  Configure at the default 1 MHz, then call `setBusClockHz()` higher to burst
  data — verified 100% reliable from 0.5 MHz up to the ArduinoNRF SPI ceiling of
  8 MHz, for both register and data reads. The magnetometer is I2C-only, so SPI
  is a 6-axis stream.
- **Data-ready INT.** `setDataReadyInterrupt(true)` pulses the INT pin once per
  sample; attach an interrupt and read only when a fresh sample exists. Verified
  a steady 100 Hz with every sample serviced across the whole SPI speed range.
- **FSYNC.** `setExternalSync(MPU6500::FSYNC_TEMP)` latches the FSYNC pin level
  into a sample LSB so an external event is time-stamped against the IMU data;
  read it back with `fsyncLevel()`. Verified the captured level tracks the pin
  exactly.

See the `GY9250_SPI`, `GY9250_DataReadyInterrupt` and `GY9250_FSYNC` examples.

## The common API (every sensor)

```cpp
bool   begin();                       // default bus + sensible defaults
bool   beginI2C(TwoWire&, uint8_t);   // pick bus / address
bool   beginSPI(SPIClass&, uint8_t);  // SPI with a CS pin
uint8_t whoAmI();  bool isConnected();

bool   update();                      // one fresh snapshot
Vec3   accelG();  Vec3 accelMs2();
Vec3   gyroDps(); Vec3 gyroRps();
Vec3   magUT();   float temperatureC();
bool   hasMagnetometer();

bool   setAccelRangeG(uint16_t);      // 2/4/8/16 …
bool   setGyroRangeDps(uint16_t);     // 250/500/1000/2000 …
bool   setLowPassFilterHz(uint16_t);
bool   setSampleRateHz(uint16_t);

bool   calibrateGyro();  calibrateAccel();  calibrateMag();
IMUCalibration getCalibration();  void setCalibration(IMUCalibration);
```

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for how the pieces fit together.

## License

Apache-2.0. See [LICENSE](LICENSE).
