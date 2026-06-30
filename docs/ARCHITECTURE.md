# NiusIMU architecture

The library has three layers. Everything a sketch sees is the top one.

```
  your sketch
      |
      v
  +-----------------------------+   <- public include forwarders (src/*.h)
  |  GY91, GY9250  (boards)     |       src/GY91.h -> boards/GY91/  (compose chips)
  |  MPU9250, MPU6500, BMP280   |       src/MPU9250.h -> sensors/MPU9250/
  +-----------------------------+
      |  IMUs implement                 (BMP280 is a barometer: its own small class)
      v
  +-----------------------------+   <- the unified interface (src/imu/)
  |  IMUSensor (abstract)       |       common API, units, calibration maths
  |  IMUTypes  (Vec3, IMUData…) |       MPU9250 : MPU6500 : IMUSensor
  +-----------------------------+
      |  talks through
      v
  +-----------------------------+   <- one bus surface (src/imu/IMUBus)
  |  IMUBus  (I2C or SPI)       |       register read/write, auto-chunked bursts
  +-----------------------------+
      |
      v
   Wire / SPI  (ArduinoNRF core)
```

## Chips, boards, and the odd sensor out

- **Chips** live in `src/sensors/<NAME>/`. The inertial ones derive from
  `IMUSensor`. Where one chip is a superset of another they inherit: `MPU9250`
  **extends** `MPU6500` and adds only the AK8963 magnetometer, so all the
  inertial code exists once. The `BMP280` is a barometer, not an IMU, so it does
  not derive from `IMUSensor` — it has its own small surface but still reads/writes
  through the same `IMUBus`.
- **Boards** live in `src/boards/<NAME>/` when they compose multiple chips,
  auto-detect different populated cores, add carrier behavior such as
  protocol-select GPIO, or preserve an established printed marketplace name.
  `GY91`, for example, combines an MPU-family IMU and a BMP280. Multiple
  unnamed pinout variants of one core do not receive invented `Module` classes;
  they share the chip driver and their silks are documented in
  `docs/IMU_SUPPORT.md`.

## Why two headers can have the same filename

Arduino adds the library's `src/` directory to the normal include path, not
every sensor subdirectory. Therefore `src/LSM6DSV320X.h` is a tiny public
forwarder that makes this work:

```cpp
#include <LSM6DSV320X.h>
```

The actual class declaration stays beside its implementation and register
definitions in `src/sensors/LSM6DSV320X/LSM6DSV320X.h`. These files are not
copies: the root header contains only an include guard and one include. Board
headers follow the same pattern. Removing the root forwarder would expose an
internal path in every Arduino sketch; flattening every implementation into
`src/` would discard the folder-per-driver organization.

## Why this shape

**All IMUs are equal citizens.** Anything common — units, the cached snapshot,
the calibration model `(raw - bias) * scale`, the generic gyro/accel calibration
routines — lives once in `IMUSensor`. A driver cannot accidentally make
`accelG()` mean something different.

**Drivers stay tiny.** A sensor driver implements a handful of hooks (identify,
read a sample, set ranges/filter/rate) and stores engineering-unit values into
`data_`. The base class does the rest.

**Unique features are not flattened away.** A capability only one chip has (the
MPU's aux-bus magnetometer, raw DLPF codes, FIFO, a DMP…) is added as extra
methods on that driver, not forced into the shared interface. Common things are
shared; special things stay special.

**One place for bus traffic.** Because every driver goes through `IMUBus`,
porting a sensor between I2C and SPI is a `begin*()` change, and debugging the
ArduinoNRF core's `Wire`/`SPI` drivers means watching a single layer.

## The bus layer (`IMUBus`)

`IMUBus` exposes register primitives (`readRegister`, `writeRegister`,
`readRegisters`, `writeRegisters`, `updateRegister`, `ping`) over either I2C or
SPI. Two details matter on this core:

- **32-byte bursts.** The ArduinoNRF `TwoWire` carries a 32-byte buffer (like
  classic Arduino). `readRegisters` splits longer reads into ≤32-byte,
  register-addressed bursts using a repeated-start, so a driver can request a
  full FIFO frame without truncation.
- **SPI read flag.** Register reads set the address MSB (InvenSense
  convention); pass a different flag to `beginSPI` for other chips.

## The data model

`update()` fills one `IMUData` snapshot (accel g, gyro deg/s, mag µT, temp °C,
`micros()` timestamp). Every accessor just reads that snapshot, so all values in
one loop iteration are from the same instant.

`IMUCalibration` holds per-axis bias and scale for accel, gyro and mag plus a
magic marker so EEPROM-restored values can be sanity-checked with `.valid()`.
