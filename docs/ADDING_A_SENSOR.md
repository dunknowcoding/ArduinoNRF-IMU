# Adding a new IMU

Every sensor is "mounted" the same way: a folder under `src/sensors/`, a class
that implements the shared interface, and a one-line public header. Here is the
whole recipe.

## 1. Create the folder

```
src/sensors/MYIMU/
    MYIMU.h
    MYIMU.cpp
    MYIMU_Registers.h    (optional but recommended)
```

## 2. Implement the interface

Derive from `nimu::IMUSensor` and implement the pure-virtual hooks. The base
class already provides units, the snapshot, and generic gyro/accel calibration.

```cpp
#include "../../imu/IMUSensor.h"

namespace nimu {
class MYIMU : public IMUSensor {
 public:
  MYIMU() { name_ = "MYIMU"; hasMag_ = false; }

  bool begin() override;                              // default bus + defaults
  bool beginI2C(TwoWire& w, uint8_t addr) override;
  bool beginSPI(SPIClass& s, uint8_t cs) override;
  uint8_t whoAmI() override;
  bool isConnected() override;
  bool update() override;                             // fill data_ in real units
  bool setAccelRangeG(uint16_t) override;
  bool setGyroRangeDps(uint16_t) override;
  bool setLowPassFilterHz(uint16_t) override;
  bool setSampleRateHz(uint16_t) override;
};
}  // namespace nimu

using nimu::MYIMU;   // let sketches write MYIMU without the namespace
```

### What `update()` must do

1. Read the chip's data registers via `bus_` (never touch `Wire`/`SPI`
   directly).
2. Convert to **g**, **deg/s**, **µT**, **°C**.
3. Apply calibration with the shared helper and store into `data_`:

```cpp
Vec3 a = /* accel in g */;
data_.accel = correct(a, cal_.accelBias, cal_.accelScale);
Vec3 g = /* gyro in deg/s */;
data_.gyro  = correct(g, cal_.gyroBias, Vec3{1,1,1});
data_.temperature = /* °C */;
data_.timestamp = micros();
```

That is all the base class needs — every accessor and unit conversion then works
for free.

## 3. Add the public forwarding header

Create `src/MYIMU.h` at the `src/` root (Arduino only puts `src/` on the include
path, not its subfolders):

```cpp
#ifndef ARDUINONRF_IMU_PUBLIC_MYIMU_H
#define ARDUINONRF_IMU_PUBLIC_MYIMU_H
#include "sensors/MYIMU/MYIMU.h"
#endif
```

Now sketches can `#include <MYIMU.h>`.

## 4. Sensor-specific extras (optional)

Anything unique to your chip — a FIFO, a DMP, an extra interrupt mode — goes on
your class as additional public methods. Do **not** widen `IMUSensor` for it;
the shared interface stays minimal so every IMU keeps an identical core API.

## 5. Examples and keywords

- Add `examples/MYIMU/MYIMU_Basic/MYIMU_Basic.ino` (plus calibration/advanced as
  needed), mirroring the MPU-9250 examples.
- Add your class and any new methods to `keywords.txt`.
- Add a row to the support table in `README.md`.

## 6. Verify

```sh
arduino-cli compile --fqbn arduinonrf:nrf52:promicro_nrf52840 \
  --library . examples/MYIMU/MYIMU_Basic
```

Then run `examples/I2C_Scanner` on hardware to confirm the address, and the
basic example to confirm live data.
