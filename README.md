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
- **Pluggable sensors as folders.** Drop a driver in `src/sensors/<NAME>/`, add a
  one-line forwarding header, and `#include <NAME.h>` works. See
  [docs/ADDING_A_SENSOR.md](docs/ADDING_A_SENSOR.md).
- **Chip-specific power kept available.** The MPU-9250 driver still exposes its
  magnetometer control, raw DLPF codes, data-ready interrupt and raw registers.

## Supported sensors

| Sensor | Bus | Accel | Gyro | Mag | Driver |
| ------ | --- | ----- | ---- | --- | ------ |
| MPU-9250 / GY-9250 | I2C, SPI* | ✅ | ✅ | ✅ (AK8963) | [`src/sensors/MPU9250`](src/sensors/MPU9250) |

\* On SPI only accel + gyro are read; the magnetometer is wired up for the I2C
path. Over I2C the AK8963 is reached through the MPU's internal I2C master (not
bypass), so a dead/absent magnetometer can never jam the accel/gyro bus.

## Install

This library lives next to the ArduinoNRF package. Either copy this folder into
your Arduino `libraries/` directory, or compile an example directly:

```sh
arduino-cli compile \
  --fqbn arduinonrf:nrf52:promicro_nrf52840 \
  --library <path-to>/ArduinoNRF-IMU \
  <path-to>/ArduinoNRF-IMU/examples/MPU9250/MPU9250_Basic
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
| [`MPU9250/MPU9250_Basic`](examples/MPU9250/MPU9250_Basic) | Minimal read loop |
| [`MPU9250/MPU9250_Calibration`](examples/MPU9250/MPU9250_Calibration) | Guided gyro/accel/mag calibration that prints paste-ready code |
| [`MPU9250/MPU9250_Advanced`](examples/MPU9250/MPU9250_Advanced) | Custom ranges, DLPF, sample rate, data-ready, raw registers |

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
