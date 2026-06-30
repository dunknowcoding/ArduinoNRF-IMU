# IMU Support and Usage

This document is the public reference for NiusIMU device support. It records
capabilities, interface limitations, module aliases, GPIO
expectations, and the Basic/Advanced example convention.

## Core Devices

| Include | Device | Accel | Gyro | Mag | Pressure |
| --- | --- | --- | --- | --- | --- |
| `<GY91.h>` | GY-91 (MPU-9250/MPU-6500 + BMP280) | Yes | Yes | Variant | Yes |
| `<GY9250.h>` | GY-9250 (MPU-9250) | Yes | Yes | AK8963 | No |
| `<MPU9250.h>` | MPU-9250 | Yes | Yes | AK8963 | No |
| `<MPU6500.h>` | MPU-6500 | Yes | Yes | No | No |
| `<BMP280.h>` | BMP280 | No | No | No | Yes |

Many GY-91 boards contain an MPU-6500 instead of an MPU-9250. The driver
detects the chip ID and `hasMagnetometer()` reports whether an AK8963 is
available.

## Additional Chips

| Include | Device | Accel | Gyro | Mag | Pressure | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| `<MPU9255.h>` | MPU-9255 | Yes | Yes | AK8963 | No | MPU-9250-compatible path |
| `<MPU6050.h>` | MPU-6050 | Yes | Yes | No | No | Auxiliary bypass available |
| `<MPU6886.h>` | MPU-6886 | Yes | Yes | No | No | MPU-6500-family register path |
| `<ICM20602.h>` | ICM-20602 | Yes | Yes | No | No | I2C/SPI |
| `<ICM20689.h>` | ICM-20689 | Yes | Yes | No | No | I2C/SPI |
| `<ICM20948.h>` | ICM-20948 | Yes | Yes | AK09916 | No | Bypass and auxiliary-master paths |
| `<ICM42688P.h>` | ICM-42688-P | Yes | Yes | No | No | I2C/SPI |
| `<ICM45686.h>` | ICM-45686 | Yes | Yes | Auxiliary | No | I2C/SPI, INT1/INT2, auxiliary I2C master |
| `<BMI088.h>` | BMI088 | Yes | Yes | No | No | Two I2C addresses or two SPI chip selects |
| `<BMI160.h>` | BMI160 | Yes | Yes | No | No | I2C/SPI |
| `<BMI270.h>` | BMI270 | Yes | Yes | No | No | Bosch configuration upload |
| `<BMI323.h>` | BMI323 | Yes | Yes | No | No | Word-addressed I2C/SPI |
| `<BMX055.h>` | BMX055 | Yes | Yes | BMM150 | No | BMA280 + BMG160 + BMM150 |
| `<BNO055.h>` | BNO055 | Yes | Yes | Yes | No | On-chip fusion |
| `<BNO08x.h>` | BNO085/BNO086 | Yes | Yes | Yes | No | SH-2/SHTP I2C fusion reports |
| `<LSM303DLHC.h>` | LSM303DLHC | Yes | No | Yes | No | Two I2C devices |
| `<LSM303D.h>` | LSM303D | Yes | No | Yes | No | Single-chip replacement found on GY-511 variants |
| `<LSM6DS3.h>` | LSM6DS3 | Yes | Yes | No | No | I2C/SPI |
| `<LSM6DS3TRC.h>` | LSM6DS3TR-C | Yes | Yes | Sensor hub | No | I2C/SPI, pedometer, tilt, significant motion |
| `<LSM6DSV.h>` | LSM6DSV | Yes | Yes | Sensor hub | No | I2C/SPI, SFLP fusion, INT1/INT2 |
| `<LSM6DSV320X.h>` | LSM6DSV320X | Dual low/high-g | Yes | Sensor hub | No | 320 g channel, SFLP, FSM/MLC/ASC |
| `<LSM6DSOX.h>` | LSM6DSOX | Yes | Yes | No | No | I2C/SPI |
| `<LSM9DS1.h>` | LSM9DS1 | Yes | Yes | Yes | No | I2C; dual-CS SPI requires a dedicated overload |
| `<QMI8658.h>` | QMI8658/QMI8658C | Yes | Yes | No | No | I2C/SPI |
| `<ADXL345.h>` | ADXL345 | Yes | No | No | No | Accelerometer |
| `<MMA8452Q.h>` | MMA8452Q | Yes | No | No | No | Accelerometer |
| `<ITG3205.h>` | ITG-3200/ITG-3205 | No | Yes | No | No | Gyroscope |
| `<L3G4200D.h>` | L3G4200D | No | Yes | No | No | Obsolete chip; clone risk |
| `<L3GD20.h>` | L3GD20/L3GD20H | No | Yes | No | No | Distinct IDs and addresses from L3G4200D |
| `<HMC5883L.h>` | HMC5883L | No | No | Yes | No | Frequently substituted by clones |
| `<QMC5883L.h>` | QMC5883L | No | No | Yes | No | Common HMC replacement |
| `<QMC6309.h>` | QMC6309 | No | No | Yes | No | Direct I2C magnetometer path |
| `<BMP180.h>` | BMP180/BMP085-compatible | No | No | No | Yes | Pressure and temperature |
| `<MS5611.h>` | MS5611 | No | No | No | Yes | Pressure and temperature |

## Board Modules

| Include | Board composition | Notes |
| --- | --- | --- |
| `<GY521.h>` | MPU-6050 | Single-chip alias |
| `<GY45.h>` | MMA8452Q | Single-chip alias |
| `<GY50.h>` | L3G4200D | Confirm `WHO_AM_I=0xD3`; clone/fault reports exist |
| `<GY63.h>` | MS5611 | Seven-pad I2C/SPI module; `PS` selects protocol |
| `<GY68.h>` | BMP180/BMP085 | Fixed address `0x77` |
| `<GY291.h>` | ADXL345 | Address and CS straps vary |
| `<GY511.h>` | LSM303DLHC | Some sellers substitute LSM303D |
| `<GYLSM6DS3.h>` | LSM6DS3 | Exact PCB silk must be matched before wiring |
| `<CJMCU633.h>` | LSM6DS3 | Purple CJMCU-633/CJMCU-603F full-pin board |
| `<LSM6DS3TRC.h>` | LSM6DS3TR-C | Blue eight-pad module uses the core header |
| `<LSM6DSV320X.h>` | LSM6DSV320X | Blue SCX/SDX module uses the core header |
| `<GYBNO055.h>` | BNO055 | Eight-pad I2C/UART board with INT and REST |
| `<GYBNO085.h>` | Ten-pad purple BNO08x board | Exact combined-label variant only |
| `<BNO08x.h>` | SparkFun-layout BNO086 clone | Dual Qwiic, SPI header, WAK/RST/INT |
| `<MUMO.h>` | ICM-45686 + QMC6309 | QMC6309 uses the ICM auxiliary I2C master |
| `<LSM6DSV.h>` + `<QMC6309.h>` | White LSM6DSV + QMC6309 board | QMC6309 may be direct-host or behind the sensor hub |
| `<ICM45686.h>` + `<QMC6309.h>` | White ICM-45686 + QMC6309 board | QMC6309 may be direct-host or behind the auxiliary master |
| `<CJMCU055.h>` | Purple CJMCU-055 9DOF BMX055 board | `PS`, two SDO, and three CS pads |
| `<ICM20948.h>` | Black ICM-20948 ten-pad board | Core header covers `INT`, `FSY`, `ACL`, and `ADA` |
| `<GY601N1.h>` | GY-601N1 with ICM-42688-P, ICM-45686, or BMI323 | One facade for the shared raw-I/O and MCU-side silk |
| `<ICM45686.h>` | ICM-45686 pin variants | Full-pin and eight-pad boards use the same core header |
| `<LSM303_L3GD20.h>` | LSM303DLHC + L3GD20/H | Adafruit-style 9DOF layout and marketplace clones |
| `<GY271.h>` / `<GY273.h>` | HMC5883L or QMC5883L | Driver probes both families |
| `<GY85.h>` | ADXL345 + ITG3205 + HMC/QMC | Multi-device board |
| `<GY80.h>` / `<GY801.h>` | ADXL345 + L3G4200D + HMC/QMC + BMP085/180 | Multi-device board |
| `<GY87.h>` | MPU-6050 + HMC/QMC + BMP180 | Compass uses MPU auxiliary bypass |
| `<GY86.h>` | MPU-6050 + HMC/QMC + MS5611 | Compass uses MPU auxiliary bypass |
| `<GY88.h>` | MPU-6050 + HMC/QMC + BMP085/180 | Marketplace composition varies |

## Important Limitations

- ICM-20948 I2C mode exposes its AK09916 through bypass mode. The one-CS SPI
  path reads accelerometer, gyroscope, and temperature; DMP firmware loading is
  outside this driver.
- BMI088 uses independent accelerometer and gyroscope addresses or chip-select
  pins. Its SPI path consumes the accelerometer's required dummy byte.
- BMI323 uses 16-bit register words, two I2C dummy bytes, and one SPI dummy
  byte. `routeDataReadyInterrupts()` can split accel and gyro across INT1/INT2.
- LSM6DSV320X supports I2C and SPI on ArduinoNRF nRF52840. The sensor also
  implements MIPI I3C, but nRF52840 has no I3C controller; `setI3CInterrupts()`
  configures the sensor-side I3C interrupt behavior and is not an I3C transport.
- BNO08x currently supports I2C. BNO08x SPI requires `CS`, `INT`, `RST`, and
  `WAKE/PS0`, so the one-CS common SPI signature is insufficient.
- LSM9DS1 SPI needs separate accel/gyro and magnetometer chip-select pins.
- HMC5883L names on inexpensive modules are not reliable. Use the board aliases
  that probe HMC and QMC variants where possible.

## GPIO and Silkscreen Policy

GPIO support is defined from the exact module PCB, not from the chip name in a
seller title.

1. A named board alias must identify the exact photographed or documented
   silkscreen variant it represents.
2. Combined labels such as `SCL/SCK/RX` are preserved exactly; they are not
   shortened to a preferred protocol name.
3. Exposed functional pins such as `INT1`, `INT2`, `DRDY`, `FSYNC`, `RST`,
   `WAKE`, auxiliary I2C, address-select, and chip-select are tracked as part of
   driver completeness.
4. Strap-only pins are documented even when software cannot change them after
   boot.
5. If a real module photograph, schematic, or manual cannot be found, the board
   remains a chip-level driver target and does not receive a guessed alias.

The ten-pad purple GY-BNO08x alias is limited to this exact header:

`VCC_3V3`, `GND`, `SCL/SCK/RX`, `SDA/MISO/TX`, `ADDR/MOSI`, `CS`, `INT`,
`RST`, `PS1`, `PS0`.

Physically different eleven-pad and SparkFun-style BNO086 boards must not use
that pin map merely because they carry the same sensor family.

### MUMO ICM-45686 + QMC6309

The open MUMO PCB has this exact silk:

`SCL`, `INT`, `CS`, `GND`, `+3V3`, `OSDO`, `OCS`, `CLK`, `SDA`, `CLK_CTL`

`INT` is ICM-45686 INT1. The board routes ICM INT2 to the `CLK` pad. `OSDO` and
`OCS` expose the auxiliary/OIS interface, while the onboard QMC6309 normally
connects to the ICM auxiliary I2C bus through the board jumpers. Some modules
use `OSDO` as their supply input and omit `CLK_CTL`; follow the labels printed
on the module in hand.

Use `<MUMO.h>` for the combined board. It configures the QMC6309 and includes
magnetic-field data in `update()`. Use `<ICM45686.h>` for the IMU alone and
`<QMC6309.h>` when the magnetometer is directly connected to the host bus.

Relevant ICM-45686 methods include:

```cpp
configureInterruptPin(1, activeLow, openDrain, latched);
routeDataReadyInterrupt(1, true, includeAux);
dataReady(1);
enableAuxI2CMaster();
setAuxAddress(address8);
auxReadRegister(reg, value);
auxWriteRegister(reg, value);
configureQMC6309();
```

### White LSM6DSV/ICM-45686 + QMC6309 Boards

The two white X-axis-marked modules use the same printed pad names even though
the center IMU differs:

```text
left:  OSDO  3V3  GND  SCL  SDA  CS  SDO
right: OCS   INT2 INT1 SCX  SDX
```

Use `<LSM6DSV.h>` for the LSM6DSV population and `<ICM45686.h>` for the
ICM-45686 population. `SCL/SDA/CS/SDO` form the host interface. `SCX/SDX` are
the sensor-hub/auxiliary clock and data lines. If the QMC6309 is wired to
those lines, call the IMU's `configureQMC6309()` and `readQMC6309()` methods.
If it is wired directly to the host I2C bus, create a separate `QMC6309`
object from `<QMC6309.h>`. `OCS/OSDO` belong to the second OIS SPI interface
and are not host-interface aliases.

The LSM6DSV API includes `sensorHubRead()`, `sensorHubWrite()`,
`configureQMC6309()`, and on-chip SFLP quaternion methods. See
`LSM6DSV_Fusion` for the FIFO-tagged game-rotation vector.
`configureOisInterface()` enables the OIS accel/gyro chain from the host UI,
and `LSM6DSV_OIS` demonstrates its ready status. The secondary OIS controller
is the SPI master on `OCS/SCX/SDX/OSDO`; those muxed pins cannot concurrently
serve as the sensor-hub bus.

### CJMCU-055 BMX055

The photographed purple CJMCU-055 board silk is:

`3.3V`, `GND`, `SCL`, `SDA`, `PS`, `SDO1`, `SDO2`, `CSB1`, `CSB2`, `CSB3`

`CSB1`, `CSB2`, and `CSB3` select the BMA280 accelerometer, BMG160 gyroscope,
and BMM150 magnetometer respectively. Call `CJMCU055::selectSPI()` before the
three-CS `beginSPI()` overload. In I2C mode, call `selectI2C()` and use the
three-address `beginI2C()` overload when strap-selected addresses differ.
`SDO1/SDO2` are shared serial-data/address functions, not interrupt outputs.

### CJMCU-633 / GY-LSM6DS3

The photographed purple full breakout is sold as CJMCU-633, CJMCU-603F, and
GY-LSM6DS3. Its exact silk is:

```text
left:  INT1 INT2 OCS SCX SDX +3.3V
right: CS   SDO  SCL SDA +3.3V GND
```

`SCL/SDA/CS/SDO` are the host I2C/SPI pins. `SCX/SDX` expose the LSM6DS3
sensor-hub bus, `OCS` is the auxiliary SPI chip select, and both interrupt
pads are covered by the routing APIs. Use `CJMCU633_Advanced` to exercise
INT1/INT2 and the sensor hub.

### LSM6DS3TR-C Blue Module

This is a separate board from the blue LSM6DSV320X module. Its exact visible
eight-pad silk is:

`INT2`, `INT1`, `CS`, `SDO`, `SDI`, `SCK`, `VCC`, `GND`

Use `LSM6DS3TRC_Basic` for ordinary samples, `LSM6DS3TRC_Advanced` for SPI
and both interrupt pads, and the focused `_Pedometer`, `_Tilt`, and
`_SignificantMotion` sketches for its embedded event engine.

### LSM6DSV320X Blue Module

The photographed V320X board exposes:

```text
left:  CS SDO SDI SCK VCC GND
right: INT1 INT2 SDX SCX
```

Use `<LSM6DSV320X.h>` and `LSM6DSV320X` for this and every other board carrying
the same core. The driver covers the low-g accelerometer, gyro, independent
32/64/128/256/320 g channel up to 7.68 kHz, high-g wake/shock routing,
SCX/SDX sensor hub, SFLP quaternions, MLC/FSM outputs, and ASC.

The complete ST register driver is bundled under the ST BSD-3-Clause license.
`stContext()` returns its initialized transport context, so every official
LSM6DSV320X register function is available without duplicating hundreds of
thin C++ wrappers:

```cpp
LSM6DSV320X imu;
imu.begin();
lsm6dsv320x_fifo_watermark_set(imu.stContext(), 16);
```

The focused examples cover the quad-channel paths: `_HighG`, `_Fusion`,
`_AI_Core`, `_FIFO`, `_EmbeddedEvents`, `_EIS_OIS`, and `_I3C_Config`.
`_AI_Core` accepts MEMS Studio extended-UCF commands and reads FSM, MLC, and
ASC state. Applications can also compile generated register tables and load
them through `loadUcf()` or `loadUcfExtended()`.

MIPI I3C 1.1 and IBI registers are accessible through the official API. The
nRF52840 itself has no I3C controller, so ArduinoNRF uses I2C or SPI for the
transport; `_I3C_Config` deliberately demonstrates sensor-side configuration
over I2C rather than presenting I2C as an I3C bus.

### ICM-45686 Standalone Variants

The purple full-pin module has:

```text
host edge: VIN 3V3 GND SCL SDA CS SDO
aux edge:  OCS INT2 INT1 SCX SDX
```

Both boards use `<ICM45686.h>` and `ICM45686`. The distinct black eight-pad
module has `VCC`, `GND`, `ADD/MISO`, `SDA/MOSI`, `SCL/SCLK`, `CS`, `INT1`,
and `INT2`. It does not expose SCX/SDX or OCS, so auxiliary methods simply do
not apply to that pin variant. `ICM45686_Advanced` covers SPI and interrupts.
On the full-pin board, `setAuxiliaryMode()` selects the mutually exclusive OIS
SPI, external-sensor I2C master, or I2C bypass use of AUX1. `ICM45686_OIS`
demonstrates OIS pad-mode selection, and `ICM45686_QMC6309` demonstrates a
QMC6309 behind the auxiliary master.

### GY-63 MS5611

The seven-pad module is printed `VCC`, `GND`, `SCL`, `SDA`, `CSB`,
`SDO`, `PS`. Call `GY63::selectI2C(PS_PIN)` before `begin()`, or drive
`PS` low with `selectSPI()` before `beginSPI(SPI, CSB_PIN)`.
`GY63_Advanced` demonstrates the complete SPI pin path.

### GY-801 / GY-80

The documented GY-801 header is `VCC_IN`, `3.3V`, `GND`, `SCL`, `SDA`,
`DRDY`, `A_INT1`, `T_INT1`, `P_XCLR`, `P_EOC`. The board exposes the
compass data-ready signal, ADXL345 INT1, L3G4200D INT1, and the BMP180 reset
and conversion-complete signals. `GY801_Advanced` routes the subdevice
interrupts and uses `configurePressurePins()`, `resetPressure()`, and
`pressureConversionComplete()`. GY-80 follows the same API; common revisions
use BMP085 where GY-801 uses its register-compatible BMP180 successor.

### GY-601N1 Variants

The raw-chip side is:

`VCC`, `GND`, `SCL/SCLK`, `SDA/SDI`, `SA0/SDO`, `CS`, `INT1`, `INT2`

The onboard-MCU side is:

`VCC`, `GND`, `RX/CL`, `TX/DA`, `INT`, `PS`

The same carrier is populated with ICM-42688-P, ICM-45686, or BMI323. Include
only `<GY601N1.h>`: `begin()`, `beginI2C()`, and `beginSPI()` probe those cores
and `core()`/`coreName()` report the result. Common sampling, range, filter,
calibration, and interrupt methods are forwarded to the detected core. Typed
accessors (`icm42688()`, `icm45686()`, and `bmi323()`) expose core-specific
features when the returned pointer is non-null.

`beginUART(Stream&)` and `readUART()` expose the MCU-produced stream without
assuming a packet format, because different onboard firmware may emit
different payloads. `driveProtocolSelect()` controls `PS` when the selected
module documentation specifies the required level. See `GY601N1_Basic`,
`GY601N1_Advanced`, and `GY601N1_UART`.

### ICM-20948 Ten-Pad Module

The photographed black board is printed, in order:

`VCC`, `GND`, `SCLK`, `SDI`, `NCS`, `ADDO`, `INT`, `FSY`, `ACL`, `ADA`

The board prints `ADDO`; it is the I2C address-select function commonly named
`AD0`. `FSY` is FSYNC. `ACL/ADA` are auxiliary I2C clock/data. Use
`<ICM20948.h>` for every pin variant. The driver provides
`enableAuxI2CMaster()`, `auxReadRegister()`, `auxWriteRegister()`, and
`auxPing()` in addition to bypass mode. See `ICM20948_Auxiliary`.

### SparkFun-Layout BNO086 Clone

The photographed red BNO086 board follows the SparkFun pin layout and has two
visible rows:

```text
top:    GND  3V3  SCK  SO  SI  CS  WAK  RST
bottom: PS0  PS1  GND  3V3 SDA SCL RST  INT
```

Use `<BNO08x.h>`. Call `configurePins(INT, RST, WAK)` before `beginI2C()`.
`interruptAsserted()`,
`hardwareReset()`, and `setAwake()` operate the active-low host signals. `PS0`
and `PS1` are boot protocol straps and must be set before reset.

### GY-BNO055

The exact eight-pad board is printed `VIN`, `GND`, `SCL/RX`, `SDA/TX`,
`ADO`, `INT`, `BOOT`, `REST`. The final label is `REST` on the PCB even
though its function is reset. `GYBNO055_Advanced` drives that reset pad and
reads/clears `INT`; `ADO` selects the I2C address and `BOOT` is a startup
strap.

### Interrupt and Auxiliary Pins

- ICM-20948: `configureInterruptPin()`, `setDataReadyInterrupt()`,
  `setFsyncInterrupt()`, bypass, and auxiliary-master methods cover `INT`,
  `FSYNC/FSY`, and `ACL/ADA`. `AD0/ADDO` and `CS/NCS` remain hardware straps.
- LSM6DS3: `configureInterruptPins()`, `routeInterrupt()`, and
  `setDataReadyInterrupt()` cover `INT1` and `INT2`. `SDO/SA0` and `CS` select
  address/protocol at the board.
- GY-50/L3G4200D: the exact observed header is `GND`, `SDA`, `SCL`, `VCC`,
  `SDO`, `CS`, `DR`, `INT`. `setDataReadyInterrupt()` routes `DR`, while
  `setThresholdInterrupt()` routes the programmable interrupt generator to
  `INT`; `configureInterruptPins()` sets polarity and output type.
- BMX055: the three subdevices have independent I2C addresses and SPI chip
  selects. The driver exposes three-CS SPI plus `read*Register()` and
  `write*Register()` for BMA280, BMG160, and BMM150 functions.

## Exposed-Pin API Coverage

`dataReady()` means a status-register check. It does not configure a physical
interrupt output unless a routing method is also called.

| Device family | Exposed functional pads | Driver API | Example |
| --- | --- | --- | --- |
| ADXL345 / GY-291 | `INT1`, `INT2`, `CS`, `SDO` | `configureInterruptPolarity()`, `routeInterrupt()`, `interruptSource()` | `ADXL345_Advanced`, `GY291_Advanced` |
| MMA8452Q / GY-45 | `INT1`, `INT2`, `SA0` | `configureInterruptPins()`, `routeInterrupt()` | `MMA8452Q_Advanced`, `GY45_Advanced` |
| ITG-3200/3205 | `INT`, `AD0` | `configureInterrupt()`, `setDataReadyInterrupt()`, `setPllReadyInterrupt()` | `ITG3205_Advanced` |
| L3G4200D / L3GD20 | `DR`, `INT`, `CS`, `SDO` | `configureInterruptPins()`, `setDataReadyInterrupt()`, `setThresholdInterrupt()` | `GY50_Advanced`, `L3GD20_Advanced` |
| LSM303DLHC | accel `INT1/INT2`, mag `DRDY` | `setAccelDataReadyInterrupt()`, `configureAccelInterruptPolarity()`, `magDataReady()` | `LSM303_L3GD20_Advanced` |
| BMI088 | accel `INT1/INT2`, gyro `INT3/INT4` | separate accel and gyro configure/route methods | `BMI088_Advanced` |
| BMI160 / BMI270 / BMI323 | `INT1`, `INT2` | `configureInterruptPin()`, `routeDataReadyInterrupt()` | each `*_Advanced` |
| ICM-20948 | `INT`, `FSYNC/FSY`, `ACL/ADA` | interrupt, FSYNC, bypass, and auxiliary-master methods | `ICM20948_Advanced`, `ICM20948_Auxiliary` |
| ICM-42688-P | `INT1`, pin-9 `INT2/FSYNC/CLKIN` | interrupt routing and `setPin9Function()` | `ICM42688P_Advanced` |
| ICM-45686 / MUMO / QMC board | `INT1`, `INT2/CLK`, `SCX/SDX`, OIS | interrupt and auxiliary-master methods | `ICM45686_Advanced`, `ICM45686_OIS`, `ICM45686_QMC6309`, `MUMO_Advanced` |
| LSM6DS3 / LSM6DS3TR-C / LSM6DSOX | `INT1`, `INT2`, `CS`, `SDO/SA0` | polarity, routing, pedometer, tilt, and motion events where supported | each `*_Advanced`, TR-C focused event sketches |
| LSM6DSV / QMC board | `INT1`, `INT2`, `SCX/SDX`, `OCS/OSDO` | interrupt, sensor-hub, QMC6309, OIS, and SFLP methods | `LSM6DSV_Advanced`, `_Fusion`, `_QMC6309`, `_OIS` |
| LSM6DSV320X module | `INT1`, `INT2`, `SCX/SDX`, `CS/SDO` | low/high-g, FIFO, EIS/OIS, sensor hub, SFLP, FSM/MLC/ASC, I3C registers | `LSM6DSV320X_Advanced` and focused examples |
| GY-63 MS5611 | `PS`, `CSB`, `SDO`, `SCL/SDA` | protocol selection plus I2C/SPI transports | `GY63_Advanced` |
| GY-80 / GY-801 | `DRDY`, `A_INT1`, `T_INT1`, `P_XCLR`, `P_EOC` | subdevice interrupt APIs plus pressure reset/EOC GPIO helpers | `GY80_Advanced`, `GY801_Advanced` |
| CJMCU-055 BMX055 | `PS`, `SDO1/2`, `CSB1/2/3` | protocol selector plus three-CS SPI | `CJMCU055_Advanced` |
| GY-601N1 | raw `INT1/2`, `CS`, `SA0`; MCU `RX/TX`, `INT`, `PS` | auto-detected ICM-42688-P/ICM-45686/BMI323 APIs and opaque UART stream | `GY601N1_Advanced`, `GY601N1_UART` |
| LSM9DS1 | AG `INT1/INT2`, magnetometer `INT`, dual address/CS | AG data-ready and magnetometer threshold methods | `LSM9DS1_Advanced` |
| QMI8658C | `INT1`, `INT2` | `setSynchronizedSampleMode()` routes data-ready to `INT2`; motion/handshake uses `INT1` | `QMI8658_Advanced` |
| HMC5883L / QMC5883L / QMC6309 | `DRDY` where broken out | `dataReady()` plus lock/overflow status | each `*_Advanced` |
| BNO055 | `RST`, `INT`, `ADR`, `PS0`, `PS1` | host-pin control, interrupt status/clear, page-register access | `BNO055_Advanced`, `BNO055_Calibration` |
| BNO08x | `RST`, `INT`, `WAK`, `PS0`, `PS1`, `BOOT` | host-pin control and active-low interrupt check | `BNO08x_HostPins` |
| MPU-6050 / GY-521 | `INT`, `FSYNC`, `XDA`, `XCL`, `AD0` | electrical INT control, data-ready, FSYNC tagging, and auxiliary bypass | `MPU6050_Advanced`, `GY521_Advanced` |
| MPU-6500 family | `INT`, `FSYNC`, auxiliary I2C, `AD0`, `CS` | electrical INT control, data-ready, FSYNC tagging, and auxiliary master/bypass APIs | family `*_Advanced` and GY-9250 focused examples |

Address, protocol, and boot-mode pads such as `AD0`, `SA0`, `SDO`, `CS`, `PS0`,
and `PS1` are sampled by hardware. Runtime software cannot change a pad that is
physically tied high or low, so the docs and board aliases preserve them as
straps rather than presenting fake setter methods.

## Calibration Workflows

### Ordinary Raw Sensors

`calibrateGyro()` measures stationary zero-rate bias. `calibrateAccel()` is a
single-resting-orientation bias correction; it is not a six-face scale and
cross-axis calibration. `calibrateMag()` uses rotating min/max hard-iron and
per-axis scale estimation where a magnetometer driver implements it. Store the
returned `IMUCalibration` in the application if it must survive reset.

### BNO055

BNO055 calibration is performed inside Bosch fusion firmware:

1. Keep the board still for several seconds for gyroscope calibration.
2. Hold all six faces stable, moving slowly between them, for accelerometer
   calibration.
3. Make slow figure-eight and full-axis rotations for magnetometer calibration.
4. Wait until system, gyro, accel, and mag status all read `3`.
5. Call `readCalibrationProfile()` to capture all 22 offset/radius bytes.
6. On later boots, call `writeCalibrationProfile()` with that exact profile.

Use `BNO055_Calibration` for the guided status/profile flow and `BNO055_Fusion`
for quaternion, Euler, linear-acceleration, and gravity outputs. Calibration
profiles must be captured and restored in configuration mode; the driver
handles those mode transitions.

### BNO085 / BNO086

BNO08x dynamic calibration and tare solve different problems. Dynamic
calibration estimates sensor corrections; tare changes the reported reference
orientation.

1. Call `beginCalibration()` after enabling calibrated sensor reports.
2. Hold still for gyro calibration, use 4-6 stable orientations for accel, and
   rotate roughly 180 degrees and back around all axes for magnetometer.
3. Monitor `accelAccuracy()`, `gyroAccuracy()`, and `magAccuracy()`.
4. When `calibrationComplete()` is true, call `saveCalibration()` to save DCD,
   then `endCalibration()`.
5. Apply `tareNow()` only after final mechanical mounting; use `saveTare()` if
   the reference orientation should persist.

Use `BNO08x_Calibration` for that sequence. `BNO08x_Fusion` demonstrates
rotation/game/geomagnetic vectors, linear acceleration, gravity, step count,
tap detection, stability classification, and personal activity classification.

## Embedded Processing

- BNO055 fusion outputs are demonstrated by `BNO055_Fusion`.
- BNO08x SH-2 fusion and classifiers are demonstrated by `BNO08x_Fusion`.
- LSM6DSOX MLC loading is implemented by `loadUcf()`. `LSM6DSOX_MLC` loads
  ST's BSD-licensed vertical angle-detection decision tree and reads MLC0.
- LSM6DSV320X SFLP is demonstrated by `LSM6DSV320X_Fusion`. Its
  `LSM6DSV320X_AI_Core` sketch accepts MEMS Studio extended-UCF commands,
  reads FSM/MLC outputs, and exposes ASC permission/status. Model tables remain
  application assets because their licensing and trained use case vary.
- BMI270 currently uses Bosch's maximum-FIFO configuration image. That image
  does not provide the standard gesture/context feature set, so those features
  are not advertised by this driver configuration.
- BMI323 exposes `enableFeatureEngine()` and word-register access, but an
  application must configure a specific Bosch feature before enabling it.

## Marketplace Variant Identification

Seller names are treated as hints. Probe IDs and addresses before selecting a
driver:

| Populated device | `WHO_AM_I` | Normal I2C addresses | Use |
| --- | --- | --- | --- |
| L3G4200D | `0xD3` | `0x68` / `0x69` | `<L3G4200D.h>` or `<GY50.h>` |
| L3GD20 | `0xD4` | `0x6A` / `0x6B` | `<L3GD20.h>` |
| L3GD20H | `0xD7` | `0x6A` / `0x6B` | `<L3GD20.h>` |
| LSM303DLHC accel | `0x33` | `0x18` / `0x19` | `<LSM303DLHC.h>` |
| LSM303D | `0x49` | `0x1D` / `0x1E` | `<LSM303D.h>` |

Several "9DOF L3G4200D" listings reproduce an Adafruit-style board populated
with LSM303DLHC plus L3GD20/L3GD20H. The product title may say L3G4200D while
the description or user identification reports L3GD20. Use
`<LSM303_L3GD20.h>` for that composition. Its Advanced example prints both
identities and configures the accel, magnetometer, and gyro interrupt paths.

Examples of this naming mismatch include an
[Adafruit-style clone user report](https://forum.arduino.cc/t/lsm303-l3gd20-accelerometer-not-working-right/566821)
and a [seller page whose title/specification says L3G4200D while its populated
device description says L3GD20](https://thinkrobotics.com/products/9dof-lsm303-l3g4200d-accelerometer-magnetometer-gyroscope).

Do not relax an L3G4200D identity check to accept `0xD4` or `0xD7`: the address
range is also different, and accepting all three under one name makes wiring
and clone diagnosis harder.

The photographed GY-511 is the two-address LSM303DLHC board. Its header is
`VIN`, `3.3V`, `GND`, `SCL`, `SDA`, `INT2`, `INT1`, `DRDY`.
Use `<GY511.h>` for that board. A standalone LSM303D remains supported through
`<LSM303D.h>`, but it is not presented as a GY-511 variant.

Calibration command details follow the
[Bosch BNO055 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf)
and [CEVA SH-2 reference manual](https://www.ceva-ip.com/wp-content/uploads/SH-2-Reference-Manual.pdf).
The bundled LSM6DSOX model comes from ST's
[STMems Machine Learning Core repository](https://github.com/STMicroelectronics/STMems_Machine_Learning_Core).

Module pin evidence used by the aliases includes the
[GY-63 seven-pin module and schematic](https://protosupplies.com/product/gy-63-ms5611-pressure-temperature-sensor-module/),
[GY-801 ten-pin listing](https://www.plexishop.it/en/10-axis-gy-801-module-gyroscope-accelerometer-magnetometer-pressure-sensor.html),
[GY-BNO055 listing photographs](https://www.jmaker.com.tw/products/gy-bno055-9dof-%E4%B9%9D%E8%BB%B8-%E6%84%9F%E6%B8%AC%E5%99%A8),
and the [SparkFun LSM6DS3 full-breakout hardware guide](https://learn.sparkfun.com/tutorials/lsm6ds3-breakout-hookup-guide/hardware-overview).

## Default ArduinoNRF I2C Wiring

| Sensor signal | ArduinoNRF board pad | nRF52 pin |
| --- | --- | --- |
| SDA | `SDA` (`D6`) | P1.00 |
| SCL | `SCL` (`D7`) | P0.11 |
| Power | `3V3` | 3.3 V |
| Ground | `GND` | Ground |

Use 3.3 V unless the exact breakout schematic proves that its regulator and
logic-level translation support another voltage.

## Common API

```cpp
bool begin();
bool beginI2C(TwoWire&, uint8_t address);
bool beginSPI(SPIClass&, uint8_t csPin);
uint8_t whoAmI();
bool isConnected();

bool update();
Vec3 accelG();
Vec3 accelMs2();
Vec3 gyroDps();
Vec3 gyroRps();
Vec3 magUT();
float temperatureC();
bool hasMagnetometer();

bool setAccelRangeG(uint16_t maxG);
bool setGyroRangeDps(uint16_t maxDps);
bool setLowPassFilterHz(uint16_t hz);
bool setSampleRateHz(uint16_t hz);

bool calibrateGyro();
bool calibrateAccel();
bool calibrateMag();
IMUCalibration getCalibration();
void setCalibration(const IMUCalibration& calibration);
```

Chip-specific methods remain available when the common interface cannot model
a feature, such as pressure, quaternion reports, interrupt routing, external
sync, auxiliary buses, FIFO controls, or multi-chip-select SPI.

## Example Convention

Every newly added sensor or named board uses this structure:

```text
examples/<NAME>/<NAME>_Basic/
examples/<NAME>/<NAME>_Advanced/
```

`Basic` contains only initialization, update, and essential value output.
Configuration, raw reads, interrupt routing, calibration, FIFO operation,
fusion reports, and other optional functionality belong in `Advanced`.

Browse the [examples directory](../examples) for the complete set. Start with
[`I2C_Scanner`](../examples/I2C_Scanner) when bringing up an unknown module.

## MPU-6500 / MPU-9250 Interfaces

The MPU-6500/MPU-9250 API includes:

- SPI configuration at 1 MHz and data reads up to the ArduinoNRF 8 MHz limit.
- Data-ready interrupt routing with `setDataReadyInterrupt(true)`.
- External synchronization with `setExternalSync(...)` and `fsyncLevel()`.

See the `GY9250_SPI`, `GY9250_DataReadyInterrupt`, and `GY9250_FSYNC`
examples for these paths.
