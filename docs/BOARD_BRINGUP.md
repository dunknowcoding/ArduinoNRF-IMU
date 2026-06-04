# Board bring-up & debugging log (ProMicro nRF52840 + GY-9250)

This library doubles as a debugging tool for the ArduinoNRF core. This file
records how the first sensor (GY-9250 over I2C) was brought up on real hardware,
what was found, and the techniques used — so the next bus/sensor can reuse them.

## Setup

- Board: AliExpress ProMicro nRF52840 (`arduinonrf:nrf52:promicro_nrf52840`),
  flashed and inspected with a SEGGER J-Link over the SWD pads.
- Sensor: GY-9250 (MPU-9250) on the default I2C pads (silkscreen SDA/SCL),
  powered from 3V3.

## Finding 1 — default I2C pins did not reach the connector (FIXED)

**Symptom.** An I2C scan on the default `Wire` bus found no devices, even though
the IMU was wired to the SDA/SCL pads.

**Root cause.** The board variant shipped a placeholder pin map: an identity
`g_ADigitalPinMap` (0..29) with `SDA=18`, `SCL=19`. Those resolve to nRF
P0.18/P0.19, which are **not broken out** on this board. The real silkscreen
pads are SDA=P1.00 and SCL=P0.11, and P1.00 (absolute pin 32) cannot even be
expressed by the 0..29 identity map.

**Evidence (read out of RAM over SWD).**

| State | `Wire.pinSDA()/SCL()` | raw nRF pins | scan result |
| ----- | --------------------- | ------------ | ----------- |
| before | 18 / 19 | P0.18 / P0.19 | 0 devices |
| after  | 30 / 11 | **P1.00 / P0.11** | **found 0x68** |

**Fix.** In the ArduinoNRF variant: append one `g_ADigitalPinMap` entry for
P1.00 and repoint the two `Wire` constants (`SDA=30 -> P1.00`, `SCL=11 ->
P0.11`). Nothing else changes, so the verified LED, analog and battery pins are
untouched. (See `docs/boards/promicro_nrf52840.md` in the ArduinoNRF repo.)

## Finding 2 — IMUBus and the core's TwoWire are sound

Reading WHO_AM_I via `nimu::IMUBus` (repeated-start: write register, repeated
start, read) returned `0x71` reliably on the first and every later transaction.
The driver's `update()` burst read path works the same way. No core change was
needed for register access.

> Note: the core's `TwoWire` carries a 32-byte buffer, so `IMUBus::readRegisters`
> chunks longer bursts into ≤32-byte register-addressed reads. Keep that in mind
> for FIFO-sized transfers.

## Technique — reading results over SWD without USB serial

USB-CDC enumeration on a freshly SWD-flashed image was unreliable, so results
were captured without it:

1. Put diagnostic output in a fixed global, e.g. `volatile T g_x __attribute__((used));`
2. Resolve its address from the `.elf` with `arm-none-eabi-nm`.
3. Flash with `JLink.exe -CommandFile` (`loadfile <hex>`).
4. Boot the app: `r; halt; WReg MSP <SP>; SetPC <ResetHandler>; go` where SP and
   the reset handler come from the app vector table at `mem32 0x26000 2`. The
   plain bootloader reset can land in DFU (double-reset detection), so jumping
   straight to the app entry is the reliable path.
5. `Sleep <ms>; halt; mem8 <addr> <len>` and decode the struct.

**Gotcha:** if you `halt` the MCU while an I2C transaction is in flight, the
slave can be left holding SDA low and the bus jams until the sensor is power-
cycled. Make diagnostics finish all I2C in `setup()` and leave `loop()` idle, so
halting to read RAM is always safe. The driver's `begin()` now also calls
`IMUBus::recoverBus()` (clock SCL until SDA releases, then STOP) to clear a bus
jammed by a previously crashed run.
