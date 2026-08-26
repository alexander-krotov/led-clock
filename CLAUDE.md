# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a single-sketch Arduino project (`led-clock.ino`) targeting an ESP32-based board (Ozobot RVDKit), despite the
"led-clock" name it currently only implements I2C sensor discovery/polling — there is no LED or clock-display driving
code yet, only an RTC (DS3231) being read for time.

## Build / upload

There is no build config checked in (no `sketch.yaml`, no `arduino-cli.yaml`). This project is normally opened and
compiled through the Arduino IDE (or Theia-based IDE — see `.theia/launch.json`, set up for OpenOCD/cortex-debug).
If using `arduino-cli` instead, you must know the target board's FQBN; the sketch itself only distinguishes between
two ESP32 variants at compile time (see Architecture below), so pick the `arduino-cli` FQBN accordingly, e.g.:

```
arduino-cli compile --fqbn esp32:esp32:esp32c3 .
arduino-cli upload -p <PORT> --fqbn esp32:esp32:esp32c3 .
```

There are no automated tests — this is embedded firmware, verified by flashing to hardware and reading serial output
at 115200 baud.

## Architecture

Everything lives in `led-clock.ino`. The chip target is selected at compile time via
`CONFIG_IDF_TARGET_ESP32C3` / `CONFIG_IDF_TARGET_ESP32S3` preprocessor guards near the top of the file, which set the
`PIN_SDA` / `PIN_SCL` / `PIN_IRQ` pin numbers for that board variant. Building for any other target fails with
`#error "Unknown target"`.

### Shared I2C bus

All five sensors sit on one I2C bus (`Wire`, pins `PIN_SDA`/`PIN_SCL`, 50kHz — lowered from the original 100kHz,
with `Wire.setTimeOut(25)` also set, for SGP30 stability). Generic helpers
(`i2cReadBytes`, `i2cReadRaw`, `i2cReadReg`, `i2cWriteReg`, `i2cProbe`, `i2cFindAddress`) wrap `Wire` calls and are
used by all the per-device sections instead of each device rolling its own I2C code. `i2cReadRaw` exists specifically
for devices like the AHT20 that return raw data streams rather than exposing addressable registers.

Because the AS3935 and BMP280/BME280 boards can have their I2C address strapped differently depending on the board,
`setup()` probes candidate addresses for those two (`i2cFindAddress`) rather than assuming a fixed address, and for
the BMx280 additionally reads the chip ID register (0xD0) to distinguish a BMP280 (0x58, no humidity) from a BME280
(0x60, has humidity) and picks the compensation/readout path accordingly. DS3231 and AHT20 use fixed addresses
(0x68, 0x38).

### Per-sensor sections

The file is organized into clearly delimited sections (see the `// ----` banners), each following the same
`initX()` / `readX()` (or `handleXIrq()`) pattern, gated by a `xPresent` bool set during `initX()`:

- **AS3935 lightning sensor** — `initAs3935()` resets/recalibrates the RC oscillators (`calibrateAs3935`) and attaches
  a `RISING`-edge interrupt on `PIN_IRQ` (`onAs3935Irq`, sets the `volatile irqFired` flag). `handleAs3935Irq()` is
  polled from `loop()` and decodes the interrupt source register (noise / disturber / lightning-with-distance-and-energy).
  Currently disabled — `initAs3935()` is commented out in `setup()`.
- **DS3231 RTC** — `initDs3231()`/`readDs3231()`. Time registers are BCD-encoded (`bcdToDec`); assumes the RTC is
  already configured for 24-hour mode. `readDs3231()` also reads the chip's internal die temperature from registers
  0x11/0x12 (signed integer °C in the MSB, plus 0.25°C steps from the top 2 bits of the LSB) and logs it alongside
  the time.
- **AHT20 temperature/humidity** — `initAht20()`/`readAht20()`. Uses raw command bytes (0xBE calibrate, 0xAC measure)
  rather than addressed registers; readings are 20-bit fixed-point values reconstructed from a 6-byte raw response.
- **BMP280/BME280 pressure/temperature(/humidity)** — the most involved section. `initBmx280()` reads factory
  calibration constants into `Bmx280Calib`, and `bmx280CompensateTemperature/Pressure/Humidity()` implement the
  integer compensation formulas straight from the Bosch datasheet (temperature must be compensated first since it
  produces the shared `bmx280TFine` value used by the pressure/humidity formulas).
- **SGP30 eCO2/TVOC gas sensor** — fixed address 0x58 (unrelated to the BMP280 chip-id value of the same number).
  `initSgp30()` sends the `init_air_quality` command; `readSgp30()` sends `measure_air_quality` and reads back two
  CRC-8-checked 16-bit words (eCO2, TVOC) via `sgp30ReadWords`/`sgp30Crc8`. Unlike the other sensors it isn't
  addressed-register based, so it talks to `Wire` directly (`sgp30SendCommand`) the same way AHT20 does. Polled on
  its own `SGP30_MEASURE_INTERVAL_MS` (1000ms) timer, separate from `SENSOR_POLL_INTERVAL_MS`, because Sensirion's
  datasheet requires calling `measure_air_quality` once per second for the sensor's dynamic baseline compensation to
  stay accurate.

### Main loop

`setup()` brings up `Wire`, runs a full `scan_i2c()` bus scan once, then calls each `initX()`. `loop()` no longer
re-scans the I2C bus or blocks on a `delay()` — it services the AS3935 IRQ flag every iteration and polls sensor
readings on non-blocking `millis()`-based intervals: `SENSOR_POLL_INTERVAL_MS` (2000ms) for DS3231/AHT20/BMx280, and
a separate `SGP30_MEASURE_INTERVAL_MS` (1000ms) timer for SGP30.
