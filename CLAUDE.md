# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a single-sketch Arduino project (`led-clock.ino`) targeting an ESP32-based board (Ozobot RVDKit). It
implements I2C sensor discovery/polling, an RTC (DS3231) being read for time, and now also drives a MAX7219 LED
matrix display (via SPI, independent of the I2C sensor bus) that shows the current time as read from the DS3231.

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
(`i2cReadBytes`, `i2cReadRaw`, `i2cReadReg`, `i2cWriteReg`, `i2cProbe`, `i2cFindAddress`) wrap `Wire` calls. The
AS3935 section (the only sensor without a maintained Arduino library) talks to its registers directly through these
helpers; the other four sensors are driven through per-device libraries (see Sensor libraries below), but still use
`i2cProbe`/`i2cFindAddress` for presence/address detection before handing off to the library, since none of those
libraries scan the bus or report "not found" on their own in a way this sketch relies on.

Because the AS3935 and BMP280/BME280 boards can have their I2C address strapped differently depending on the board,
`setup()` probes candidate addresses for those two (`i2cFindAddress`) rather than assuming a fixed address, and for
the BMx280 additionally reads back the chip ID (via `Adafruit_BME280::sensorID()`) to distinguish a BMP280 (0x58, no
humidity) from a BME280 (0x60, has humidity) and picks the read path accordingly. DS3231 and AHT20 use fixed
addresses (0x68, 0x38); the DS3231 library doesn't expose its address, so `DS3231_ADDR` is redeclared locally in the
sketch just for the presence probe.

### Sensor libraries

Four of the five sensors are driven through third-party Arduino libraries rather than hand-rolled register access
(install via Library Manager if missing):

- **DS3231** (Eric Ayars' `DS3231` library, `DS3231.h`) — `DS3231 rtc;` with `rtc.getHour()/getMinute()/getSecond()/
  getYear()/getMonth()/getDate()/getTemperature()`. It doesn't expose a `begin()` or the I2C address, so
  presence is still established with a manual `i2cProbe(DS3231_ADDR)` before any `rtc.*` calls are trusted.
- **Adafruit AHTX0** (`Adafruit_AHTX0.h`, depends on Adafruit Unified Sensor) — `Adafruit_AHTX0 aht;`,
  `aht.begin()` then `aht.getEvent(&humidity, &temp)` filling `sensors_event_t` structs. Presence is
  `i2cProbe(AHT20_ADDR) && aht.begin()` so a failed `begin()` (not just a failed bus probe) also clears the
  present flag.
- **Adafruit BME280 Library** (`Adafruit_BME280.h`, depends on Adafruit Unified Sensor) — `Adafruit_BME280 bme;`,
  `bme.begin(addr)` against whichever of 0x76/0x77 `i2cFindAddress` found, then `bme.sensorID()` to tell a BMP280
  apart from a BME280 (see above) and `bme.readTemperature()/readPressure()/readHumidity()` (humidity only called
  when a BME280 was detected — a BMP280 has no humidity element and returns NaN for it).
- **Adafruit SGP30** (`Adafruit_SGP30.h`, depends on Adafruit Unified Sensor) — `Adafruit_SGP30 sgp;`, `sgp.begin()`
  then, once per second, `sgp.IAQmeasure()` (its bool return must be checked — a failed measurement leaves
  `sgp.eCO2`/`sgp.TVOC` stale rather than updating them) followed by reading the `sgp.eCO2`/`sgp.TVOC` members.
- **Adafruit Unified Sensor** (`Adafruit_Sensor.h`) — not a sensor driver itself, just the shared `sensors_event_t`/
  `Adafruit_Sensor` base type that AHTX0 and BME280 build on.

The AS3935 has no such library in use here, so it stays on the raw `i2cReadReg`/`i2cWriteReg` helpers described
above.

- **MD_Parola** / **MD_MAX72xx** (both by MajicDesigns, `MD_Parola.h`/`MD_MAX72xx.h`) — drive the MAX7219 LED matrix
  display over bit-banged SPI (`PIN_MAX7219_DATA`/`PIN_MAX7219_CLK`/`PIN_MAX7219_CS`, fixed at GPIO7/6/10 regardless
  of board target). Not gated behind a presence probe like the I2C sensors — it's on its own SPI-style bus, not the
  shared `Wire` bus.

### Per-sensor sections

The file is organized into clearly delimited sections (see the `// ----` banners), each following the same
`initX()` / `readX()` (or `handleXIrq()`) pattern, gated by a `xPresent` bool set during `initX()`:

- **AS3935 lightning sensor** — `initAs3935()` resets/recalibrates the RC oscillators (`calibrateAs3935`) and attaches
  a `RISING`-edge interrupt on `PIN_IRQ` (`onAs3935Irq`, sets the `volatile irqFired` flag). `handleAs3935Irq()` is
  polled from `loop()` and decodes the interrupt source register (noise / disturber / lightning-with-distance-and-energy).
  Currently disabled — `initAs3935()` is commented out in `setup()`.
- **DS3231 RTC** — `initDs3231()`/`readDs3231()`, using the `DS3231` library (see Sensor libraries above) rather than
  reading BCD time registers directly. `readDs3231()` also reads the chip's internal die temperature via
  `rtc.getTemperature()` and logs it alongside the time.
- **AHT20 temperature/humidity** — `initAht20()`/`readAht20()`, using Adafruit AHTX0 (`aht.begin()`/`aht.getEvent()`)
  rather than the raw command-byte sequence (0xBE calibrate, 0xAC measure) the sketch used before adopting the
  library.
- **BMP280/BME280 pressure/temperature(/humidity)** — `initBmx280()` probes 0x76/0x77 (`i2cFindAddress`), calls
  `bme.begin(addr)` (Adafruit BME280 Library), and uses `bme.sensorID()` to tell a BMP280 (0x58, no humidity) from a
  BME280 (0x60, has humidity) so `readBmx280()` knows whether to call `bme.readHumidity()`. The manual Bosch
  compensation-formula implementation this sketch used before adopting the library has been replaced by the
  library's own `readTemperature()/readPressure()/readHumidity()`.
- **SGP30 eCO2/TVOC gas sensor** — fixed address 0x58 (unrelated to the BMP280 chip-id value of the same number).
  `initSgp30()` calls `sgp.begin()` (Adafruit SGP30 library); `readSgp30()` calls `sgp.IAQmeasure()` once per second
  and, on success, reads back `sgp.eCO2`/`sgp.TVOC`. A failed `IAQmeasure()` is logged and skipped rather than
  logging stale values. Polled on its own `SGP30_MEASURE_INTERVAL_MS` (1000ms) timer, separate from
  `SENSOR_POLL_INTERVAL_MS`, because Sensirion's datasheet requires calling `measure_air_quality` once per second for
  the sensor's dynamic baseline compensation to stay accurate.
- **MAX7219 LED matrix clock display** — `initMax7219()`/`pollMax7219()`, using MD_Parola/MD_MAX72xx (see Sensor
  libraries above). Four 8x8 modules are addressed as one MD_Parola zone per HH:MM digit (`MAX7219_NUM_ZONES` = 4;
  zone 3 = hours-tens down to zone 0 = minutes-units), each fed from the DS3231 RTC (`rtc.getHour()`/`getMinute()`)
  once a second. A digit that changes scrolls down (`PA_SCROLL_DOWN`) while unchanged digits stay static; the colon
  between hours and minutes has no Parola glyph, so it's drawn by poking two column bytes directly through
  `MD_MAX72XX::setColumn()` (`drawMax7219Colon()`), redrawn every tick since Parola's own redraws can overwrite it.
  Unlike the other sections there's no presence flag — the display isn't probed, and `pollMax7219()` only skips the
  RTC read (not the animation tick, needed for smooth scrolling) when `ds3231Present` is false. This section was
  adapted from a standalone example sketch that also had NTP time sync, temperature display, and a physical
  brightness button on GPIO5; all three were dropped when merging — NTP/temperature because this project already has
  a real-time DS3231 and its own temperature sensors, and the button specifically because GPIO5 is `PIN_IRQ` on the
  ESP32-C3 variant and must stay reserved for the AS3935 interrupt.

### Main loop

`setup()` brings up `Wire`, runs a full `scan_i2c()` bus scan once, then calls each `initX()` (including
`initMax7219()`). `loop()` no longer re-scans the I2C bus or blocks on a `delay()` — it services the AS3935 IRQ flag
and ticks the MAX7219 display (`pollMax7219()`) every iteration, and polls sensor readings on non-blocking
`millis()`-based intervals: `SENSOR_POLL_INTERVAL_MS` (2000ms) for DS3231/AHT20/BMx280, and a separate
`SGP30_MEASURE_INTERVAL_MS` (1000ms) timer for SGP30. The MAX7219 display keeps its own internal 1000ms timer
(inside `pollMax7219()`) for pushing a fresh HH:MM, decoupled from both of the above since it must also tick Parola's
animation every loop iteration regardless of that timer.
