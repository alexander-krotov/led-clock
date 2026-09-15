# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a single-sketch Arduino project (`led-clock.ino`) targeting an ESP32-based board (Ozobot RVDKit). It
implements I2C sensor discovery/polling, an RTC (DS3231) used as the time source, drives a MAX7219 LED matrix
display (via SPI, independent of the I2C sensor bus) that shows the current time as read from the DS3231, connects to
WiFi at boot via WiFiManager (captive-portal provisioning, credentials stored in NVS), keeps the DS3231 disciplined
to an NTP server (name + UTC offset configurable in the portal, persisted in EEPROM), and exposes an unauthenticated
MCP (Model Context Protocol) HTTP endpoint for reading the sensors / RTC / config and changing the NTP config and
display brightness.

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
at 115200 baud. The one exception is `test/mcp-test.sh`, a curl-based smoke test for the MCP server
(see MCP below) that runs from any Linux/WSL shell against a running device:
`MCP_HOST=<ip>:8080 bash test/mcp-test.sh` (`jq` optional).

The full sketch is close to the 1.3 MB app limit of the default partition scheme (~94%); flash with a
larger-app partition scheme (e.g. "Huge APP") if it stops fitting.

## Architecture

Everything lives in `led-clock.ino`. The chip target is selected at compile time via
`CONFIG_IDF_TARGET_ESP32C3` / `CONFIG_IDF_TARGET_ESP32S3` preprocessor guards near the top of the file, which set the
`PIN_SDA` / `PIN_SCL` / `PIN_IRQ` pin numbers for that board variant. Building for any other target fails with
`#error "Unknown target"`.

### Shared I2C bus

All five sensors sit on one I2C bus (`Wire`, pins `PIN_SDA`/`PIN_SCL`, 50kHz — lowered from the original 100kHz,
with `Wire.setTimeOut(25)` also set, for SGP30 stability). Two generic helpers (`i2cProbe`, `i2cFindAddress`) wrap
`Wire` calls. All five sensors are driven through per-device libraries (see Sensor libraries below), but still use
`i2cProbe`/`i2cFindAddress` for presence/address detection before handing off to the library, since none of those
libraries scan the bus or report "not found" on their own in a way this sketch relies on.

Because the BMP280/BME280 board can have its I2C address strapped differently depending on the board, `setup()`
probes both candidate addresses for it (`i2cFindAddress`) rather than assuming a fixed address, and for the BMx280
additionally reads back the chip ID (via `Adafruit_BME280::sensorID()`) to distinguish a BMP280 (0x58, no
humidity) from a BME280 (0x60, has humidity) and picks the read path accordingly. DS3231, AHT20 and AS3935 use fixed
addresses (0x68, 0x38, 0x03); the DS3231 library doesn't expose its address, so `DS3231_ADDR` is redeclared locally
in the sketch just for the presence probe.

### Sensor libraries

All five sensors are driven through third-party Arduino libraries rather than hand-rolled register access
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
- **SparkFun AS3935 Lightning Detector** (`SparkFun_AS3935.h`) — `SparkFun_AS3935 as3935(AS3935_ADDR);` constructed
  with the fixed I2C address `defAddr` (0x03, the WCMCU-3935 board straps both address pins HIGH). `as3935.begin(Wire)`
  returns whether the chip ACKs; `initAs3935()` still runs `i2cProbe(AS3935_ADDR)` first so a bus miss is reported the
  same way as the other sensors. After `begin()` it calls `resetSettings()`, `calibrateOsc()` (RC oscillator
  calibration against the antenna LCO — logs a warning on failure but continues), and `setIndoorOutdoor(INDOOR)`.
  `handleAs3935Irq()` reads `readInterruptReg()` (which itself waits the datasheet-mandated 2ms settle time) and, on a
  `LIGHTNING` result, `distanceToStorm()` / `lightningEnergy()`. The `NOISE_TO_HIGH` / `DISTURBER_DETECT` / `LIGHTNING`
  enum values (0x01/0x04/0x08) come from the library header.

- **MD_Parola** / **MD_MAX72xx** (both by MajicDesigns, `MD_Parola.h`/`MD_MAX72xx.h`) — drive the MAX7219 LED matrix
  display over bit-banged SPI (`PIN_MAX7219_DATA`/`PIN_MAX7219_CLK`/`PIN_MAX7219_CS`, fixed at GPIO7/6/10 regardless
  of board target). Not gated behind a presence probe like the I2C sensors — it's on its own SPI-style bus, not the
  shared `Wire` bus.

- **WiFiManager** (tzapu, `WiFiManager.h`, pulls in the ESP32 core `WiFi.h` and — because `WM_MDNS` is `#define`d
  before the include — `ESPmDNS.h`) — `initWifi()` sets STA mode, then `wm.setHostname("led-clock")` and
  `wm.autoConnect("led-clock")` with a 60s `setConfigPortalTimeout`; on failure to join a known network it serves a
  captive portal from a `led-clock` AP. Two `WiFiManagerParameter`s (`"ntp"`, `"utc"`) are added to the portal so
  the NTP server name and UTC offset can be entered alongside the WiFi credentials; `setSaveParamsCallback` copies
  them into `ntpServer` / `utcOffsetHours` and calls `saveConfig()`. With `WM_MDNS` set, `setHostname()` also makes
  WiFiManager start an mDNS responder, so the clock answers to `led-clock.local` (ESP32 runs mDNS in the background —
  no `loop()` tick needed). WiFi credentials persist in the ESP32 NVS (the library's own storage), separate from the
  sketch's own EEPROM config.
- **`EEPROM.h`** / **`WiFiUdp.h`** (ESP32 core) — `EEPROM` backs the persistent NTP config (see the EEPROM section
  below); `WiFiUDP` carries the SNTP request in the NTP section.
- **`WebServer.h`** (ESP32 core) + **ArduinoJson** v7 — serve the MCP endpoint (see the MCP section). The core's
  *synchronous* `WebServer` is used (not ESPAsyncWebServer — the installed `ESPAsyncWebServer 3.1.0` / `AsyncTCP 1.1.4`
  don't compile against ESP32 core 3.x: their `WebRequestMethod` enum clashes with the core's `http_parser.h`).
  `mcpServer.handleClient()` is pumped from `loop()`, so handlers run in the loop task. ArduinoJson v7's elastic
  `JsonDocument` is used for both request parsing and response building.

### Per-sensor sections

The file is organized into clearly delimited sections (see the `// ----` banners), each following the same
`initX()` / `readX()` (or `handleXIrq()`) pattern, gated by a `xPresent` bool set during `initX()`:

- **AS3935 lightning sensor** — `initAs3935()` (using the SparkFun AS3935 library, see Sensor libraries above) probes
  the bus, calls `as3935.begin()`, resets settings, recalibrates the RC oscillators (`calibrateOsc()`), selects the
  indoor profile, sets the `as3935Present` flag, and attaches a `RISING`-edge interrupt on `PIN_IRQ` (`onAs3935Irq`,
  sets the `volatile irqFired` flag). `handleAs3935Irq()` is polled from `loop()`, guarded by `as3935Present`, and
  decodes the interrupt source (noise / disturber / lightning-with-distance-and-energy) through the library.
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
  libraries above). Four 8x8 modules are split into two MD_Parola zones (`MAX7219_NUM_ZONES` = 2): the left zone
  (`MAX7219_ZONE_LEFT` = 1, modules 2-3) shows `"HH"` right-aligned, the right zone (`MAX7219_ZONE_RIGHT` = 0,
  modules 0-1) shows `":MM"` left-aligned, so the colon (the right zone's leading glyph) sits at the middle of the
  chain and reads as the HH:MM separator (no hand-drawn colon any more). Both zones are fed from the DS3231 RTC
  (`rtc.getHour()`/`getMinute()`) once a second; a zone whose text changed scrolls the new text down
  (`PA_SCROLL_DOWN`) while the other stays static (`PA_PRINT`/`PA_NO_EFFECT`). Per-zone alignment is stored in
  `Max7219ZoneState.align` and reused for the static redraw. Parola keeps a pointer to (not a copy of) the text it is
  shown, so each zone's `current`/`next` char buffers live in the file-scope `max7219Zones[]` array rather than on
  the stack. Because MD_Parola only positions text at whole-module (8px) granularity, `"HH"` would otherwise be flush
  against the colon; `applyMax7219LeftNudge()` shifts the left zone's two modules `MAX7219_LEFT_NUDGE_COLS` (1)
  column(s) sideways with `MD_MAX72xx::transform()` (direction `MAX7219_LEFT_NUDGE_XFORM`, `TSL`/`TSR` depending on
  module wiring) to open a small gap before the colon. The shift has to be redone after every static redraw of the
  zone, so `max7219LeftNudgePending` is set on each redraw and `pollMax7219()` re-applies the nudge once the zone is
  idle again. Unlike the other sections there's no presence flag — the display isn't probed, and `pollMax7219()` only
  skips the RTC read (not the animation tick, needed for smooth scrolling) when `ds3231Present` is false. This section was
  adapted from a standalone example sketch that also had NTP time sync, temperature display, and a physical
  brightness button on GPIO5; all three were dropped when merging — temperature because this project has its own
  sensors, the button because GPIO5 is `PIN_IRQ` on the ESP32-C3 variant and must stay reserved for the AS3935
  interrupt, and NTP because the DS3231 was the time source (NTP has since been re-added as its own section, keeping
  the DS3231 as the source and just disciplining it).
- **Persistent config (EEPROM)** — `loadConfig()` / `saveConfig()`, run over the ESP32 EEPROM emulation
  (`EEPROM.begin(EEPROM_SIZE)` first thing in `setup()`). Layout is a magic byte (`EEPROM_MAGIC`, detects an
  uninitialized/foreign partition), an `int8_t` UTC offset, a fixed-length NUL-terminated `ntpServer[]` byte array
  (written/read with `EEPROM.writeBytes`/`readBytes`), and a `uint8_t` display brightness (`EEPROM_BRIGHTNESS_ADDR`,
  0..15). `loadConfig()` seeds the defaults (`"fi.pool.ntp.org"`, offset 0, brightness 4) on first boot and, since
  the brightness byte was added without bumping `EEPROM_MAGIC`, treats an out-of-range value (e.g. `0xFF` from an
  older layout) as "use the default". Writers: the WiFiManager save-params callback and the MCP `set_ntp_config` /
  `set_display_brightness` tools.
- **WiFi** — `initWifi()` (see WiFiManager under Sensor libraries above). Sets `WIFI_STA` mode, calls
  `WiFiManager::setHostname("led-clock")` and `autoConnect("led-clock")` with a 60s config-portal timeout, recording
  the outcome in the file-scope `wifiConnected` flag. It also registers the `"ntp"` / `"utc"` portal parameters and a
  save callback (see WiFiManager above). `WM_MDNS` is `#define`d before the WiFiManager include, so the device is also
  reachable at `led-clock.local` over mDNS. This is the one `initX()` that can block `setup()` for a long time: if no
  known network is reachable the captive portal runs until the user configures WiFi or `WIFI_MANAGER_TIMEOUT_S`
  elapses, and the clock display does not tick during that window.
  When the connection succeeds, `setup()` calls `showIpOnMax7219()`, which scrolls `WiFi.localIP()` once
  right-to-left across the whole four-module chain and then restores the clock layout. It does this by blanking the
  left zone, widening zone 0 (`MAX7219_ZONE_RIGHT`) to span all `MAX7219_MAX_DEVICES` modules, running a
  `PA_SCROLL_LEFT` in/out effect, and pumping `displayAnimate()` synchronously (`pumpMax7219UntilIdle()`, with a
  timeout) until the scroll finishes, before calling `initMax7219Zones()` to put the two-zone HH / :MM layout back.
- **NTP time sync** — `fetchNtpEpoch()` / `syncNtp()`. `fetchNtpEpoch()` does one SNTPv4 request to `ntpServer` over
  `WiFiUDP` with a ~1.5s timeout and returns the UTC epoch (0 on failure or an implausibly-early timestamp).
  `syncNtp()` adds `utcOffsetHours` and writes the result to the DS3231 with `rtc.setEpoch(..., false)` (+
  `setClockMode(false)` for 24h) — the DS3231 stays the single time source the display reads, NTP only corrects its
  drift. `loop()` calls `syncNtp()` on its first pass, then every `NTP_SYNC_INTERVAL_MS` (1h) once a sync has
  succeeded, or every `NTP_RETRY_INTERVAL_MS` (5min) while it is failing; the blocking UDP exchange stalls `loop()`
  (and the display animation) for up to ~1.5s each time.
- **MCP server** — `initMcp()` (called from `setup()` only when WiFi connected) starts the synchronous `WebServer`
  `mcpServer` on `MCP_PORT` (8080) serving JSON-RPC 2.0 at `POST /mcp` — an unauthenticated MCP endpoint,
  `http://led-clock.local:8080/mcp`. `loop()` pumps `mcpServer.handleClient()`, so `handleMcpPost()` and the tool
  handlers run in the loop task and may touch I2C / flash directly (nothing else in `loop()` overlaps a request).
  `initialize` / `ping` / `tools/list` / `tools/call` are dispatched in `handleMcpPost` (`notifications/*` get a bare
  `202`); responses are always `application/json`, never SSE; the request body comes from `mcpServer.arg("plain")`.
  Tools: `get_sensors` (dumps the `sensors` struct via `mcpFillSensors`), `get_ds3231_time` (reads the RTC live),
  `get_ntp_config` / `set_ntp_config` (`ntp_server`, `utc_offset_hours`; a set validates, applies, `saveConfig()`s,
  then clears `ntpLastAttemptMs` to force an immediate NTP re-sync — a get also reports `last_sync_ok`), and
  `get_display_brightness` / `set_display_brightness` (`brightness` 0..15, applied via `setMax7219Brightness()` and
  persisted). The `get_*` / `set_*` responses share a `mcpFill*` builder. Every tool result carries both a `content`
  text block and a `structuredContent` object.

### Global sensor readings

A single file-scope `SensorReadings sensors;` struct holds the latest values from every sensor. Each `readX()` (and
`handleAs3935Irq()`) writes its freshly-read values into the matching `sensors.<group>` sub-struct right after
logging them, so downstream consumers pull from this struct instead of calling the sensor libraries directly. Groups:
`ds3231` (date/time + on-chip `dieTempC`), `aht20` (`tempC`/`humidityPct`), `bmx280` (`tempC`/`pressurePa`/
`humidityPct`, plus a `hasHumidity` flag — `humidityPct` is `NAN` on a BMP280), `sgp30` (`eco2Ppm`/`tvocPpb`), and
`as3935` (`lastIntSrc` and, for a lightning strike, `distanceKm`/`energy`). Every group carries a `valid` flag (false
until the first good reading, then sticky) and an `updatedMs` millis() timestamp of its last update; static storage
zero-initializes all of it. The one exception is `sgp30`: its readings are stored from the first measurement, but
`sgp30.valid` stays false until `SGP30_WARMUP_MS` (4 hours) after power-on, because the sensor's dynamic baseline
needs hours to settle before eCO2/TVOC mean anything. The MCP `get_sensors` tool serializes this struct;
nothing else consumes it yet.

### Main loop

`setup()` runs `EEPROM.begin()` + `loadConfig()`, brings up `Wire`, runs a full `scan_i2c()` bus scan once, then
calls each `initX()` (including `initMax7219()` and, last, `initWifi()` — which can block on the WiFiManager config
portal, see WiFi above — followed, when WiFi connected, by `showIpOnMax7219()` (blocks while the IP scrolls once)
and `initMcp()`). `loop()` no longer re-scans the I2C bus or blocks on a `delay()` — it services the AS3935 IRQ flag
and ticks the MAX7219 display (`pollMax7219()`) every iteration, pumps `mcpServer.handleClient()` when WiFi is up,
and polls on non-blocking `millis()`-based intervals: `SENSOR_POLL_INTERVAL_MS` (2000ms) for DS3231/AHT20/BMx280, a
separate `SGP30_MEASURE_INTERVAL_MS` (1000ms) timer for SGP30, and `NTP_SYNC_INTERVAL_MS` / `NTP_RETRY_INTERVAL_MS`
for `syncNtp()` (see NTP above). The MAX7219 display keeps its own internal 1000ms timer (inside `pollMax7219()`) for
pushing a fresh HH:MM, decoupled from all of the above since it must also tick Parola's animation every loop
iteration regardless of that timer.
