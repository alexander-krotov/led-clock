/*
 * ESP32 (ESP32C3 Super Mini) with sensors sharing one I2C bus:
 *   - WCMCU-3935   (AS3935 lightning sensor)
 *   - DS3231       (RTC)
 *   - AHT20        (temperature/humidity)
 *   - BMP280/BME280 (pressure/temperature, BME280 also humidity)
 *   - SGP30        (eCO2/TVOC gas sensor)
 *
 * ...plus a MAX7219 LED matrix display (4 x 8x8 modules, SPI bit-banged)
 * driven by the DS3231 RTC, showing the current time HH:MM.
 *
 * Wiring (shared bus):
 *   SCL -> IO9
 *   SDA -> IO18 (labeled MOSI on the WCMCU-3935 board, which doubles as
 *                 I2C SDA in I2C mode)
 *   AS3935 IRQ -> IO5
 *
 * Wiring (MAX7219 display, independent of the I2C bus above):
 *   DIN     -> GPIO6
 *   CLK     -> GPIO10
 *   CS/LOAD -> GPIO7
 *
 * The AS3935 and BMx280 I2C addresses depend on how each board ties its
 * address pins, so setup() probes the common candidates for each and, for
 * the BMx280, reads the chip ID to tell a BMP280 (0x58) from a BME280
 * (0x60) apart so the right compensation/readout path is used. DS3231 and
 * AHT20 use their fixed addresses (0x68 and 0x38). SGP30 also uses a fixed
 * address (0x58), which is unrelated to the BMP280 chip-id value of the
 * same number.
 */

#include <string.h>
#include <time.h>

#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <ArduinoJson.h>

#define WM_MDNS  // let WiFiManager start an mDNS responder for its hostname
#include <WiFiManager.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SGP30.h>
#include <Adafruit_AHTX0.h>
#include <SparkFun_AS3935.h>
#include <DS3231.h>
#include <MD_Parola.h>
#include <MD_MAX72xx.h>

#include "esp32-hal-log.h"

#if defined(CONFIG_IDF_TARGET_ESP32C3)
static const int PIN_SDA = 8;
static const int PIN_SCL = 9;
static const int PIN_IRQ = 5;

static const int PIN_MAX7219_CLK  = 10;
static const int PIN_MAX7219_DATA = 6;
static const int PIN_MAX7219_CS   = 7;

#elif defined(CONFIG_IDF_TARGET_ESP32S3)
static const int PIN_SDA = 15; // 15;
static const int PIN_SCL = 15; // 16;
static const int PIN_IRQ = 4;


// MAX7219 display pins are fixed regardless of board target.
static const int PIN_MAX7219_CLK  = 10;
static const int PIN_MAX7219_DATA = 6;
static const int PIN_MAX7219_CS   = 7;
#else
#error "Unknown target"
#endif

static const uint32_t SENSOR_POLL_INTERVAL_MS = 2000;

// ---------------------------------------------------------------------
// Shared I2C helpers
// ---------------------------------------------------------------------

static const uint8_t I2C_ADDR_NONE = 0xFF;

static bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static uint8_t i2cFindAddress(const uint8_t *candidates, size_t count) {
  for (size_t i = 0; i < count; i++) {
    if (i2cProbe(candidates[i])) {
      return candidates[i];
    }
  }
  return I2C_ADDR_NONE;
}

// ---------------------------------------------------------------------
// Global sensor readings
//
// Every readX() / handleXIrq() writes its latest values into this single
// struct as soon as it has them, so later consumers (e.g. the display)
// can pull from here instead of talking to the sensors directly.
//
// Per group: `valid` becomes true after the first good reading and stays
// true; `updatedMs` is the millis() timestamp of the most recent update.
// Static storage zero-initializes everything, so all groups start
// invalid with zeroed fields.
// ---------------------------------------------------------------------

struct SensorReadings {
  struct {
    bool valid;
    uint32_t updatedMs;
    uint16_t year;      // full 4-digit year
    uint8_t month;
    uint8_t day;
    uint8_t hour;       // 24-hour
    uint8_t minute;
    uint8_t second;
    float dieTempC;      // DS3231 on-chip temperature sensor
  } ds3231;

  struct {
    bool valid;
    uint32_t updatedMs;
    float tempC;
    float humidityPct;
  } aht20;

  struct {
    bool valid;
    uint32_t updatedMs;
    bool hasHumidity;    // true = BME280, false = BMP280
    float tempC;
    float pressurePa;
    float humidityPct;   // NAN when hasHumidity is false
  } bmx280;

  struct {
    bool valid;
    uint32_t updatedMs;
    uint16_t eco2Ppm;
    uint16_t tvocPpb;
  } sgp30;

  struct {
    bool valid;
    uint32_t updatedMs;  // timestamp of the most recent interrupt
    uint8_t lastIntSrc;  // raw INT_SRC nibble: 0x01 noise, 0x04 disturber, 0x08 lightning
    uint8_t distanceKm;  // meaningful only when lastIntSrc == 0x08
    uint32_t energy;     // meaningful only when lastIntSrc == 0x08
  } as3935;
};

static SensorReadings sensors;

// ---------------------------------------------------------------------
// AS3935 (WCMCU-3935 lightning sensor)
// ---------------------------------------------------------------------

// The WCMCU-3935 board straps both address pins HIGH, giving I2C address
// 0x03 (SparkFun's `defAddr`).
static const i2cAddress AS3935_ADDR = defAddr;

SparkFun_AS3935 as3935(AS3935_ADDR);

static bool as3935Present = false;

static volatile bool irqFired = false;

void IRAM_ATTR onAs3935Irq() {
  irqFired = true;
}

static void initAs3935() {
  if (!i2cProbe(AS3935_ADDR) || !as3935.begin(Wire)) {
    log_printf("AS3935 not found at 0x%02X\n", AS3935_ADDR);
    return;
  }

  as3935.resetSettings();

  // Calibrate the internal RC oscillators against the antenna LCO.
  if (!as3935.calibrateOsc()) {
    log_printf("AS3935 oscillator calibration failed, readings may be unreliable\n");
  }

  as3935.setIndoorOutdoor(INDOOR);

  as3935Present = true;
  log_printf("AS3935 found at I2C address 0x%02X\n", AS3935_ADDR);

  attachInterrupt(digitalPinToInterrupt(PIN_IRQ), onAs3935Irq, RISING);
}

static void handleAs3935Irq() {
  if (!irqFired) {
    return;
  }
  irqFired = false;

  if (!as3935Present) {
    return;
  }

  // readInterruptReg() already waits the datasheet-mandated 2ms settle time
  // before reading the interrupt source register.
  uint8_t intSrc = as3935.readInterruptReg();

  sensors.as3935.valid      = true;
  sensors.as3935.updatedMs  = millis();
  sensors.as3935.lastIntSrc = intSrc;
  sensors.as3935.distanceKm = 0;
  sensors.as3935.energy     = 0;

  switch (intSrc) {
    case NOISE_TO_HIGH:
      log_printf("AS3935 IRQ: noise level too high\n");
      break;
    case DISTURBER_DETECT:
      log_printf("AS3935 IRQ: disturber detected\n");
      break;
    case LIGHTNING: {
      uint8_t distance = as3935.distanceToStorm();
      uint32_t energy = as3935.lightningEnergy();
      sensors.as3935.distanceKm = distance;
      sensors.as3935.energy     = energy;
      log_printf("AS3935 IRQ: lightning detected, distance=%u km, energy=%u\n",
                 distance, energy);
      break;
    }
    default:
      log_printf("AS3935 IRQ: unknown interrupt source 0x%02X\n", intSrc);
      break;
  }
}

// ---------------------------------------------------------------------
// DS3231 (RTC)
// ---------------------------------------------------------------------

static const uint8_t DS3231_ADDR = 0x68;

DS3231 rtc;

static bool ds3231Present = false;

static void initDs3231() {
  ds3231Present = i2cProbe(DS3231_ADDR);
  if (!ds3231Present) {
    log_printf("DS3231 not found at 0x%02X\n", DS3231_ADDR);
  }
}

static void readDs3231() {
  if (!ds3231Present) {
    return;
  }

  bool h12Flag;
  bool pmFlag;
  bool century;
  unsigned int h = rtc.getHour(h12Flag, pmFlag);
  unsigned int m = rtc.getMinute();
  unsigned int s = rtc.getSecond();
  unsigned int year = rtc.getYear();
  unsigned int month = rtc.getMonth(century);
  unsigned int day =  rtc.getDate();
  
  float temperatureC = rtc.getTemperature();

  sensors.ds3231.valid     = true;
  sensors.ds3231.updatedMs = millis();
  sensors.ds3231.year      = 2000 + year;
  sensors.ds3231.month     = month;
  sensors.ds3231.day       = day;
  sensors.ds3231.hour      = h;
  sensors.ds3231.minute    = m;
  sensors.ds3231.second    = s;
  sensors.ds3231.dieTempC  = temperatureC;

#if 0
  log_printf("DS3231 time: 20%02u-%02u-%02u %02u:%02u:%02u, temperature: %.2f C\n",
             year, month, day, h, m, s, temperatureC);
#endif
}

// ---------------------------------------------------------------------
// AHT20 (temperature/humidity)
// ---------------------------------------------------------------------

static const uint8_t AHT20_ADDR = 0x38;

Adafruit_AHTX0 aht;

static bool aht20Present = false;

static void initAht20() {
  aht20Present = i2cProbe(AHT20_ADDR) && aht.begin();
  if (!aht20Present) {
    log_printf("AHT20 not found at 0x%02X\n", AHT20_ADDR);
  }
}

static void readAht20() {
  if (!aht20Present) {
    return;
  }
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data

  sensors.aht20.valid       = true;
  sensors.aht20.updatedMs   = millis();
  sensors.aht20.tempC       = temp.temperature;
  sensors.aht20.humidityPct = humidity.relative_humidity;
#if 0
  log_printf("AHT20: temperature=%.2f C, humidity=%.2f %%\n",
             temp.temperature, humidity.relative_humidity);
#endif
}

// ---------------------------------------------------------------------
// BMP280 / BME280 (pressure/temperature, BME280 also humidity)
// ---------------------------------------------------------------------

static const uint8_t BMX280_CHIP_ID_BME280 = 0x60;

static uint8_t bmx280Addr = I2C_ADDR_NONE;
static bool bmx280Present = false;
static bool bmx280HasHumidity = false;

Adafruit_BME280 bme; // I2C

static void initBmx280() {
  static const uint8_t candidates[] = {0x76, 0x77};
  bmx280Addr = i2cFindAddress(candidates, sizeof(candidates) / sizeof(candidates[0]));
  if (bmx280Addr == I2C_ADDR_NONE) {
    log_printf("BMP280/BME280 not found at 0x76/0x77\n");
    return;
  }
  if (!bme.begin(bmx280Addr)) {
    log_printf("BMx280 failed to initialize at 0x%02X\n", bmx280Addr);
    return;
  }
  bmx280Present = true;
  bmx280HasHumidity = (bme.sensorID() == BMX280_CHIP_ID_BME280);
  log_printf("%s found at 0x%02X\n", bmx280HasHumidity ? "BME280" : "BMP280", bmx280Addr);
}

static void readBmx280() {
  if (!bmx280Present) {
    return;
  }
  float pressure = bme.readPressure();
  float temp = bme.readTemperature();
  float humidity = bmx280HasHumidity ? bme.readHumidity() : NAN;

  sensors.bmx280.valid       = true;
  sensors.bmx280.updatedMs   = millis();
  sensors.bmx280.hasHumidity = bmx280HasHumidity;
  sensors.bmx280.tempC       = temp;
  sensors.bmx280.pressurePa  = pressure;
  sensors.bmx280.humidityPct = humidity;

#if 0
  if (bmx280HasHumidity) {
    log_printf("BME280: temperature=%.2f C, pressure=%.2f Pa, humidity=%.2f %%\n",
               temp, pressure, humidity);
  } else {
    log_printf("BMP280: temperature=%.2f C, pressure=%.2f Pa\n", temp, pressure);
  }
#endif
}

// ---------------------------------------------------------------------
// SGP30 (eCO2/TVOC gas sensor)
// ---------------------------------------------------------------------

Adafruit_SGP30 sgp;

// Sensirion recommends calling measure_air_quality exactly once per second
// so the sensor's internal dynamic baseline compensation stays accurate.
static const uint32_t SGP30_MEASURE_INTERVAL_MS = 1000;

// The SGP30's dynamic baseline needs hours of continuous operation to
// settle; until then eCO2/TVOC are effectively meaningless. We still keep
// storing every reading, but don't mark sensors.sgp30 valid until this
// long after power-on.
static const uint32_t SGP30_WARMUP_MS = 4UL * 60 * 60 * 1000; // 4 hours

static bool sgp30Present = false;

static void initSgp30() {
  if (sgp.begin()) {
    sgp30Present = true;
  } else {
    log_printf("SGP30 not found\n");
  }
}

static void readSgp30() {
  if (!sgp30Present) {
    return;
  }
  if (!sgp.IAQmeasure()) {
    log_printf("SGP30 read failed\n");
    return;
  }

  sensors.sgp30.updatedMs = millis();
  sensors.sgp30.eco2Ppm   = sgp.eCO2;
  sensors.sgp30.tvocPpb   = sgp.TVOC;
  if (millis() >= SGP30_WARMUP_MS) {
    sensors.sgp30.valid = true;
  }

#if 0
  log_printf("SGP30: eCO2=%u ppm, TVOC=%u ppb\n", sgp.eCO2, sgp.TVOC);
#endif
}

// ---------------------------------------------------------------------
// MAX7219 LED matrix clock display
//
// Four 8x8 modules split into two MD_Parola zones, both fed from the
// DS3231 RTC once a second:
//   - left zone  (modules 2-3): text "HH:", right-aligned
//   - right zone (modules 0-1): text "MM", left-aligned
// The colon lives in the left (hours) zone, sitting at the middle of the
// chain and reading as the HH:MM separator, so a minutes-only change never
// touches it. A zone whose text changes scrolls the new text down while
// the other zone stays static.
// ---------------------------------------------------------------------

#define MAX7219_HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX7219_MAX_DEVICES   4  // total 8x8 modules in the chain
#define MAX7219_NUM_ZONES     2  // left = "HH:", right = "MM"

#define MAX7219_ZONE_RIGHT    0  // modules 0-1, "MM", left-aligned
#define MAX7219_ZONE_LEFT     1  // modules 2-3, "HH:", right-aligned

static const uint8_t MAX7219_BRIGHTNESS_MAX     = 15;  // MAX7219 hardware intensity range is 0..15
static const uint8_t MAX7219_BRIGHTNESS_DEFAULT = 4;
static uint8_t max7219Brightness = MAX7219_BRIGHTNESS_DEFAULT;  // 0..15, loaded from / saved to EEPROM

static const uint32_t MAX7219_SCROLL_SPEED = 40; // ms per scroll frame

// MD_Parola only positions text at whole-module (8px) granularity, so the
// right zone's "MM" ends up flush against the left zone's colon. After each
// static redraw of the right zone its two modules' pixels are shifted this
// many columns to open a gap between the colon and "MM".
static const uint8_t MAX7219_RIGHT_NUDGE_COLS = 1;
// Shift direction: TSR moves "MM" away from the colon. If it moves
// the wrong way (into the left zone) on your module wiring, use TSL.
static const MD_MAX72XX::transformType_t MAX7219_RIGHT_NUDGE_XFORM = MD_MAX72XX::TSR;

MD_Parola maxDisplay(MAX7219_HARDWARE_TYPE, PIN_MAX7219_DATA, PIN_MAX7219_CLK,
                      PIN_MAX7219_CS, MAX7219_MAX_DEVICES);

struct Max7219ZoneState {
  textPosition_t align;  // PA_LEFT / PA_RIGHT for this zone
  char current[8];       // text currently on screen (Parola holds this pointer)
  char next[8];          // text being scrolled in
  bool scrolling;        // true while a scroll animation is running
};

Max7219ZoneState max7219Zones[MAX7219_NUM_ZONES];

// Set when the right zone has just been drawn as static text and still needs
// its column nudge re-applied (each redraw starts from the module-aligned
// position and overwrites the previous shift).
static bool max7219RightNudgePending = false;

static void initMax7219Zones() {
  maxDisplay.setZone(MAX7219_ZONE_RIGHT, 0, 1);
  maxDisplay.setZone(MAX7219_ZONE_LEFT, 2, 3);

  max7219Zones[MAX7219_ZONE_RIGHT].align = PA_LEFT;
  max7219Zones[MAX7219_ZONE_LEFT].align  = PA_RIGHT;

  MD_MAX72XX::fontType_t *fontDef;

  for (uint8_t z = 0; z < MAX7219_NUM_ZONES; z++) {
    maxDisplay.setSpeed(z, MAX7219_SCROLL_SPEED);
    maxDisplay.setIntensity(z, max7219Brightness);
    maxDisplay.setPause(z, 0);

    max7219Zones[z].current[0] = '\0';
    max7219Zones[z].next[0]    = '\0';
    max7219Zones[z].scrolling  = false;

    maxDisplay.displayZoneText(z, max7219Zones[z].current, max7219Zones[z].align,
                                MAX7219_SCROLL_SPEED, 0, PA_PRINT, PA_NO_EFFECT);
  }

  max7219RightNudgePending = true;
}

// Shift the right zone (modules 0-1) sideways by MAX7219_RIGHT_NUDGE_COLS so
// its "MM" is not flush against the colon. Must run after every static redraw
// of that zone, since Parola redraws it module-aligned.
static void applyMax7219RightNudge() {
  MD_MAX72XX *mx = maxDisplay.getGraphicObject();
  for (uint8_t i = 0; i < MAX7219_RIGHT_NUDGE_COLS; i++) {
    mx->transform(0, 1, MAX7219_RIGHT_NUDGE_XFORM);
  }
  mx->update();
}

static void triggerMax7219ScrollDown(uint8_t z, const char *text) {
  strncpy(max7219Zones[z].next, text, sizeof(max7219Zones[z].next) - 1);
  max7219Zones[z].next[sizeof(max7219Zones[z].next) - 1] = '\0';
  // PA_SCROLL_DOWN scrolls content downward (new text enters from the top).
  maxDisplay.displayZoneText(z, max7219Zones[z].next, max7219Zones[z].align,
                              MAX7219_SCROLL_SPEED, 0, PA_SCROLL_DOWN, PA_SCROLL_DOWN);
  maxDisplay.displayReset(z);
  max7219Zones[z].scrolling = true;
}

// Feed a fresh HH:MM to the two zones ("HH:" left, "MM" right). Keeping the
// colon in the left (hours) zone means a minutes-only update never touches
// it, so it doesn't scroll every time the minutes change.
static void updateMax7219Time(char hTens, char hUnits, char mTens, char mUnits) {
  char leftText[8]  = {hTens, hUnits, ':', '\0'};
  char rightText[8] = {mTens, mUnits, '\0'};
  const char *want[MAX7219_NUM_ZONES];
  want[MAX7219_ZONE_LEFT]  = leftText;
  want[MAX7219_ZONE_RIGHT] = rightText;

#if 0
  log_printf("MAX7219 display=%s%s\n", leftText, rightText);
#endif

  for (uint8_t z = 0; z < MAX7219_NUM_ZONES; z++) {
    if (max7219Zones[z].scrolling) {
      continue; // wait for the running scroll to finish
    }
    if (strcmp(want[z], max7219Zones[z].current) != 0) {
      triggerMax7219ScrollDown(z, want[z]);
    }
  }
}

static void initMax7219() {
  maxDisplay.begin(MAX7219_NUM_ZONES);
  initMax7219Zones();
}

// Set the display intensity (0..MAX7219_BRIGHTNESS_MAX) on every zone and
// record it in max7219Brightness. Persisting is the caller's job.
static void setMax7219Brightness(uint8_t level) {
  if (level > MAX7219_BRIGHTNESS_MAX) {
    level = MAX7219_BRIGHTNESS_MAX;
  }
  max7219Brightness = level;
  maxDisplay.setIntensity(max7219Brightness);
}

// Ticks Parola's animation every call (needed for smooth scrolling) and
// pushes a fresh HH:MM from the RTC once a second.
static void pollMax7219() {
  if (maxDisplay.displayAnimate()) {
    for (uint8_t z = 0; z < MAX7219_NUM_ZONES; z++) {
      if (!maxDisplay.getZoneStatus(z) || !max7219Zones[z].scrolling) {
        continue;
      }
      // Scroll finished -- latch the new text and hold it static.
      strcpy(max7219Zones[z].current, max7219Zones[z].next);
      max7219Zones[z].scrolling = false;
      maxDisplay.displayZoneText(z, max7219Zones[z].current, max7219Zones[z].align,
                                  MAX7219_SCROLL_SPEED, 0, PA_PRINT, PA_NO_EFFECT);
      maxDisplay.displayReset(z);
      if (z == MAX7219_ZONE_RIGHT) {
        max7219RightNudgePending = true;
      }
    }
  }

  // Re-apply the right-zone column nudge once the zone is idle again.
  if (max7219RightNudgePending &&
      !max7219Zones[MAX7219_ZONE_RIGHT].scrolling &&
      maxDisplay.getZoneStatus(MAX7219_ZONE_RIGHT)) {
    applyMax7219RightNudge();
    max7219RightNudgePending = false;
  }

  if (!ds3231Present) {
    log_printf("pollMax7219: no ds3231\n");
    return;
  }

  static uint32_t lastMax7219SecMs = 0;
  uint32_t now = millis();
  if (now - lastMax7219SecMs > 1000UL) {
    lastMax7219SecMs = now;

    bool h12Flag;
    bool pmFlag;
    unsigned int h = rtc.getHour(h12Flag, pmFlag);
    unsigned int m = rtc.getMinute();

    updateMax7219Time('0' + h / 10, '0' + h % 10, '0' + m / 10, '0' + m % 10);
  }
}

// ---------------------------------------------------------------------
// Persistent configuration (EEPROM)
//
// The ESP32 EEPROM library emulates a small EEPROM in a flash partition;
// EEPROM.begin(EEPROM_SIZE) must run once in setup() before any access.
// Layout: a magic byte to detect an uninitialized/foreign partition, the
// UTC offset (int8_t hours), the NTP server name as a fixed-length,
// NUL-terminated byte array, then the display brightness (uint8_t 0..15).
// loadConfig() seeds the defaults on first boot; saveConfig() is called by
// the WiFiManager save-params callback (see initWifi) and by the MCP
// set_ntp_config / set_display_brightness tools.
// ---------------------------------------------------------------------

static const int     EEPROM_SIZE            = 128;
static const int     EEPROM_MAGIC_ADDR      = 0;
static const int     EEPROM_TZ_ADDR         = 1;   // int8_t, UTC offset in hours
static const int     EEPROM_NTP_ADDR        = 2;   // char[NTP_SERVER_MAXLEN]
static const int     EEPROM_BRIGHTNESS_ADDR = 66;  // uint8_t, 0..MAX7219_BRIGHTNESS_MAX
static const uint8_t EEPROM_MAGIC           = 0x37;

static const size_t  NTP_SERVER_MAXLEN  = 64;

static char   ntpServer[NTP_SERVER_MAXLEN] = "fi.pool.ntp.org";
static int8_t utcOffsetHours              = 0;

static void saveConfig() {
  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
  EEPROM.write(EEPROM_TZ_ADDR, (uint8_t)utcOffsetHours);
  EEPROM.writeBytes(EEPROM_NTP_ADDR, ntpServer, NTP_SERVER_MAXLEN);
  EEPROM.write(EEPROM_BRIGHTNESS_ADDR, max7219Brightness);
  EEPROM.commit();
  log_printf("config saved: ntpServer=%s, utcOffset=%dh, brightness=%u\n",
             ntpServer, utcOffsetHours, max7219Brightness);
}

static void loadConfig() {
  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC) {
    log_printf("config: EEPROM not initialized, writing defaults\n");
    saveConfig();
    return;
  }

  utcOffsetHours = (int8_t)EEPROM.read(EEPROM_TZ_ADDR);
  EEPROM.readBytes(EEPROM_NTP_ADDR, ntpServer, NTP_SERVER_MAXLEN);
  ntpServer[NTP_SERVER_MAXLEN - 1] = '\0';
  if (ntpServer[0] == '\0') {
    strncpy(ntpServer, "fi.pool.ntp.org", NTP_SERVER_MAXLEN - 1);
  }

  uint8_t brightness = EEPROM.read(EEPROM_BRIGHTNESS_ADDR);
  // Falls here for EEPROM written before the brightness byte existed (0xFF).
  max7219Brightness = (brightness <= MAX7219_BRIGHTNESS_MAX) ? brightness
                                                            : MAX7219_BRIGHTNESS_DEFAULT;

  log_printf("config loaded: ntpServer=%s, utcOffset=%dh, brightness=%u\n",
             ntpServer, utcOffsetHours, max7219Brightness);
}

// ---------------------------------------------------------------------
// WiFi (WiFiManager captive-portal provisioning)
//
// WiFiManager keeps the last working credentials in the ESP32 NVS and
// reconnects to them on boot. When it can't connect within
// WIFI_MANAGER_TIMEOUT_S it starts a temporary "led-clock" access point
// serving a captive configuration portal; initWifi() blocks in setup()
// until the user enters credentials or the timeout expires.
//
// The portal also carries two custom fields -- the NTP server name and
// the UTC offset -- so both can be set alongside the WiFi credentials.
// WiFiManager only shows the portal when it can't auto-connect, so these
// fields are editable at first setup (or whenever WiFi is reconfigured);
// otherwise the values persisted in EEPROM are used as-is.
//
// With WM_MDNS defined (see the include above), setHostname() also makes
// WiFiManager bring up an mDNS responder, so the clock is reachable as
// "led-clock.local". ESP32's mDNS runs in the background -- no loop() tick
// is needed to keep it alive.
// ---------------------------------------------------------------------

static const char WIFI_DEVICE_NAME[] = "led-clock";
static const uint32_t WIFI_MANAGER_TIMEOUT_S = 60;

static bool wifiConnected = false;

static void initWifi() {
  WiFi.mode(WIFI_STA);

  WiFiManager wm;
  wm.setHostname(WIFI_DEVICE_NAME);
  wm.setConfigPortalTimeout(WIFI_MANAGER_TIMEOUT_S);

  char utcOffsetStr[6];
  snprintf(utcOffsetStr, sizeof(utcOffsetStr), "%d", utcOffsetHours);
  WiFiManagerParameter ntpParam("ntp", "NTP server", ntpServer, NTP_SERVER_MAXLEN - 1);
  WiFiManagerParameter utcParam("utc", "UTC offset (hours)", utcOffsetStr, 5);
  wm.addParameter(&ntpParam);
  wm.addParameter(&utcParam);
  wm.setSaveParamsCallback([&]() {
    strncpy(ntpServer, ntpParam.getValue(), NTP_SERVER_MAXLEN - 1);
    ntpServer[NTP_SERVER_MAXLEN - 1] = '\0';
    utcOffsetHours = (int8_t)atoi(utcParam.getValue());
    saveConfig();
  });

  wifiConnected = wm.autoConnect(WIFI_DEVICE_NAME);
  wm.stopWebPortal();

  if (wifiConnected) {
    log_printf("WiFi connected: SSID=%s, IP=%s, mDNS=%s.local\n",
               WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(),
               WIFI_DEVICE_NAME);
  } else {
    log_printf("WiFi not connected (config portal timed out)\n");
  }
}

// Spin displayAnimate() until zone z finishes its current effect (or a
// timeout), so a one-off animation can be run synchronously from setup().
static void pumpMax7219UntilIdle(uint8_t z, uint32_t timeoutMs) {
  uint32_t start = millis();
  while (!maxDisplay.getZoneStatus(z) && millis() - start < timeoutMs) {
    maxDisplay.displayAnimate();
  }
}

// Scroll the DHCP-assigned IP address once across the whole four-module
// chain, then restore the two-zone HH: / MM clock layout. Called from
// setup() after WiFi connects -- the display is too narrow to show an IP
// address all at once.
static void showIpOnMax7219() {
  String ip = WiFi.localIP().toString();
  log_printf("MAX7219: scrolling IP %s\n", ip.c_str());

  // Blank the left zone and let it settle so only zone 0 drives the chain
  // while it is temporarily widened below.
  maxDisplay.displayZoneText(MAX7219_ZONE_LEFT, "", PA_LEFT, 0, 0,
                              PA_PRINT, PA_NO_EFFECT);
  maxDisplay.displayReset(MAX7219_ZONE_LEFT);
  pumpMax7219UntilIdle(MAX7219_ZONE_LEFT, 1000);

  // Repurpose zone 0 to span every module for a single right-to-left scroll.
  maxDisplay.setZone(MAX7219_ZONE_RIGHT, 0, MAX7219_MAX_DEVICES - 1);
  maxDisplay.setIntensity(MAX7219_ZONE_RIGHT, max7219Brightness);
  maxDisplay.displayClear();
  // Parola keeps the text pointer, not a copy -- `ip` must outlive the scroll
  // below, and it does (initMax7219Zones() swaps the pointer before return).
  maxDisplay.displayZoneText(MAX7219_ZONE_RIGHT, ip.c_str(), PA_LEFT,
                              MAX7219_SCROLL_SPEED, 0, PA_SCROLL_LEFT, PA_SCROLL_LEFT);
  maxDisplay.displayReset(MAX7219_ZONE_RIGHT);
  pumpMax7219UntilIdle(MAX7219_ZONE_RIGHT, 30000);

  maxDisplay.displayClear();
  initMax7219Zones();
}

// ---------------------------------------------------------------------
// NTP time sync
//
// Once WiFi is up, syncNtp() asks the configured NTP server (SNTPv4 over
// UDP, one request, ~1.5s timeout) for the current UTC, adds the
// configured utcOffsetHours, and writes the result to the DS3231 with
// rtc.setEpoch(). The DS3231 stays the single time source the display
// reads from -- NTP just corrects its drift. loop() runs the first sync
// on its first pass and then re-syncs every NTP_SYNC_INTERVAL_MS, backing
// off to NTP_RETRY_INTERVAL_MS while a sync is failing. The UDP exchange
// blocks loop() for up to ~1.5s while it runs.
// ---------------------------------------------------------------------

static const uint16_t NTP_LOCAL_PORT       = 8888;
static const uint32_t NTP_SYNC_INTERVAL_MS  = 60UL * 60 * 1000;  // hourly when healthy
static const uint32_t NTP_RETRY_INTERVAL_MS = 5UL * 60 * 1000;   // sooner after a failure
static const uint32_t NTP_UNIX_EPOCH_DIFF   = 2208988800UL;      // 1900 -> 1970

static WiFiUDP ntpUdp;

// loop() scheduling state for syncNtp(); file-scope so an MCP config change
// can force an immediate re-sync by clearing ntpLastAttemptMs.
static uint32_t ntpLastAttemptMs = 0;
static bool     ntpLastOk        = false;

// Returns UTC epoch seconds from the configured NTP server, or 0 on failure.
static uint32_t fetchNtpEpoch() {
  IPAddress serverIp;
  if (!WiFi.hostByName(ntpServer, serverIp)) {
    log_printf("NTP: DNS lookup for %s failed\n", ntpServer);
    return 0;
  }

  uint8_t pkt[48] = {0};
  pkt[0] = 0b11100011;  // LI = 3 (unsynchronized), VN = 4, Mode = 3 (client)
  pkt[1] = 0;           // stratum
  pkt[2] = 6;           // polling interval
  pkt[3] = 0xEC;        // peer clock precision
  pkt[12] = 49; pkt[13] = 0x4E; pkt[14] = 49; pkt[15] = 52;  // reference id

  ntpUdp.begin(NTP_LOCAL_PORT);
  while (ntpUdp.parsePacket() > 0) {
    /* drain any stale datagrams */
  }
  ntpUdp.beginPacket(serverIp, 123);
  ntpUdp.write(pkt, sizeof(pkt));
  ntpUdp.endPacket();

  uint32_t deadline = millis() + 1500;
  while ((int32_t)(deadline - millis()) > 0) {
    if (ntpUdp.parsePacket() >= (int)sizeof(pkt)) {
      ntpUdp.read(pkt, sizeof(pkt));
      ntpUdp.stop();
      uint32_t secsSince1900 = ((uint32_t)pkt[40] << 24) | ((uint32_t)pkt[41] << 16) |
                               ((uint32_t)pkt[42] << 8)  |  (uint32_t)pkt[43];
      uint32_t epoch = secsSince1900 - NTP_UNIX_EPOCH_DIFF;
      if (epoch < 1577836800UL) {  // before 2020-01-01 -> implausible reply
        log_printf("NTP: implausible timestamp from %s\n", ntpServer);
        return 0;
      }
      return epoch;
    }
  }

  ntpUdp.stop();
  log_printf("NTP: no response from %s\n", ntpServer);
  return 0;
}

static bool syncNtp() {
  if (!WiFi.isConnected() || !ds3231Present) {
    return false;
  }

  uint32_t utc = fetchNtpEpoch();
  if (utc == 0) {
    return false;
  }

  rtc.setEpoch((time_t)utc + (int32_t)utcOffsetHours * 3600, false);
  rtc.setClockMode(false);  // 24-hour
  log_printf("NTP: DS3231 set from %s (UTC epoch %lu, offset %dh)\n",
             ntpServer, (unsigned long)utc, utcOffsetHours);
  return true;
}

// ---------------------------------------------------------------------
// MCP server (Model Context Protocol over HTTP)
//
// Unauthenticated JSON-RPC 2.0 endpoint at
//   POST http://led-clock.local:MCP_PORT/mcp
// served by the ESP32 core's synchronous WebServer: mcpServer.handleClient()
// is pumped from loop(), so the handlers run in the loop task and can touch
// I2C / flash directly (no other loop() work overlaps a request). Responses
// are plain application/json -- no SSE streaming.
//
// Tools:
//   get_sensors             -- dump the whole `sensors` struct
//   get_ds3231_time         -- read the DS3231 wall clock live
//   get_ntp_config          -- current ntpServer / utcOffsetHours / last sync ok
//   set_ntp_config          -- change ntpServer / utcOffsetHours, persist, re-sync
//   get_display_brightness  -- current MAX7219 intensity (0..15)
//   set_display_brightness  -- change MAX7219 intensity, persist
// ---------------------------------------------------------------------

static const uint16_t MCP_PORT = 8080;
static const char     MCP_PROTOCOL_VERSION[] = "2025-06-18";

static WebServer mcpServer(MCP_PORT);

static void mcpAddText(JsonObject result, const String &text) {
  JsonArray content = result["content"].to<JsonArray>();
  JsonObject item = content.add<JsonObject>();
  item["type"] = "text";
  item["text"] = text;
}

static void mcpFillSensors(JsonObject o) {
  JsonObject ds = o["ds3231"].to<JsonObject>();
  ds["valid"]     = sensors.ds3231.valid;
  ds["updatedMs"] = sensors.ds3231.updatedMs;
  ds["year"]      = sensors.ds3231.year;
  ds["month"]     = sensors.ds3231.month;
  ds["day"]       = sensors.ds3231.day;
  ds["hour"]      = sensors.ds3231.hour;
  ds["minute"]    = sensors.ds3231.minute;
  ds["second"]    = sensors.ds3231.second;
  ds["dieTempC"]  = sensors.ds3231.dieTempC;

  JsonObject aht = o["aht20"].to<JsonObject>();
  aht["valid"]       = sensors.aht20.valid;
  aht["updatedMs"]   = sensors.aht20.updatedMs;
  aht["tempC"]       = sensors.aht20.tempC;
  aht["humidityPct"] = sensors.aht20.humidityPct;

  JsonObject bmx = o["bmx280"].to<JsonObject>();
  bmx["valid"]       = sensors.bmx280.valid;
  bmx["updatedMs"]   = sensors.bmx280.updatedMs;
  bmx["hasHumidity"] = sensors.bmx280.hasHumidity;
  bmx["tempC"]       = sensors.bmx280.tempC;
  bmx["pressurePa"]  = sensors.bmx280.pressurePa;
  bmx["humidityPct"] = sensors.bmx280.humidityPct;  // NaN on a BMP280 -> null

  JsonObject sgp = o["sgp30"].to<JsonObject>();
  sgp["valid"]     = sensors.sgp30.valid;
  sgp["updatedMs"] = sensors.sgp30.updatedMs;
  sgp["eco2Ppm"]   = sensors.sgp30.eco2Ppm;
  sgp["tvocPpb"]   = sensors.sgp30.tvocPpb;

  JsonObject as = o["as3935"].to<JsonObject>();
  as["valid"]      = sensors.as3935.valid;
  as["updatedMs"]  = sensors.as3935.updatedMs;
  as["lastIntSrc"] = sensors.as3935.lastIntSrc;
  as["distanceKm"] = sensors.as3935.distanceKm;
  as["energy"]     = sensors.as3935.energy;
}

static void mcpToolGetSensors(JsonObject result) {
  JsonObject sc = result["structuredContent"].to<JsonObject>();
  mcpFillSensors(sc);
  String text;
  serializeJson(sc, text);
  mcpAddText(result, text);
}

static void mcpToolGetDs3231Time(JsonObject result, bool &isError) {
  if (!ds3231Present) {
    mcpAddText(result, "DS3231 not present");
    isError = true;
    return;
  }

  bool h12Flag, pmFlag, century;
  unsigned year   = 2000 + rtc.getYear();
  unsigned month  = rtc.getMonth(century);
  unsigned day    = rtc.getDate();
  unsigned hour   = rtc.getHour(h12Flag, pmFlag);
  unsigned minute = rtc.getMinute();
  unsigned second = rtc.getSecond();
  float dieTempC  = rtc.getTemperature();

  char formatted[24];
  snprintf(formatted, sizeof(formatted), "%04u-%02u-%02u %02u:%02u:%02u",
           year, month, day, hour, minute, second);

  JsonObject sc = result["structuredContent"].to<JsonObject>();
  sc["localTime"]      = String(formatted);
  sc["year"]           = year;
  sc["month"]          = month;
  sc["day"]            = day;
  sc["hour"]           = hour;
  sc["minute"]         = minute;
  sc["second"]         = second;
  sc["dieTempC"]       = dieTempC;
  sc["utcOffsetHours"] = utcOffsetHours;

  String text;
  serializeJson(sc, text);
  mcpAddText(result, text);
}

static void mcpFillNtpConfig(JsonObject result) {
  JsonObject sc = result["structuredContent"].to<JsonObject>();
  sc["ntp_server"]       = String(ntpServer);
  sc["utc_offset_hours"] = utcOffsetHours;
  sc["last_sync_ok"]     = ntpLastOk;
  String text;
  serializeJson(sc, text);
  mcpAddText(result, text);
}

static void mcpToolGetNtpConfig(JsonObject result) {
  mcpFillNtpConfig(result);
}

static void mcpToolSetNtpConfig(JsonObjectConst args, JsonObject result, bool &isError) {
  bool changed = false;

  if (!args["ntp_server"].isNull()) {
    const char *s = args["ntp_server"].as<const char *>();
    if (s == nullptr || s[0] == '\0' || strlen(s) >= NTP_SERVER_MAXLEN) {
      mcpAddText(result, "ntp_server must be 1.." + String(NTP_SERVER_MAXLEN - 1) + " characters");
      isError = true;
      return;
    }
    strncpy(ntpServer, s, NTP_SERVER_MAXLEN - 1);
    ntpServer[NTP_SERVER_MAXLEN - 1] = '\0';
    changed = true;
  }

  if (!args["utc_offset_hours"].isNull()) {
    if (!args["utc_offset_hours"].is<int>()) {
      mcpAddText(result, "utc_offset_hours must be an integer");
      isError = true;
      return;
    }
    int offset = args["utc_offset_hours"].as<int>();
    if (offset < -14 || offset > 14) {
      mcpAddText(result, "utc_offset_hours must be between -14 and 14");
      isError = true;
      return;
    }
    utcOffsetHours = (int8_t)offset;
    changed = true;
  }

  if (!changed) {
    mcpAddText(result, "provide ntp_server and/or utc_offset_hours");
    isError = true;
    return;
  }

  saveConfig();
  ntpLastAttemptMs = 0;  // force a fresh NTP sync with the new settings

  mcpFillNtpConfig(result);
}

static void mcpFillBrightness(JsonObject result) {
  JsonObject sc = result["structuredContent"].to<JsonObject>();
  sc["brightness"] = max7219Brightness;
  sc["min"]        = 0;
  sc["max"]        = MAX7219_BRIGHTNESS_MAX;
  String text;
  serializeJson(sc, text);
  mcpAddText(result, text);
}

static void mcpToolGetDisplayBrightness(JsonObject result) {
  mcpFillBrightness(result);
}

static void mcpToolSetDisplayBrightness(JsonObjectConst args, JsonObject result, bool &isError) {
  if (args["brightness"].isNull() || !args["brightness"].is<int>()) {
    mcpAddText(result, "brightness (integer 0..15) is required");
    isError = true;
    return;
  }
  int level = args["brightness"].as<int>();
  if (level < 0 || level > MAX7219_BRIGHTNESS_MAX) {
    mcpAddText(result, "brightness must be between 0 and 15");
    isError = true;
    return;
  }

  setMax7219Brightness((uint8_t)level);
  saveConfig();
  mcpFillBrightness(result);
}

static void mcpFillToolsList(JsonObject result) {
  JsonArray tools = result["tools"].to<JsonArray>();

  {
    JsonObject t = tools.add<JsonObject>();
    t["name"] = "get_sensors";
    t["description"] = "Read every sensor group from the clock's SensorReadings struct "
                       "(ds3231, aht20, bmx280, sgp30, as3935), each with its valid flag "
                       "and updatedMs timestamp.";
    JsonObject schema = t["inputSchema"].to<JsonObject>();
    schema["type"] = "object";
    schema["properties"].to<JsonObject>();
  }
  {
    JsonObject t = tools.add<JsonObject>();
    t["name"] = "get_ds3231_time";
    t["description"] = "Read the current wall-clock time live from the DS3231 RTC "
                       "(local time, i.e. UTC plus the configured offset).";
    JsonObject schema = t["inputSchema"].to<JsonObject>();
    schema["type"] = "object";
    schema["properties"].to<JsonObject>();
  }
  {
    JsonObject t = tools.add<JsonObject>();
    t["name"] = "get_ntp_config";
    t["description"] = "Read the configured NTP server hostname, the UTC offset (hours), "
                       "and whether the last NTP sync succeeded.";
    JsonObject schema = t["inputSchema"].to<JsonObject>();
    schema["type"] = "object";
    schema["properties"].to<JsonObject>();
  }
  {
    JsonObject t = tools.add<JsonObject>();
    t["name"] = "set_ntp_config";
    t["description"] = "Update the NTP server hostname and/or the UTC offset (hours). "
                       "Changes are persisted to EEPROM and trigger an immediate re-sync.";
    JsonObject schema = t["inputSchema"].to<JsonObject>();
    schema["type"] = "object";
    JsonObject props = schema["properties"].to<JsonObject>();
    JsonObject p1 = props["ntp_server"].to<JsonObject>();
    p1["type"] = "string";
    p1["description"] = "NTP server hostname, e.g. fi.pool.ntp.org";
    JsonObject p2 = props["utc_offset_hours"].to<JsonObject>();
    p2["type"] = "integer";
    p2["minimum"] = -14;
    p2["maximum"] = 14;
    p2["description"] = "Hours to add to UTC before writing the RTC";
  }
  {
    JsonObject t = tools.add<JsonObject>();
    t["name"] = "get_display_brightness";
    t["description"] = "Read the current MAX7219 LED matrix intensity (0..15).";
    JsonObject schema = t["inputSchema"].to<JsonObject>();
    schema["type"] = "object";
    schema["properties"].to<JsonObject>();
  }
  {
    JsonObject t = tools.add<JsonObject>();
    t["name"] = "set_display_brightness";
    t["description"] = "Set the MAX7219 LED matrix intensity (0 = dimmest, 15 = "
                       "brightest). Persisted to EEPROM.";
    JsonObject schema = t["inputSchema"].to<JsonObject>();
    schema["type"] = "object";
    JsonObject props = schema["properties"].to<JsonObject>();
    JsonObject p = props["brightness"].to<JsonObject>();
    p["type"] = "integer";
    p["minimum"] = 0;
    p["maximum"] = 15;
    p["description"] = "MAX7219 intensity level";
    JsonArray req = schema["required"].to<JsonArray>();
    req.add("brightness");
  }
}

static void mcpSendJson(JsonDocument &doc, int code = 200) {
  String out;
  serializeJson(doc, out);
  mcpServer.send(code, "application/json", out);
}

static void handleMcpPost() {
  const String &body = mcpServer.arg("plain");

  JsonDocument req;
  if (deserializeJson(req, body) != DeserializationError::Ok || !req.is<JsonObject>()) {
    JsonDocument resp;
    resp["jsonrpc"] = "2.0";
    resp["id"] = nullptr;
    JsonObject e = resp["error"].to<JsonObject>();
    e["code"] = -32700;
    e["message"] = "Parse error";
    mcpSendJson(resp);
    return;
  }

  const char *method = req["method"].as<const char *>();
  if (method == nullptr) method = "";

  // A JSON-RPC notification has no id -- acknowledge without a body.
  if (req["id"].isNull()) {
    mcpServer.send(202, "text/plain", "");
    return;
  }

  JsonDocument resp;
  resp["jsonrpc"] = "2.0";
  resp["id"] = req["id"];

  if (strcmp(method, "initialize") == 0) {
    JsonObject result = resp["result"].to<JsonObject>();
    result["protocolVersion"] = MCP_PROTOCOL_VERSION;
    result["capabilities"].to<JsonObject>()["tools"].to<JsonObject>();
    JsonObject info = result["serverInfo"].to<JsonObject>();
    info["name"] = "led-clock";
    info["version"] = "1.0.0";
  } else if (strcmp(method, "ping") == 0) {
    resp["result"].to<JsonObject>();
  } else if (strcmp(method, "tools/list") == 0) {
    mcpFillToolsList(resp["result"].to<JsonObject>());
  } else if (strcmp(method, "tools/call") == 0) {
    JsonObjectConst params = req["params"].as<JsonObjectConst>();
    const char *name = params["name"].as<const char *>();
    if (name == nullptr) name = "";
    JsonObjectConst args = params["arguments"].as<JsonObjectConst>();

    JsonObject result = resp["result"].to<JsonObject>();
    bool isError = false;
    if (strcmp(name, "get_sensors") == 0) {
      mcpToolGetSensors(result);
    } else if (strcmp(name, "get_ds3231_time") == 0) {
      mcpToolGetDs3231Time(result, isError);
    } else if (strcmp(name, "get_ntp_config") == 0) {
      mcpToolGetNtpConfig(result);
    } else if (strcmp(name, "set_ntp_config") == 0) {
      mcpToolSetNtpConfig(args, result, isError);
    } else if (strcmp(name, "get_display_brightness") == 0) {
      mcpToolGetDisplayBrightness(result);
    } else if (strcmp(name, "set_display_brightness") == 0) {
      mcpToolSetDisplayBrightness(args, result, isError);
    } else {
      mcpAddText(result, String("unknown tool: ") + name);
      isError = true;
    }
    if (isError) result["isError"] = true;
  } else {
    JsonObject e = resp["error"].to<JsonObject>();
    e["code"] = -32601;
    e["message"] = "Method not found";
  }

  mcpSendJson(resp);
}

static void initMcp() {
  mcpServer.on("/mcp", HTTP_POST, handleMcpPost);
  mcpServer.on("/mcp", HTTP_GET, []() {
    mcpServer.send(405, "text/plain", "MCP endpoint accepts POST only");
  });
  mcpServer.onNotFound([]() {
    mcpServer.send(404, "application/json", "{\"error\":\"not found\"}");
  });
  mcpServer.begin();
  log_printf("MCP server on http://%s.local:%u/mcp\n", WIFI_DEVICE_NAME, MCP_PORT);
}

// ---------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  pinMode(PIN_IRQ, INPUT);

  Wire.begin(PIN_SDA, PIN_SCL);

  // Lower the speed and set higher timeouts for SGP30 stability.
  Wire.setClock(50000);
  Wire.setTimeOut(25);

  delay(1000);
  log_printf("Starting\n");

  EEPROM.begin(EEPROM_SIZE);
  loadConfig();

  scan_i2c();

  initAs3935();
  initDs3231();
  initAht20();
  initBmx280();
  initSgp30();
  initMax7219();
  initWifi();

  if (wifiConnected) {
    showIpOnMax7219();
    initMcp();
  }
}

void scan_i2c()
{
  int nDevices=0;

  log_printf("Scanning for I2C devices ...\n");
  for (int address = 0x01; address < 0x7f; address++) {
    Wire.beginTransmission(address);
    int error = Wire.endTransmission();
    if (error == 0) {
      log_printf("I2C device found at address 0x%02X\n", address);
      nDevices++;
    } else if (error != 2) {
      // log_printf("Error %u at address 0x%02X\n", error, address);
    }
  }
  if (nDevices == 0) {
    log_printf("No I2C devices found\n");
  } 
}

void loop() {
  handleAs3935Irq();
  pollMax7219();

  static uint32_t lastSensorPoll = 0;
  uint32_t now = millis();
  if (now - lastSensorPoll >= SENSOR_POLL_INTERVAL_MS) {
    lastSensorPoll = now;
    readDs3231();
    readAht20();
    readBmx280();
  }

  static uint32_t lastSgp30Poll = 0;
  if (now - lastSgp30Poll >= SGP30_MEASURE_INTERVAL_MS) {
    lastSgp30Poll = now;
    readSgp30();
  }

  if (wifiConnected) {
    mcpServer.handleClient();
  }

  uint32_t ntpInterval = ntpLastOk ? NTP_SYNC_INTERVAL_MS : NTP_RETRY_INTERVAL_MS;
  if (wifiConnected && (ntpLastAttemptMs == 0 || now - ntpLastAttemptMs >= ntpInterval)) {
    ntpLastAttemptMs = now;
    ntpLastOk = syncNtp();
  }
}
