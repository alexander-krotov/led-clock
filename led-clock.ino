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

#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>

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
//   - right zone (modules 0-1): text ":MM", left-aligned
// The two colons meet in the middle of the chain and read as the HH:MM
// separator. A zone whose text changes scrolls the new text down while
// the other zone stays static.
// ---------------------------------------------------------------------

#define MAX7219_HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX7219_MAX_DEVICES   4  // total 8x8 modules in the chain
#define MAX7219_NUM_ZONES     2  // left = "HH:", right = ":MM"

#define MAX7219_ZONE_RIGHT    0  // modules 0-1, ":MM", left-aligned
#define MAX7219_ZONE_LEFT     1  // modules 2-3, "HH:", right-aligned

static uint8_t max7219Brightness = 4;            // MAX7219 range 0-15, runtime-adjustable
static const uint32_t MAX7219_SCROLL_SPEED = 40; // ms per scroll frame

// MD_Parola only positions text at whole-module (8px) granularity, so the
// left zone's "HH" ends up flush against the colon (the first glyph of the
// right zone). After each static redraw of the left zone its two modules'
// pixels are shifted this many columns to open a gap before the colon.
static const uint8_t MAX7219_LEFT_NUDGE_COLS = 1;
// Shift direction: TSL moves the digits away from the colon. If they move
// the wrong way (into the right zone) on your module wiring, use TSR.
static const MD_MAX72XX::transformType_t MAX7219_LEFT_NUDGE_XFORM = MD_MAX72XX::TSL;

MD_Parola maxDisplay(MAX7219_HARDWARE_TYPE, PIN_MAX7219_DATA, PIN_MAX7219_CLK,
                      PIN_MAX7219_CS, MAX7219_MAX_DEVICES);

struct Max7219ZoneState {
  textPosition_t align;  // PA_LEFT / PA_RIGHT for this zone
  char current[8];       // text currently on screen (Parola holds this pointer)
  char next[8];          // text being scrolled in
  bool scrolling;        // true while a scroll animation is running
};

Max7219ZoneState max7219Zones[MAX7219_NUM_ZONES];

// Set when the left zone has just been drawn as static text and still needs
// its column nudge re-applied (each redraw starts from the module-aligned
// position and overwrites the previous shift).
static bool max7219LeftNudgePending = false;

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

  max7219LeftNudgePending = true;
}

// Shift the left zone (modules 2-3) sideways by MAX7219_LEFT_NUDGE_COLS so
// its "HH" is not flush against the colon. Must run after every static
// redraw of that zone, since Parola redraws it module-aligned.
static void applyMax7219LeftNudge() {
  MD_MAX72XX *mx = maxDisplay.getGraphicObject();
  for (uint8_t i = 0; i < MAX7219_LEFT_NUDGE_COLS; i++) {
    mx->transform(2, 3, MAX7219_LEFT_NUDGE_XFORM);
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

// Feed a fresh HH:MM to the two zones ("HH:" left, ":MM" right).
static void updateMax7219Time(char hTens, char hUnits, char mTens, char mUnits) {
  char leftText[8]  = {hTens, hUnits,  '\0'};
  char rightText[8] = {':', mTens, mUnits, '\0'};
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
      if (z == MAX7219_ZONE_LEFT) {
        max7219LeftNudgePending = true;
      }
    }
  }

  // Re-apply the left-zone column nudge once the zone is idle again.
  if (max7219LeftNudgePending &&
      !max7219Zones[MAX7219_ZONE_LEFT].scrolling &&
      maxDisplay.getZoneStatus(MAX7219_ZONE_LEFT)) {
    applyMax7219LeftNudge();
    max7219LeftNudgePending = false;
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
// WiFi (WiFiManager captive-portal provisioning)
//
// WiFiManager keeps the last working credentials in the ESP32 NVS and
// reconnects to them on boot. When it can't connect within
// WIFI_MANAGER_TIMEOUT_S it starts a temporary "led-clock" access point
// serving a captive configuration portal; initWifi() blocks in setup()
// until the user enters credentials or the timeout expires. Nothing in
// loop() depends on the connection yet -- wifiConnected is just recorded
// for planned NTP time sync.
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
// chain, then restore the two-zone HH / :MM clock layout. Called from
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

void setup() {
  Serial.begin(115200);
  pinMode(PIN_IRQ, INPUT);

  Wire.begin(PIN_SDA, PIN_SCL);

  // Lower the speed and set higher timeouts for SGP30 stability.
  Wire.setClock(50000);
  Wire.setTimeOut(25);

  delay(1000);
  log_printf("Starting\n");

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
}
