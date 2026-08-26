/*
 * ESP32 (Ozobot RVDKit) with sensors sharing one I2C bus:
 *   - WCMCU-3935   (AS3935 lightning sensor)
 *   - DS3231       (RTC)
 *   - AHT20        (temperature/humidity)
 *   - BMP280/BME280 (pressure/temperature, BME280 also humidity)
 *   - SGP30        (eCO2/TVOC gas sensor)
 *
 * Wiring (shared bus):
 *   SCL -> IO18
 *   SDA -> IO17  (labeled MOSI on the WCMCU-3935 board, which doubles as
 *                 I2C SDA in I2C mode)
 *   AS3935 IRQ -> IO4
 *
 * The AS3935 and BMx280 I2C addresses depend on how each board ties its
 * address pins, so setup() probes the common candidates for each and, for
 * the BMx280, reads the chip ID to tell a BMP280 (0x58) from a BME280
 * (0x60) apart so the right compensation/readout path is used. DS3231 and
 * AHT20 use their fixed addresses (0x68 and 0x38). SGP30 also uses a fixed
 * address (0x58), which is unrelated to the BMP280 chip-id value of the
 * same number.
 */

#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SGP30.h>
#include <Adafruit_AHTX0.h>
#include <DS3231.h>

#include "esp32-hal-log.h"

#if defined(CONFIG_IDF_TARGET_ESP32C3)
static const int PIN_SDA = 8;
static const int PIN_SCL = 9;
static const int PIN_IRQ = 5;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
static const int PIN_SDA = 15; // 15;
static const int PIN_SCL = 15; // 16;
static const int PIN_IRQ = 4;
#else
#error "Unknown target"
#endif

static const uint32_t SENSOR_POLL_INTERVAL_MS = 2000;

// ---------------------------------------------------------------------
// Shared I2C helpers
// ---------------------------------------------------------------------

static const uint8_t I2C_ADDR_NONE = 0xFF;

static bool i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom((int)addr, (int)len) != (int)len) {
    return false;
  }
  for (size_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
  return true;
}

// For devices (e.g. AHT20) that reply with raw data instead of exposing
// registers you address before reading.
static bool i2cReadRaw(uint8_t addr, uint8_t *buf, size_t len) {
  if (Wire.requestFrom((int)addr, (int)len) != (int)len) {
    return false;
  }
  for (size_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
  return true;
}

static uint8_t i2cReadReg(uint8_t addr, uint8_t reg) {
  uint8_t value = 0xFF;
  i2cReadBytes(addr, reg, &value, 1);
  return value;
}

static void i2cWriteReg(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

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
// AS3935 (WCMCU-3935 lightning sensor)
// ---------------------------------------------------------------------

// AS3935 register map (see AMS AS3935 datasheet, table 18)
static const uint8_t REG_AFE_PWD    = 0x00;
static const uint8_t REG_INT_SRC    = 0x03; // interrupt source, bits [3:0]
static const uint8_t REG_ENERGY_LSB = 0x04;
static const uint8_t REG_ENERGY_MSB = 0x05;
static const uint8_t REG_ENERGY_MMSB = 0x06;
static const uint8_t REG_DISTANCE   = 0x07;
static const uint8_t REG_TRCO_CALIB = 0x3A;
static const uint8_t REG_SRCO_CALIB = 0x3B;
static const uint8_t CMD_PRESET_DEFAULT = 0x3C;
static const uint8_t CMD_CALIB_RCO      = 0x3D;

static uint8_t as3935Addr = 0x3;

static volatile bool irqFired = false;

void IRAM_ATTR onAs3935Irq() {
  irqFired = true;
}

static bool calibrateAs3935() {
  // Reset all registers to their power-on defaults.
  i2cWriteReg(as3935Addr, CMD_PRESET_DEFAULT, 0x96);
  delay(2);

  // Calibrate the internal RC oscillators against the antenna LCO.
  i2cWriteReg(as3935Addr, CMD_CALIB_RCO, 0x96);
  delay(2);

  uint8_t trco = i2cReadReg(as3935Addr, REG_TRCO_CALIB);
  uint8_t srco = i2cReadReg(as3935Addr, REG_SRCO_CALIB);
  bool trcoOk = (trco & 0x40) && !(trco & 0x20);
  bool srcoOk = (srco & 0x40) && !(srco & 0x20);

  log_printf("AS3935 calibration: TRCO reg=0x%02X (%s), SRCO reg=0x%02X (%s)\n",
             trco, trcoOk ? "OK" : "FAILED",
             srco, srcoOk ? "OK" : "FAILED");

  return trcoOk && srcoOk;
}

static void initAs3935() {
  log_printf("AS3935 found at I2C address 0x%02X\n", as3935Addr);

  if (!calibrateAs3935()) {
    log_printf("AS3935 calibration failed, readings may be unreliable\n");
  }

  uint8_t afePwd = i2cReadReg(as3935Addr, REG_AFE_PWD);
  log_printf("AS3935 status: REG0=0x%02X, PWD=%d, AFE_GB=%d\n",
             afePwd, afePwd & 0x01, (afePwd >> 1) & 0x1F);

  attachInterrupt(digitalPinToInterrupt(PIN_IRQ), onAs3935Irq, RISING);
}

static void handleAs3935Irq() {
  if (!irqFired) {
    return;
  }
  irqFired = false;
  delay(2); // datasheet: wait 2ms after IRQ before reading the interrupt source

  uint8_t intSrc = i2cReadReg(as3935Addr, REG_INT_SRC) & 0x0F;
  switch (intSrc) {
    case 0x01:
      log_printf("AS3935 IRQ: noise level too high\n");
      break;
    case 0x04:
      log_printf("AS3935 IRQ: disturber detected\n");
      break;
    case 0x08: {
      uint8_t distance = i2cReadReg(as3935Addr, REG_DISTANCE) & 0x3F;
      uint32_t energy = ((uint32_t)(i2cReadReg(as3935Addr, REG_ENERGY_MMSB) & 0x1F) << 16) |
                         ((uint32_t)i2cReadReg(as3935Addr, REG_ENERGY_MSB) << 8) |
                         i2cReadReg(as3935Addr, REG_ENERGY_LSB);
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

  log_printf("DS3231 time: 20%02u-%02u-%02u %02u:%02u:%02u, temperature: %.2f C\n",
             year, month, day, h, m, s, temperatureC);
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
  log_printf("AHT20: temperature=%.2f C, humidity=%.2f %%\n",
             temp.temperature, humidity.relative_humidity);
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

  if (bmx280HasHumidity) {
    float humidity = bme.readHumidity();
    log_printf("BME280: temperature=%.2f C, pressure=%.2f Pa, humidity=%.2f %%\n",
               temp, pressure, humidity);
  } else {
    log_printf("BMP280: temperature=%.2f C, pressure=%.2f Pa\n", temp, pressure);
  }
}

// ---------------------------------------------------------------------
// SGP30 (eCO2/TVOC gas sensor)
// ---------------------------------------------------------------------

Adafruit_SGP30 sgp;

// Sensirion recommends calling measure_air_quality exactly once per second
// so the sensor's internal dynamic baseline compensation stays accurate.
static const uint32_t SGP30_MEASURE_INTERVAL_MS = 1000;

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

  log_printf("SGP30: eCO2=%u ppm, TVOC=%u ppb\n", sgp.eCO2, sgp.TVOC);
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
