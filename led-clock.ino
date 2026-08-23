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
#include "esp32-hal-log.h"

#if defined(CONFIG_IDF_TARGET_ESP32C3)
static const int PIN_SDA = 8;
static const int PIN_SCL = 9;
static const int PIN_IRQ = 7;
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
static const uint8_t DS3231_REG_SECONDS = 0x00;
static const uint8_t DS3231_REG_STATUS  = 0x0F;

static bool ds3231Present = false;

static uint8_t bcdToDec(uint8_t bcd) {
  return (bcd >> 4) * 10 + (bcd & 0x0F);
}

static void initDs3231() {
  ds3231Present = i2cProbe(DS3231_ADDR);
  if (!ds3231Present) {
    log_printf("DS3231 not found at 0x%02X\n", DS3231_ADDR);
    return;
  }

  uint8_t status = i2cReadReg(DS3231_ADDR, DS3231_REG_STATUS);
  bool oscillatorStopped = status & 0x80;
  log_printf("DS3231 found at 0x%02X, status=0x%02X, oscillator %s\n",
             DS3231_ADDR, status, oscillatorStopped ? "STOPPED (time may be invalid)" : "OK");
}

static void readDs3231() {
  if (!ds3231Present) {
    return;
  }

  uint8_t buf[7];
  if (!i2cReadBytes(DS3231_ADDR, DS3231_REG_SECONDS, buf, sizeof(buf))) {
    log_printf("DS3231 read failed\n");
    return;
  }

  // Assumes the RTC is set to 24-hour mode (register 2 bit 6 clear).
  uint8_t seconds = bcdToDec(buf[0] & 0x7F);
  uint8_t minutes = bcdToDec(buf[1] & 0x7F);
  uint8_t hours   = bcdToDec(buf[2] & 0x3F);
  uint8_t day     = bcdToDec(buf[4] & 0x3F);
  uint8_t month   = bcdToDec(buf[5] & 0x1F);
  uint8_t year    = bcdToDec(buf[6]);

  log_printf("DS3231 time: 20%02u-%02u-%02u %02u:%02u:%02u\n",
             year, month, day, hours, minutes, seconds);
}

// ---------------------------------------------------------------------
// AHT20 (temperature/humidity)
// ---------------------------------------------------------------------

static const uint8_t AHT20_ADDR = 0x38;

static bool aht20Present = false;

static void initAht20() {
  aht20Present = i2cProbe(AHT20_ADDR);
  if (!aht20Present) {
    log_printf("AHT20 not found at 0x%02X\n", AHT20_ADDR);
    return;
  }

  Wire.beginTransmission(AHT20_ADDR);
  Wire.write(0xBE);
  Wire.write(0x08);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(10);

  uint8_t status = 0xFF;
  i2cReadRaw(AHT20_ADDR, &status, 1);
  bool calibrated = status & 0x08;
  log_printf("AHT20 found at 0x%02X, status=0x%02X, calibrated=%s\n",
             AHT20_ADDR, status, calibrated ? "yes" : "no");
}

static void readAht20() {
  if (!aht20Present) {
    return;
  }

  Wire.beginTransmission(AHT20_ADDR);
  Wire.write(0xAC);
  Wire.write(0x33);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(80);

  uint8_t buf[6];
  if (!i2cReadRaw(AHT20_ADDR, buf, sizeof(buf))) {
    log_printf("AHT20 read failed\n");
    return;
  }
  if (buf[0] & 0x80) {
    log_printf("AHT20 still busy, skipping this reading\n");
    return;
  }

  uint32_t rawHumidity = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4) | (buf[3] >> 4);
  uint32_t rawTemp = (((uint32_t)buf[3] & 0x0F) << 16) | ((uint32_t)buf[4] << 8) | buf[5];
  float humidity = (rawHumidity / 1048576.0f) * 100.0f;
  float temperature = (rawTemp / 1048576.0f) * 200.0f - 50.0f;

  log_printf("AHT20: temperature=%.2f C, humidity=%.2f %%\n", temperature, humidity);
}

// ---------------------------------------------------------------------
// BMP280 / BME280 (pressure/temperature, BME280 also humidity)
// ---------------------------------------------------------------------

static const uint8_t BMX280_REG_CHIP_ID  = 0xD0;
static const uint8_t BMX280_REG_CALIB    = 0x88;
static const uint8_t BMX280_REG_CALIB_H1 = 0xA1;
static const uint8_t BMX280_REG_CALIB_H2 = 0xE1;
static const uint8_t BMX280_REG_CTRL_HUM  = 0xF2;
static const uint8_t BMX280_REG_CTRL_MEAS = 0xF4;
static const uint8_t BMX280_REG_CONFIG    = 0xF5;
static const uint8_t BMX280_REG_DATA      = 0xF7;

static const uint8_t BMX280_CHIP_ID_BMP280 = 0x58;
static const uint8_t BMX280_CHIP_ID_BME280 = 0x60;

struct Bmx280Calib {
  uint16_t dig_T1;
  int16_t dig_T2, dig_T3;
  uint16_t dig_P1;
  int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
  uint8_t dig_H1;
  int16_t dig_H2;
  uint8_t dig_H3;
  int16_t dig_H4, dig_H5;
  int8_t dig_H6;
};

static uint8_t bmx280Addr = I2C_ADDR_NONE;
static bool bmx280Present = false;
static bool bmx280HasHumidity = false;
static Bmx280Calib bmx280Calib;
static int32_t bmx280TFine;

static void initBmx280() {
  static const uint8_t candidates[] = {0x76, 0x77};
  bmx280Addr = i2cFindAddress(candidates, sizeof(candidates) / sizeof(candidates[0]));
  if (bmx280Addr == I2C_ADDR_NONE) {
    log_printf("BMP280/BME280 not found at 0x76/0x77\n");
    return;
  }

  uint8_t chipId = i2cReadReg(bmx280Addr, BMX280_REG_CHIP_ID);
  bmx280HasHumidity = (chipId == BMX280_CHIP_ID_BME280);
  log_printf("%s found at 0x%02X, chip id=0x%02X\n",
             bmx280HasHumidity ? "BME280" : "BMP280", bmx280Addr, chipId);

  uint8_t calib[24];
  if (!i2cReadBytes(bmx280Addr, BMX280_REG_CALIB, calib, sizeof(calib))) {
    log_printf("BMx280 failed to read calibration data\n");
    return;
  }
  bmx280Calib.dig_T1 = (uint16_t)(calib[1] << 8 | calib[0]);
  bmx280Calib.dig_T2 = (int16_t)(calib[3] << 8 | calib[2]);
  bmx280Calib.dig_T3 = (int16_t)(calib[5] << 8 | calib[4]);
  bmx280Calib.dig_P1 = (uint16_t)(calib[7] << 8 | calib[6]);
  bmx280Calib.dig_P2 = (int16_t)(calib[9] << 8 | calib[8]);
  bmx280Calib.dig_P3 = (int16_t)(calib[11] << 8 | calib[10]);
  bmx280Calib.dig_P4 = (int16_t)(calib[13] << 8 | calib[12]);
  bmx280Calib.dig_P5 = (int16_t)(calib[15] << 8 | calib[14]);
  bmx280Calib.dig_P6 = (int16_t)(calib[17] << 8 | calib[16]);
  bmx280Calib.dig_P7 = (int16_t)(calib[19] << 8 | calib[18]);
  bmx280Calib.dig_P8 = (int16_t)(calib[21] << 8 | calib[20]);
  bmx280Calib.dig_P9 = (int16_t)(calib[23] << 8 | calib[22]);

  if (bmx280HasHumidity) {
    bmx280Calib.dig_H1 = i2cReadReg(bmx280Addr, BMX280_REG_CALIB_H1);

    uint8_t calibH[7];
    i2cReadBytes(bmx280Addr, BMX280_REG_CALIB_H2, calibH, sizeof(calibH));
    bmx280Calib.dig_H2 = (int16_t)(calibH[1] << 8 | calibH[0]);
    bmx280Calib.dig_H3 = calibH[2];
    bmx280Calib.dig_H4 = (int16_t)((calibH[3] << 4) | (calibH[4] & 0x0F));
    bmx280Calib.dig_H5 = (int16_t)((calibH[5] << 4) | (calibH[4] >> 4));
    bmx280Calib.dig_H6 = (int8_t)calibH[6];

    // Humidity oversampling x1; must be written before ctrl_meas.
    i2cWriteReg(bmx280Addr, BMX280_REG_CTRL_HUM, 0x01);
  }

  // Normal mode, temperature/pressure oversampling x1 (Bosch datasheet 3.3.1).
  i2cWriteReg(bmx280Addr, BMX280_REG_CTRL_MEAS, 0x27);
  i2cWriteReg(bmx280Addr, BMX280_REG_CONFIG, 0x00);
  bmx280Present = true;
}

// Compensation formulas below are the integer versions from the Bosch
// BMP280/BME280 datasheets (temperature/pressure section 3.11.3, humidity
// section 4.2.3 for BME280).
static int32_t bmx280CompensateTemperature(int32_t adcT) {
  int32_t var1 = ((((adcT >> 3) - ((int32_t)bmx280Calib.dig_T1 << 1))) * (int32_t)bmx280Calib.dig_T2) >> 11;
  int32_t var2 = (((((adcT >> 4) - (int32_t)bmx280Calib.dig_T1) *
                     ((adcT >> 4) - (int32_t)bmx280Calib.dig_T1)) >> 12) *
                   (int32_t)bmx280Calib.dig_T3) >> 14;
  bmx280TFine = var1 + var2;
  return (bmx280TFine * 5 + 128) >> 8; // 0.01 degC
}

static uint32_t bmx280CompensatePressure(int32_t adcP) {
  int64_t var1 = (int64_t)bmx280TFine - 128000;
  int64_t var2 = var1 * var1 * (int64_t)bmx280Calib.dig_P6;
  var2 += (var1 * (int64_t)bmx280Calib.dig_P5) << 17;
  var2 += ((int64_t)bmx280Calib.dig_P4) << 35;
  var1 = ((var1 * var1 * (int64_t)bmx280Calib.dig_P3) >> 8) +
         ((var1 * (int64_t)bmx280Calib.dig_P2) << 12);
  var1 = ((((int64_t)1 << 47) + var1) * (int64_t)bmx280Calib.dig_P1) >> 33;
  if (var1 == 0) {
    return 0; // avoid division by zero
  }
  int64_t p = 1048576 - adcP;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = ((int64_t)bmx280Calib.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
  var2 = ((int64_t)bmx280Calib.dig_P8 * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)bmx280Calib.dig_P7) << 4);
  return (uint32_t)p; // Q24.8 fixed point, Pa = p / 256
}

static uint32_t bmx280CompensateHumidity(int32_t adcH) {
  int32_t v = bmx280TFine - 76800;
  v = (((((adcH << 14) - ((int32_t)bmx280Calib.dig_H4 << 20) -
          ((int32_t)bmx280Calib.dig_H5 * v)) + 16384) >> 15) *
       ((((((v * (int32_t)bmx280Calib.dig_H6) >> 10) *
           (((v * (int32_t)bmx280Calib.dig_H3) >> 11) + 32768)) >> 10) +
         2097152) * (int32_t)bmx280Calib.dig_H2 + 8192) >> 14);
  v = v - (((((v >> 15) * (v >> 15)) >> 7) * (int32_t)bmx280Calib.dig_H1) >> 4);
  v = v < 0 ? 0 : v;
  v = v > 419430400 ? 419430400 : v;
  return (uint32_t)(v >> 12); // Q22.10 fixed point, %RH = value / 1024
}

static void readBmx280() {
  if (!bmx280Present) {
    return;
  }

  uint8_t len = bmx280HasHumidity ? 8 : 6;
  uint8_t buf[8];
  if (!i2cReadBytes(bmx280Addr, BMX280_REG_DATA, buf, len)) {
    log_printf("BMx280 read failed\n");
    return;
  }

  int32_t adcP = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | (buf[2] >> 4);
  int32_t adcT = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | (buf[5] >> 4);

  int32_t temp = bmx280CompensateTemperature(adcT);
  uint32_t pressure = bmx280CompensatePressure(adcP);

  if (bmx280HasHumidity) {
    int32_t adcH = ((int32_t)buf[6] << 8) | buf[7];
    uint32_t humidity = bmx280CompensateHumidity(adcH);
    log_printf("BME280: temperature=%.2f C, pressure=%.2f hPa, humidity=%.2f %%\n",
               temp / 100.0f, pressure / 256.0f / 100.0f, humidity / 1024.0f);
  } else {
    log_printf("BMP280: temperature=%.2f C, pressure=%.2f hPa\n",
               temp / 100.0f, pressure / 256.0f / 100.0f);
  }
}

// ---------------------------------------------------------------------
// SGP30 (eCO2/TVOC gas sensor)
// ---------------------------------------------------------------------

static const uint8_t SGP30_ADDR = 0x58;
static const uint16_t SGP30_CMD_INIT_AIR_QUALITY    = 0x2003;
static const uint16_t SGP30_CMD_MEASURE_AIR_QUALITY = 0x2008;

// Sensirion recommends calling measure_air_quality exactly once per second
// so the sensor's internal dynamic baseline compensation stays accurate.
static const uint32_t SGP30_MEASURE_INTERVAL_MS = 1000;

static bool sgp30Present = false;

// Sensirion I2C checksum: CRC-8, polynomial 0x31, initial value 0xFF.
static uint8_t sgp30Crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static bool sgp30SendCommand(uint16_t command) {
  Wire.beginTransmission(SGP30_ADDR);
  Wire.write((uint8_t)(command >> 8));
  Wire.write((uint8_t)(command & 0xFF));
  return Wire.endTransmission() == 0;
}

// Reads `words` 16-bit words, each followed by a CRC-8 byte, validating each checksum.
static bool sgp30ReadWords(uint16_t *out, size_t words) {
  uint8_t buf[3 * 2]; // largest response used here is 2 words
  size_t len = words * 3;
  if (!i2cReadRaw(SGP30_ADDR, buf, len)) {
    return false;
  }
  for (size_t i = 0; i < words; i++) {
    uint8_t *w = buf + i * 3;
    if (sgp30Crc8(w, 2) != w[2]) {
      return false;
    }
    out[i] = ((uint16_t)w[0] << 8) | w[1];
  }
  return true;
}

static void initSgp30() {
  sgp30Present = i2cProbe(SGP30_ADDR);
  if (!sgp30Present) {
    log_printf("SGP30 not found at 0x%02X\n", SGP30_ADDR);
    return;
  }

  if (!sgp30SendCommand(SGP30_CMD_INIT_AIR_QUALITY)) {
    log_printf("SGP30 init_air_quality command failed\n");
    sgp30Present = false;
    return;
  }
  delay(10);

  log_printf("SGP30 found at 0x%02X\n", SGP30_ADDR);
}

static void readSgp30() {
  if (!sgp30Present) {
    return;
  }

  if (!sgp30SendCommand(SGP30_CMD_MEASURE_AIR_QUALITY)) {
    log_printf("SGP30 measure_air_quality command failed\n");
    return;
  }
  delay(12);

  // For the first 15 measurements after init, the sensor returns fixed
  // values (eCO2=400ppm, TVOC=0ppb) while its baseline warms up.
  uint16_t words[2];
  if (!sgp30ReadWords(words, 2)) {
    log_printf("SGP30 read failed\n");
    return;
  }

  log_printf("SGP30: eCO2=%u ppm, TVOC=%u ppb\n", words[0], words[1]);
}

// ---------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  pinMode(PIN_IRQ, INPUT);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);

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
  // scan_i2c(); // Temporary helper.

  delay(10000);
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
