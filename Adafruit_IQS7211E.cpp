/*!
 * @file Adafruit_IQS7211E.cpp
 * @brief Polling driver for the Azoteq IQS7211E capacitive trackpad.
 * Written by Adafruit. MIT license; see LICENSE.
 */
#include "Adafruit_IQS7211E.h"

#include "Adafruit_IQS7211E_Config.h"

/** @brief Construct an IQS7211E interface without touching the bus. */
Adafruit_IQS7211E::Adafruit_IQS7211E() {}

/** @brief Release the owned BusIO interface. */
Adafruit_IQS7211E::~Adafruit_IQS7211E() {
  delete _i2c;
}

/*!
 * @brief Initialize and calibrate all 42 points on Rev B or repaired Rev A.
 * @param wire I2C bus. The driver initializes it at 100 kHz.
 * @param readyPin Optional active-low RDY input; -1 uses I2C polling only.
 * No MCU pull-up or interrupt is enabled. Keep the pad untouched during
 * begin().
 * @return True after product validation, settings upload and successful ATI.
 * @note Defaults: active mode, 20 ms reports, 0-1000 coordinates, two fingers,
 * no gestures or low-power modes. AVR Wire timeout is set to 250 ms because
 * the chip stretches SCL. Other cores must support appropriate clock
 * stretching.
 */
bool Adafruit_IQS7211E::begin(TwoWire* wire, int16_t readyPin) {
  _initialized = false;
  _validData = false;
  _readyPin = readyPin;
  delete _i2c;
  _i2c = new Adafruit_I2CDevice(IQS7211E_I2C_ADDRESS, wire);
  if (!_i2c || !_i2c->begin(false)) {
    _error = IQS7211E_ERROR_I2C;
    return false;
  }
  _i2c->setSpeed(100000);
#if defined(WIRE_HAS_TIMEOUT)
  wire->setWireTimeout(250000, true);
#endif
  if (_readyPin >= 0) {
    pinMode(_readyPin, INPUT);
  }

  // The product number, not an ACK alone, identifies the peripheral.
  if (getProductNumber() != IQS7211E_PRODUCT_NUMBER) {
    _error = IQS7211E_ERROR_DEVICE;
    return false;
  }
  if (!reset()) {
    return false;
  }
  if (!loadSettings()) {
    _error = IQS7211E_ERROR_I2C;
    return false;
  }

  // Section 9.3.1: acknowledge reset only after replacing the settings.
  Adafruit_BusIO_Register system(_i2c, IQS7211E_REG_SYSTEM, 2, LSBFIRST);
  bool written = system.write(IQS7211E_COMMAND_ACK_RESET);
  bool ended = endCommunication();
  if (!written || !ended) {
    _error = IQS7211E_ERROR_I2C;
    return false;
  }

  _initialized = true;
  if (!calibrate()) {
    _initialized = false;
    return false;
  }
  _error = IQS7211E_ERROR_NONE;
  return true;
}

/*!
 * @brief Read the hardware product number.
 * @return 1112 for IQS7211E, another product number, or 0 on a failed read.
 */
uint16_t Adafruit_IQS7211E::getProductNumber() {
  if (!_i2c) {
    return 0;
  }
  Adafruit_BusIO_Register product(_i2c, IQS7211E_REG_PRODUCT, 2, LSBFIRST);
  uint16_t value;
  bool read = product.read(&value);
  bool ended = endCommunication();
  if (!read || !ended || value == IQS7211E_INVALID_WORD) {
    return 0;
  }
  return value;
}

/** @brief Read firmware major version. @return Version or -1 on read failure.
 */
int16_t Adafruit_IQS7211E::getFirmwareMajor() {
  if (!_i2c) {
    return -1;
  }
  Adafruit_BusIO_Register version(_i2c, IQS7211E_REG_VERSION_MAJOR, 2,
                                  LSBFIRST);
  uint16_t value;
  bool read = version.read(&value);
  bool ended = endCommunication();
  if (!read || !ended || value > 255) {
    return -1;
  }
  return value;
}

/** @brief Read firmware minor version. @return Version or -1 on read failure.
 */
int16_t Adafruit_IQS7211E::getFirmwareMinor() {
  if (!_i2c) {
    return -1;
  }
  Adafruit_BusIO_Register version(_i2c, IQS7211E_REG_VERSION_MINOR, 2,
                                  LSBFIRST);
  uint16_t value;
  bool read = version.read(&value);
  bool ended = endCommunication();
  if (!read || !ended || value > 255) {
    return -1;
  }
  return value;
}

/*!
 * @brief Reset the controller; call begin() afterward to restore its settings.
 * @return True if the reset command was delivered and the device identifies.
 * @note Section 9.3.2: reset occurs after the communication window ends.
 */
bool Adafruit_IQS7211E::reset() {
  _initialized = false;
  _validData = false;
  if (!_i2c) {
    _error = IQS7211E_ERROR_NOT_INITIALIZED;
    return false;
  }
  // Establish explicit window termination even after a previous host restart.
  Adafruit_BusIO_Register config(_i2c, IQS7211E_REG_CONFIG, 2, LSBFIRST);
  bool written = config.write(IQS7211E_CONFIG_POLLING);
  // System-control is a command register: do not replay other command bits.
  Adafruit_BusIO_Register system(_i2c, IQS7211E_REG_SYSTEM, 2, LSBFIRST);
  if (written) {
    written = system.write(IQS7211E_COMMAND_RESET);
  }
  bool ended = endCommunication();
  if (!written || !ended) {
    _error = IQS7211E_ERROR_I2C;
    return false;
  }
  delay(100);
  if (getProductNumber() != IQS7211E_PRODUCT_NUMBER) {
    _error = IQS7211E_ERROR_DEVICE;
    return false;
  }
  _error = IQS7211E_ERROR_NONE;
  return true;
}

/*!
 * @brief Replace all writable settings before starting calibration.
 * @return True when all profile, mapping and sensing-cycle bytes were sent.
 */
bool Adafruit_IQS7211E::loadSettings() {
  // Section 10.1.2 recommends replacing the complete writable memory map.
  // Hold one communication window so the partially loaded profile cannot scan.
  Adafruit_BusIO_Register config(_i2c, IQS7211E_REG_CONFIG, 2, LSBFIRST);
  if (!config.write(IQS7211E_CONFIG_POLLING)) {
    endCommunication();
    return false;
  }

  bool written = true;
  // 14 words plus an address fit even in the ATmega328 Wire transmit buffer.
  uint16_t words[14];
  for (uint8_t offset = 0; offset < 55 && written; offset += 14) {
    uint8_t count = min((uint8_t)14, (uint8_t)(55 - offset));
    for (uint8_t i = 0; i < count; i++) {
#if defined(__AVR__) || defined(ESP8266)
      // ESP8266 flash also requires the program-memory accessor.
      words[i] = pgm_read_word(&iqs7211e_default_settings[offset + i]);
#else
      words[i] = iqs7211e_default_settings[offset + i];
#endif
    }
    Adafruit_BusIO_Register settings(_i2c, IQS7211E_REG_CONFIG_START + offset,
                                     2, LSBFIRST);
    written = settings.write((uint8_t*)words, count * 2);
  }

  // Rev B, or Rev A with RX6/TX3 physically exchanged: bottom-to-top RX rows,
  // then left-to-right TX columns. The middle row now uses physical RX6.
  uint8_t mapping[14] = {0, 1, 2, 6, 4, 5, 12, 11, 10, 9, 8, 7, 3, 0};
  Adafruit_BusIO_Register map(_i2c, IQS7211E_REG_MAPPING, 2, LSBFIRST);
  if (written) {
    written = map.write(mapping, sizeof(mapping));
  }

  // Each cycle has a control byte, one block A channel and one block B channel.
  // Three rows in each block fit all 42 channels into 21 cycles
  // (datasheet sections 7.1.2-7.1.4). Pair physical RX0/RX6, RX1/RX4,
  // then RX2/RX5 for each transmitter; allocation uses logical row indices.
  uint8_t cycles[64];
  for (uint8_t cycle = 0; cycle < 21; cycle++) {
    uint8_t tx = cycle / 3;
    uint8_t rx = cycle % 3;
    cycles[cycle * 3] = 0x05; // Required cycle control, section 12.
    cycles[cycle * 3 + 1] = tx * 6 + rx;
    cycles[cycle * 3 + 2] = tx * 6 + rx + 3;
  }
  cycles[63] = 0x01; // Required final control at 0x7C, not 0x05.
  for (uint8_t offset = 0; offset < sizeof(cycles) && written; offset += 28) {
    uint8_t length = min((uint8_t)28, (uint8_t)(sizeof(cycles) - offset));
    Adafruit_BusIO_Register allocation(_i2c, IQS7211E_REG_CYCLES + offset / 2,
                                       2, LSBFIRST);
    written = allocation.write(cycles + offset, length);
  }
  bool ended = endCommunication();
  return written && ended;
}

/*!
 * @brief Recalibrate the electrodes with a bounded wait; leave them untouched.
 * @param timeoutMS Maximum wait in milliseconds, excluding one bus transaction.
 * @return True when trackpad ATI finishes without an ATI error.
 * @note ATI compensation is volatile. This does not write OTP or flash.
 */
bool Adafruit_IQS7211E::calibrate(uint16_t timeoutMS) {
  _validData = false;
  if (!_initialized) {
    _error = IQS7211E_ERROR_NOT_INITIALIZED;
    return false;
  }
  // Section 5.6.3: this command clears itself when trackpad ATI has completed.
  Adafruit_BusIO_Register system(_i2c, IQS7211E_REG_SYSTEM, 2, LSBFIRST);
  bool written = system.write(IQS7211E_COMMAND_CALIBRATE);
  bool ended = endCommunication();
  if (!written || !ended) {
    _error = IQS7211E_ERROR_I2C;
    return false;
  }
  uint32_t start = millis();
  while (millis() - start < timeoutMS) {
    delay(10);
    iqs7211e_system_t command;
    bool read = system.read(&command.value);
    if (!read || command.value == IQS7211E_INVALID_WORD) {
      endCommunication();
      continue; // Comms is unavailable while the ATI routine runs.
    }
    // Use the named command field after a checked read; BusIO's field-only
    // getter cannot distinguish an I2C failure from an asserted bit.
    if (!command.bits.trackpadATI) {
      // Keep this window open through the result read. This also keeps RDY
      // asserted when an optional ready pin is used; readData ends the window.
      return readData();
    }
    if (!endCommunication()) {
      _error = IQS7211E_ERROR_I2C;
      return false;
    }
  }
  _error = IQS7211E_ERROR_TIMEOUT;
  return false;
}

/*!
 * @brief Acquire one coherent set of finger coordinates and status.
 * @return True for a valid frame, including a valid no-touch frame. On failure
 * touch getters return zero/false, and getLastError() explains the failure.
 * @note No software wait/retry loop is used. With no RDY pin, I2C clock
 * stretching waits for the next available communication window. Poll about
 * once per report interval. With RDY connected, an inactive input returns
 * immediately. A reset requires begin() to reload the board configuration.
 */
bool Adafruit_IQS7211E::readData() {
  _validData = false;
  if (!_initialized) {
    _error = IQS7211E_ERROR_NOT_INITIALIZED;
    return false;
  }
  if (_readyPin >= 0 && digitalRead(_readyPin) != LOW) {
    _error = IQS7211E_ERROR_NOT_READY;
    return false;
  }
  // The complete 0x0A-0x17 frame fits within AVR Wire's 32-byte receive buffer.
  // A single read prevents finger coordinates and status from different scans.
  iqs7211e_frame_t frame;
  Adafruit_BusIO_Register data(_i2c, IQS7211E_REG_DATA, 2, LSBFIRST);
  bool read = data.read(frame.bytes, sizeof(frame.bytes));
  bool ended = endCommunication();
  if (!read || !ended || frame.data.info.reserved15 ||
      frame.data.info.reserved13 || frame.data.info.reserved11 ||
      (frame.data.info.fingers > IQS7211E_MAX_TOUCHES &&
       !frame.data.info.tooManyFingers) ||
      frame.data.info.chargingMode > 4) {
    _error = IQS7211E_ERROR_I2C;
    return false;
  }
  _frame = frame;
  if (frame.data.info.showReset) {
    _initialized = false;
    _error = IQS7211E_ERROR_RESET;
    return false;
  }
  if (frame.data.info.atiError) {
    _error = IQS7211E_ERROR_ATI;
    return false;
  }
  _validData = true;
  _error = IQS7211E_ERROR_NONE;
  return true;
}

/** @brief Return the last valid frame's finger count. @return 0-2 normally;
 * check tooManyFingers() for over-limit input. Invalid data returns 0. */
uint8_t Adafruit_IQS7211E::getNumTouches() const {
  if (!_validData) {
    return 0;
  }
  return _frame.data.info.fingers;
}

/*!
 * @brief Check a tracked finger slot in the most recent acquisition.
 * @param finger Slot 0 or 1. A surviving second finger stays in slot 1.
 * @return True when that slot has a valid touch; false on invalid data/index.
 */
bool Adafruit_IQS7211E::isTouched(uint8_t finger) const {
  return _validData && finger < IQS7211E_MAX_TOUCHES &&
         !_frame.data.info.tooManyFingers && _frame.data.info.fingers != 0 &&
         _frame.data.finger[finger].area != 0;
}

/** @brief Get X coordinate. @param finger Slot 0 or 1. @return Coordinate, or 0
 * if not touched. */
uint16_t Adafruit_IQS7211E::getX(uint8_t finger) const {
  if (!isTouched(finger)) {
    return 0;
  }
  return _frame.data.finger[finger].x;
}

/** @brief Get Y coordinate. @param finger Slot 0 or 1. @return Coordinate, or 0
 * if not touched. */
uint16_t Adafruit_IQS7211E::getY(uint8_t finger) const {
  if (!isTouched(finger)) {
    return 0;
  }
  return _frame.data.finger[finger].y;
}

/** @brief Get the sum of finger channel deltas, not pressure. @param finger
 * Slot 0 or 1. @return Raw strength, or 0 if not touched. */
uint16_t Adafruit_IQS7211E::getTouchStrength(uint8_t finger) const {
  if (!isTouched(finger)) {
    return 0;
  }
  return _frame.data.finger[finger].strength;
}

/** @brief Get the number of channels under a finger. @param finger Slot 0 or 1.
 * @return Channel count, or 0 if not touched. */
uint16_t Adafruit_IQS7211E::getArea(uint8_t finger) const {
  if (!isTouched(finger)) {
    return 0;
  }
  return _frame.data.finger[finger].area;
}

/** @brief Check whether the finger limit was exceeded. @return True only for a
 * valid frame with excess fingers. */
bool Adafruit_IQS7211E::tooManyFingers() const {
  return _validData && _frame.data.info.tooManyFingers;
}

/** @brief Get the last acquisition/initialization outcome. @return Error code.
 */
iqs7211e_error_t Adafruit_IQS7211E::getLastError() const {
  return _error;
}

/** @brief Release the communication window (datasheet section 11.7). @return
 * True if the end command was acknowledged. */
bool Adafruit_IQS7211E::endCommunication() {
  Adafruit_BusIO_Register end(_i2c, IQS7211E_REG_END_COMMS);
  return end.write((uint32_t)0, 1);
}

/*!
 * @brief Read all channel counts in TX-major order (six RX rows per TX).
 * @param counts Destination for 42 counts.
 * @return True if the entire bank was read; false leaves partial data invalid.
 */
bool Adafruit_IQS7211E::readChannelCounts(uint16_t counts[IQS7211E_CHANNELS]) {
  return readChannelBank(IQS7211E_REG_COUNTS, counts);
}

/*!
 * @brief Read all channel reference counts in TX-major order.
 * @param references Destination for 42 reference values.
 * @return True if the entire bank was read.
 */
bool Adafruit_IQS7211E::readChannelReferences(
    uint16_t references[IQS7211E_CHANNELS]) {
  return readChannelBank(IQS7211E_REG_REFERENCES, references);
}

/*!
 * @brief Read channel ATI compensation, including after failed calibration.
 * @param compensation Destination for 42 values, in TX-major order.
 * @return True if the entire bank was read. Values 0 or 1023 are ATI limits.
 */
bool Adafruit_IQS7211E::readATICompensation(
    uint16_t compensation[IQS7211E_CHANNELS]) {
  return readChannelBank(IQS7211E_REG_COMPENSATION, compensation);
}

/*!
 * @brief Read an extended bank within one explicit communication window.
 * @param bank Extended bank address.
 * @param values Destination for all 42 little-endian channel words.
 * @return True on a complete read.
 */
bool Adafruit_IQS7211E::readChannelBank(uint8_t bank, uint16_t* values) {
  if (!_i2c || !values) {
    return false;
  }
  bool read = true;
  for (uint8_t offset = 0; offset < IQS7211E_CHANNELS && read; offset += 14) {
    // Extended pointers use bank then WORD offset, unlike ordinary registers.
    // Keep each read below AVR Wire's 32-byte limit. STOP does not end the
    // explicit communication window, so all chunks belong to the same scan.
    uint8_t pointer[2] = {bank, offset};
    read = _i2c->write_then_read(pointer, sizeof(pointer),
                                 (uint8_t*)(values + offset), 28);
  }
  bool ended = endCommunication();
  if (read && ended) {
    for (uint8_t i = 0; i < IQS7211E_CHANNELS; i++) {
      if (values[i] == IQS7211E_INVALID_WORD) {
        return false;
      }
    }
  }
  return read && ended;
}

/*!
 * @brief Set the coordinate ranges; this does not change physical resolution.
 * @param x Nonzero X coordinate range, default 1000.
 * @param y Nonzero Y coordinate range, default 1000.
 * @return True if both registers were written.
 * @note Both registers change within one communication window. Cached touch
 * data is invalidated because its coordinates use the previous ranges.
 */
bool Adafruit_IQS7211E::setResolution(uint16_t x, uint16_t y) {
  if (!_initialized || !x || !y) {
    return false;
  }
  _validData = false;
  Adafruit_BusIO_Register xRange(_i2c, IQS7211E_REG_X_RESOLUTION, 2, LSBFIRST);
  Adafruit_BusIO_Register yRange(_i2c, IQS7211E_REG_Y_RESOLUTION, 2, LSBFIRST);
  bool written = xRange.write(x) && yRange.write(y);
  bool ended = endCommunication();
  return written && ended;
}

/** @brief Read the configured X range. @return Range, or 0 on read failure. */
uint16_t Adafruit_IQS7211E::getXResolution() {
  if (!_initialized) {
    return 0;
  }
  Adafruit_BusIO_Register range(_i2c, IQS7211E_REG_X_RESOLUTION, 2, LSBFIRST);
  uint16_t value;
  bool read = range.read(&value);
  bool ended = endCommunication();
  if (!read || !ended || value == IQS7211E_INVALID_WORD) {
    return 0;
  }
  return value;
}

/** @brief Read the configured Y range. @return Range, or 0 on read failure. */
uint16_t Adafruit_IQS7211E::getYResolution() {
  if (!_initialized) {
    return 0;
  }
  Adafruit_BusIO_Register range(_i2c, IQS7211E_REG_Y_RESOLUTION, 2, LSBFIRST);
  uint16_t value;
  bool read = range.read(&value);
  bool ended = endCommunication();
  if (!read || !ended || value == IQS7211E_INVALID_WORD) {
    return 0;
  }
  return value;
}

/*!
 * @brief Set touch sensitivity and release hysteresis (datasheet section 5.4).
 * @param set Touch threshold as a fraction of reference counts, in 1/128 units.
 * Range 1-255; smaller values are more sensitive. Default 24.
 * @param clear Release threshold, in 1/128 units. Must be below set.
 * Default 20.
 * @return True if both fields were written.
 */
bool Adafruit_IQS7211E::setTouchThresholds(uint8_t set, uint8_t clear) {
  if (!_initialized || !set || clear >= set) {
    return false;
  }
  Adafruit_BusIO_Register thresholds(_i2c, IQS7211E_REG_THRESHOLDS, 2,
                                     LSBFIRST);
  Adafruit_BusIO_RegisterBits setThreshold(&thresholds, 8, 0);
  Adafruit_BusIO_RegisterBits clearThreshold(&thresholds, 8, 8);
  bool written = setThreshold.write(set) && clearThreshold.write(clear);
  bool ended = endCommunication();
  return written && ended;
}

/*!
 * @brief Read the touch set threshold in 1/128 units.
 * @return Threshold; 255 also indicates a failed read (BusIO field sentinel).
 */
uint8_t Adafruit_IQS7211E::getTouchSetThreshold() {
  if (!_initialized) {
    return 255;
  }
  Adafruit_BusIO_Register thresholds(_i2c, IQS7211E_REG_THRESHOLDS, 2,
                                     LSBFIRST);
  Adafruit_BusIO_RegisterBits setThreshold(&thresholds, 8, 0);
  uint8_t value = setThreshold.read();
  if (!endCommunication()) {
    return 255;
  }
  return value;
}

/*!
 * @brief Read the touch release threshold in 1/128 units.
 * @return Threshold; 255 also indicates a failed read (BusIO field sentinel).
 */
uint8_t Adafruit_IQS7211E::getTouchClearThreshold() {
  if (!_initialized) {
    return 255;
  }
  Adafruit_BusIO_Register thresholds(_i2c, IQS7211E_REG_THRESHOLDS, 2,
                                     LSBFIRST);
  Adafruit_BusIO_RegisterBits clearThreshold(&thresholds, 8, 8);
  uint8_t value = clearThreshold.read();
  if (!endCommunication()) {
    return 255;
  }
  return value;
}

/*!
 * @brief Set the active-mode report interval, not a software polling delay.
 * @param milliseconds Nonzero requested interval, default 20 ms. The actual
 * interval cannot be shorter than the chip's electrode scan time.
 * @return True if the register was written.
 */
bool Adafruit_IQS7211E::setReportRate(uint16_t milliseconds) {
  if (!_initialized || !milliseconds) {
    return false;
  }
  Adafruit_BusIO_Register rate(_i2c, IQS7211E_REG_REPORT_RATE, 2, LSBFIRST);
  bool written = rate.write(milliseconds);
  bool ended = endCommunication();
  return written && ended;
}

/** @brief Read the requested report interval. @return Milliseconds, or 0 on
 * failure. */
uint16_t Adafruit_IQS7211E::getReportRate() {
  if (!_initialized) {
    return 0;
  }
  Adafruit_BusIO_Register rate(_i2c, IQS7211E_REG_REPORT_RATE, 2, LSBFIRST);
  uint16_t value;
  bool read = rate.read(&value);
  bool ended = endCommunication();
  if (!read || !ended || value == IQS7211E_INVALID_WORD) {
    return 0;
  }
  return value;
}

/*!
 * @brief Configure output axes (datasheet section 7.5).
 * @param flipX Reverse X coordinates.
 * @param flipY Reverse Y coordinates.
 * @param swapXY True makes TX columns the X axis on this board (default).
 * @return True if all three fields were written.
 */
bool Adafruit_IQS7211E::setOrientation(bool flipX, bool flipY, bool swapXY) {
  if (!_initialized) {
    return false;
  }
  _validData = false;
  Adafruit_BusIO_Register trackpad(_i2c, IQS7211E_REG_TRACKPAD, 2, LSBFIRST);
  Adafruit_BusIO_RegisterBits xReverse(&trackpad, 1, 0);
  Adafruit_BusIO_RegisterBits yReverse(&trackpad, 1, 1);
  Adafruit_BusIO_RegisterBits swap(&trackpad, 1, 2);
  bool written =
      xReverse.write(flipX) && yReverse.write(flipY) && swap.write(swapXY);
  bool ended = endCommunication();
  return written && ended;
}
