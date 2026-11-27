/*!
 * @file Adafruit_IQS7211E.cpp
 * @brief Polling driver for the Azoteq IQS7211E capacitive trackpad.
 * Written by Adafruit. MIT license; see LICENSE.
 */
#include "Adafruit_IQS7211E.h"

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
 * If calibration fails after configuration, setters remain available so the
 * application can adjust the settings and call calibrate() again.
 * @note Defaults: active mode, 20 ms reports, 0-1000 coordinates, two fingers,
 * no gestures or low-power modes. AVR Wire timeout is set to 250 ms because
 * the chip stretches SCL. Other cores must support appropriate clock
 * stretching.
 */
bool Adafruit_IQS7211E::begin(TwoWire* wire, int16_t readyPin) {
  _initialized = false;
  _configuring = false;
  _configurationFailed = false;
  _validData = false;
  // Keep an existing RDY pin long enough to leave request mode, even when
  // the caller is reinitializing into polling mode with the default -1.
  if (!_communicationRequest || readyPin >= 0) {
    _readyPin = readyPin;
  }
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

  // Reinitialization must also work after enabling explicit requests. The
  // old transport mode remains in force until reset() replaces it below.
  if (_communicationRequest) {
    if (_readyPin < 0) {
      return argumentError();
    }
    uint32_t start = millis();
    while (!prepareCommunication(false)) {
      if (_error != IQS7211E_ERROR_NOT_READY) {
        return false;
      }
      if (millis() - start >= 3000) {
        _error = IQS7211E_ERROR_TIMEOUT;
        return false;
      }
      delay(1);
    }
  }

  // The product number, not an ACK alone, identifies the peripheral.
  if (getProductNumber() != IQS7211E_PRODUCT_NUMBER) {
    _error = IQS7211E_ERROR_DEVICE;
    return false;
  }
  if (!reset()) {
    return false;
  }
  _readyPin = readyPin;
  // Establish an explicit window before replacing the complete writable map
  // (datasheet section 10.1.2). No partially configured sensor is scanned.
  Adafruit_BusIO_Register config(_i2c, IQS7211E_REG_CONFIG, 2, LSBFIRST);
  if (!config.write(IQS7211E_CONFIG_POLLING)) {
    _error = IQS7211E_ERROR_I2C;
    return false;
  }
  _commandTermination = true;
  _communicationRequest = false;
  _configuring = true;
  _configurationFailed = false;

  // Clear reserved bits as well as writable settings. Keep communication
  // control and its timeout intact until their named setters run below.
  for (uint8_t address = IQS7211E_REG_CONFIG_START;
       address <= IQS7211E_REG_GESTURE_SETTINGS; address++) {
    if (address == IQS7211E_REG_CONFIG ||
        address == IQS7211E_REG_COMMUNICATION_TIMEOUT) {
      continue;
    }
    Adafruit_BusIO_Register setting(_i2c, address, 2, LSBFIRST);
    if (!finishSetting(setting.write((uint16_t)0))) {
      endConfiguration();
      return false;
    }
  }

  // Calibration and acquisition defaults for the Adafruit 7 TX by 6 RX pad.
  bool configured =
      setALPCompensation(IQS7211E_ENGINE_A, 0) &&
      setALPCompensation(IQS7211E_ENGINE_B, 0) && setATICoarseDivider(1) &&
      setATICoarseMultiplier(4) && setATIFineDivider(5) &&
      setATICompensationDivider(6) && setATIDriftLimit(50) &&
      setATITarget(300) && setMinimumCountReATI(50) &&
      setATICoarseDivider(3, IQS7211E_ALP) &&
      setATICoarseMultiplier(1, IQS7211E_ALP) &&
      setATIFineDivider(1, IQS7211E_ALP) &&
      setATICompensationDivider(4, IQS7211E_ALP) &&
      setATIDriftLimit(20, IQS7211E_ALP) && setATITarget(200, IQS7211E_ALP);

  configured = configured && setReportRate(20) &&
               setReportRate(50, IQS7211E_MODE_IDLE_TOUCH) &&
               setReportRate(20, IQS7211E_MODE_IDLE) &&
               setReportRate(80, IQS7211E_MODE_LP1) &&
               setReportRate(160, IQS7211E_MODE_LP2) &&
               setModeTimeout(IQS7211E_MODE_ACTIVE, 0) &&
               setModeTimeout(IQS7211E_MODE_IDLE_TOUCH, 0) &&
               setModeTimeout(IQS7211E_MODE_IDLE, 0) &&
               setModeTimeout(IQS7211E_MODE_LP1, 0) &&
               setReferenceUpdateTime(8) && setATIRetryTime(5) &&
               setCommunicationTimeout(100) &&
               setPowerMode(IQS7211E_MODE_ACTIVE) && enableTXTest(false) &&
               enableManualMode(true) && enableEventMode(false) &&
               enableWatchdog(true) && enableCommandTermination(true) &&
               enableCommunicationRequest(false) && enableAutoATI(true) &&
               enableAutoATI(false, IQS7211E_ALP) &&
               enableEvent(IQS7211E_EVENT_GESTURE, false) &&
               enableEvent(IQS7211E_EVENT_TRACKPAD, false) &&
               enableEvent(IQS7211E_EVENT_ATI, false) &&
               enableEvent(IQS7211E_EVENT_ALP, false) &&
               enableEvent(IQS7211E_EVENT_TOUCH, false) &&
               setOscillator(IQS7211E_OSC_14MHZ) && setOscillatorAdjustment(0);

  // ALP is configured but inactive while manual active mode is selected.
  configured = configured && setALPSensingMethod(IQS7211E_MUTUAL_CAPACITIVE) &&
               enableALPCountFilter(true) && setALPThreshold(8) &&
               setALPDebounce(4, 4) && setALPCountBeta(IQS7211E_LP1, 220) &&
               setALPReferenceBeta(IQS7211E_LP1, 8) &&
               setALPCountBeta(IQS7211E_LP2, 240) &&
               setALPReferenceBeta(IQS7211E_LP2, 16);
  for (uint8_t pin = 0; pin < 8 && configured; pin++) {
    bool enabled =
        pin == 0 || pin == 1 || pin == 2 || pin == 4 || pin == 5 || pin == 6;
    configured = enableALPReceiver(pin, enabled);
  }
  for (uint8_t pin = 0; pin < IQS7211E_MAX_ELECTRODES && configured; pin++) {
    bool enabled = pin == 3 || pin >= 7;
    configured = enableALPTransmitter(pin, enabled);
  }

  for (uint8_t index = 0; index < 2 && configured; index++) {
    iqs7211e_sensor_t sensor = (iqs7211e_sensor_t)index;
    configured = setConversionPeriod(26, sensor) &&
                 setConversionFraction(127, sensor) &&
                 enableNMStatic(false, sensor) &&
                 enableHalfVoltDischarge(false, sensor) &&
                 enableRFFilter(false, sensor) &&
                 setChargeCapacitor(IQS7211E_CAP_80PF, sensor) &&
                 setOpampBias(IQS7211E_BIAS_10UA, sensor) &&
                 setCountLimit(IQS7211E_COUNT_1023, sensor) &&
                 setInitialDelay(IQS7211E_INIT_64, sensor);
  }
  configured = configured &&
               setAutoProxCycles(IQS7211E_LP1, IQS7211E_AUTOPROX_DISABLED) &&
               setAutoProxCycles(IQS7211E_LP2, IQS7211E_AUTOPROX_DISABLED) &&
               setElectrodeCounts(6, 7) && setMaxTouches(2) &&
               setTouchThresholds(24, 20) && setResolution(1000, 1000) &&
               setOrientation(false, false, true) && enableMAVFilter(true) &&
               enableIIRFilter(true) && enableStaticIIR(false) &&
               setDynamicFilterSpeeds(6, 124) && setDynamicFilterBeta(7) &&
               setStaticFilterBeta(128) && setFingerSplitFactor(1) &&
               setStationaryThreshold(20) && setEdgeTrim(IQS7211E_AXIS_X, 0) &&
               setEdgeTrim(IQS7211E_AXIS_Y, 0) && setSettingsVersion(0, 2);

  // Gesture detection is disabled; these are the starting values if enabled.
  configured = configured &&
               enableGesture(IQS7211E_GESTURE_SINGLE_TAP, false) &&
               enableGesture(IQS7211E_GESTURE_DOUBLE_TAP, false) &&
               enableGesture(IQS7211E_GESTURE_TRIPLE_TAP, false) &&
               enableGesture(IQS7211E_GESTURE_HOLD, false) &&
               enableGesture(IQS7211E_GESTURE_PALM, false) &&
               enableGesture(IQS7211E_GESTURE_SWIPE_X_POS, false) &&
               enableGesture(IQS7211E_GESTURE_SWIPE_X_NEG, false) &&
               enableGesture(IQS7211E_GESTURE_SWIPE_Y_POS, false) &&
               enableGesture(IQS7211E_GESTURE_SWIPE_Y_NEG, false) &&
               enableGesture(IQS7211E_GESTURE_SWIPE_HOLD_X_POS, false) &&
               enableGesture(IQS7211E_GESTURE_SWIPE_HOLD_X_NEG, false) &&
               enableGesture(IQS7211E_GESTURE_SWIPE_HOLD_Y_POS, false) &&
               enableGesture(IQS7211E_GESTURE_SWIPE_HOLD_Y_NEG, false) &&
               setTapTime(150) && setAirTime(150) && setTapDistance(50) &&
               setHoldTime(300) && setSwipeTime(150) &&
               setSwipeInitialDistance(IQS7211E_AXIS_X, 200) &&
               setSwipeInitialDistance(IQS7211E_AXIS_Y, 200) &&
               setSwipeConsecutiveDistance(IQS7211E_AXIS_X, 100) &&
               setSwipeConsecutiveDistance(IQS7211E_AXIS_Y, 100) &&
               setSwipeAngle(23) && setPalmThreshold(30);

  // RX rows bottom-to-top, then TX columns left-to-right.
  const uint8_t mapping[IQS7211E_MAX_ELECTRODES] = {0,  1,  2, 6, 4, 5, 12,
                                                    11, 10, 9, 8, 7, 3};
  configured = configured && setElectrodeMapping(mapping);
  // Pair one block A row with one block B row on the same transmitter.
  for (uint8_t cycle = 0; cycle < IQS7211E_SENSING_CYCLES && configured;
       cycle++) {
    uint8_t channel = (cycle / 3) * 6 + cycle % 3;
    configured = setSensingCycle(cycle, channel, channel + 3);
  }
  if (!configured) {
    endConfiguration();
    return false;
  }

  // Section 9.3.1: acknowledge reset only after replacing the settings.
  Adafruit_BusIO_Register system(_i2c, IQS7211E_REG_SYSTEM, 2, LSBFIRST);
  bool written = system.write(IQS7211E_COMMAND_ACK_RESET);
  finishSetting(written);
  bool ended = endConfiguration();
  if (!written || !ended) {
    _error = IQS7211E_ERROR_I2C;
    return false;
  }

  _initialized = true;
  if (!calibrate()) {
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
  if (!prepareCommunication(false)) {
    return 0;
  }
  Adafruit_BusIO_Register product(_i2c, IQS7211E_REG_PRODUCT, 2, LSBFIRST);
  uint16_t value;
  bool read = product.read(&value);
  if (!finishSetting(read && value != IQS7211E_INVALID_WORD)) {
    return 0;
  }
  return value;
}

/** @brief Read firmware major version. @return Version or -1 on read failure.
 */
int16_t Adafruit_IQS7211E::getFirmwareMajor() {
  if (!prepareCommunication(false)) {
    return -1;
  }
  Adafruit_BusIO_Register version(_i2c, IQS7211E_REG_VERSION_MAJOR, 2,
                                  LSBFIRST);
  uint16_t value;
  bool read = version.read(&value);
  if (!finishSetting(read && value <= 255)) {
    return -1;
  }
  return value;
}

/** @brief Read firmware minor version. @return Version or -1 on read failure.
 */
int16_t Adafruit_IQS7211E::getFirmwareMinor() {
  if (!prepareCommunication(false)) {
    return -1;
  }
  Adafruit_BusIO_Register version(_i2c, IQS7211E_REG_VERSION_MINOR, 2,
                                  LSBFIRST);
  uint16_t value;
  bool read = version.read(&value);
  if (!finishSetting(read && value <= 255)) {
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
  if (_configuring) {
    return argumentError();
  }
  if (_communicationRequest && !prepareCommunication(false)) {
    return false;
  }
  _initialized = false;
  _validData = false;
  if (!_i2c) {
    _error = IQS7211E_ERROR_NOT_INITIALIZED;
    return false;
  }
  // Establish explicit window termination even after a previous host restart.
  Adafruit_BusIO_Register config(_i2c, IQS7211E_REG_CONFIG, 2, LSBFIRST);
  bool written = config.write(IQS7211E_CONFIG_POLLING);
  if (written) {
    _commandTermination = true;
    _communicationRequest = false;
  }
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
 * @brief Recalibrate the electrodes with a bounded wait; leave them untouched.
 * @param timeoutMS Maximum wait in milliseconds, excluding one bus transaction.
 * @return True when trackpad ATI finishes without an ATI error.
 * @note ATI compensation is volatile. This does not write OTP or flash.
 */
bool Adafruit_IQS7211E::calibrate(uint16_t timeoutMS) {
  _validData = false;
  if (_configuring) {
    return argumentError();
  }
  if (!_initialized) {
    _error = IQS7211E_ERROR_NOT_INITIALIZED;
    return false;
  }
  // Section 5.6.3: this command clears itself when trackpad ATI has completed.
  Adafruit_BusIO_Register system(_i2c, IQS7211E_REG_SYSTEM, 2, LSBFIRST);
  if (!queueATI()) {
    return false;
  }
  uint32_t start = millis();
  bool complete = false;
  while (millis() - start < timeoutMS) {
    delay(10);
    if (!prepareCommunication()) {
      continue;
    }
    if (!complete) {
      iqs7211e_system_t command;
      bool read = system.read(&command.value);
      if (!read || command.value == IQS7211E_INVALID_WORD) {
        endCommunication();
        continue; // Comms is unavailable while the ATI routine runs.
      }
      // A checked snapshot distinguishes an invalid window from an ATI bit.
      complete = !command.bits.trackpadATI;
      if (!complete) {
        if (!endCommunication()) {
          _error = IQS7211E_ERROR_I2C;
          return false;
        }
        continue;
      }
    }
    // In command mode this uses the same window. In STOP mode the status
    // read closed that window, so wait for a new RDY without consuming it
    // with another status read. Only calibration uses this bounded retry.
    if (readData()) {
      return true;
    }
    if (_error != IQS7211E_ERROR_NOT_READY) {
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
  if (_configuring) {
    _error = IQS7211E_ERROR_NOT_READY;
    return false;
  }
  if (!prepareCommunication()) {
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

/** @brief Get the last device-operation outcome. @return Error code.
 */
iqs7211e_error_t Adafruit_IQS7211E::getLastError() const {
  return _error;
}

/** @brief Release the communication window (datasheet section 11.7). @return
 * True if the end command was acknowledged. */
bool Adafruit_IQS7211E::endCommunication() {
  if (!_i2c) {
    return false;
  }
  if (_configuring || !_commandTermination) {
    return true;
  }
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
  if (!values) {
    return argumentError();
  }
  // Multiple chunks must share one scan. STOP-terminated windows cannot
  // guarantee this, so require explicit command termination for bank reads.
  if (!_commandTermination) {
    return argumentError();
  }
  if (!prepareCommunication(false)) {
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
  if (read) {
    for (uint8_t i = 0; i < IQS7211E_CHANNELS; i++) {
      if (values[i] == IQS7211E_INVALID_WORD) {
        read = false;
        break;
      }
    }
  }
  return finishSetting(read);
}
