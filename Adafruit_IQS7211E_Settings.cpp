/*!
 * @file Adafruit_IQS7211E_Settings.cpp
 * @brief Runtime configuration for the IQS7211E (datasheet v1.6, Appendix A).
 */
#include "Adafruit_IQS7211E.h"

/*!
 * @brief Set the logical electrode dimensions.
 * @param receivers RX count, 1-8.
 * @param transmitters TX count, 1-12.
 * @return True on success; requires an open beginConfiguration() group.
 * @note At most 13 pins and 42 intersections. Update mapping and cycle
 * allocation in the same group, then endConfiguration() and calibrate().
 */
bool Adafruit_IQS7211E::setElectrodeCounts(uint8_t receivers,
                                           uint8_t transmitters) {
  if (!_configuring || receivers < 1 || receivers > 8 || transmitters < 1 ||
      transmitters > 12 || receivers + transmitters > IQS7211E_MAX_ELECTRODES ||
      receivers * transmitters > IQS7211E_CHANNELS) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register rx(_i2c, IQS7211E_REG_TRACKPAD, 2, LSBFIRST);
  Adafruit_BusIO_Register tx(_i2c, IQS7211E_REG_TX_COUNT, 2, LSBFIRST);
  uint16_t previous;
  if (!rx.read(&previous) || previous == IQS7211E_INVALID_WORD ||
      !tx.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits rxCount(&rx, 8, 8);
  Adafruit_BusIO_RegisterBits txCount(&tx, 8, 0);
  return finishSetting(rxCount.write(receivers) && txCount.write(transmitters),
                       true);
}

/*!
 * @brief Map logical electrodes to physical RX/TX pins (section 7.1).
 * @param mapping Thirteen physical pins, RX entries followed by TX entries;
 * unused entries may be IQS7211E_CHANNEL_DISABLED.
 * @return True on success; requires an open beginConfiguration() group.
 * @note RX pins must be 0-7. Pins cannot be duplicated. Mapping does not
 * reroute the PCB: cycles must pair one RX0-3 channel with one RX4-7 channel.
 */
bool Adafruit_IQS7211E::setElectrodeMapping(
    const uint8_t mapping[IQS7211E_MAX_ELECTRODES]) {
  if (!_configuring || !mapping) {
    return argumentError();
  }
  for (uint8_t i = 0; i < IQS7211E_MAX_ELECTRODES; i++) {
    if (mapping[i] == IQS7211E_CHANNEL_DISABLED) {
      continue;
    }
    if (mapping[i] >= IQS7211E_MAX_ELECTRODES) {
      return argumentError();
    }
    for (uint8_t j = 0; j < i; j++) {
      if (mapping[i] == mapping[j]) {
        return argumentError();
      }
    }
  }
  uint8_t receivers = getReceiverCount();
  if (_error != IQS7211E_ERROR_NONE) {
    return false;
  }
  uint8_t transmitters = getTransmitterCount();
  if (_error != IQS7211E_ERROR_NONE) {
    return false;
  }
  if (receivers < 1 || receivers > 8 || transmitters < 1 ||
      receivers + transmitters > IQS7211E_MAX_ELECTRODES ||
      receivers * transmitters > IQS7211E_CHANNELS) {
    return argumentError();
  }
  for (uint8_t i = 0; i < receivers + transmitters; i++) {
    if (mapping[i] == IQS7211E_CHANNEL_DISABLED ||
        (i < receivers && mapping[i] > 7)) {
      return argumentError();
    }
  }
  uint8_t bytes[14] = {};
  memcpy(bytes, mapping, IQS7211E_MAX_ELECTRODES);
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_MAPPING, 2, LSBFIRST);
  return finishSetting(reg.write(bytes, sizeof(bytes)), true);
}

/*!
 * @brief Read the physical electrode mapping.
 * @param mapping Destination for thirteen pin numbers.
 * @return True on a checked read; false leaves the destination unchanged.
 */
bool Adafruit_IQS7211E::getElectrodeMapping(
    uint8_t mapping[IQS7211E_MAX_ELECTRODES]) {
  if (!mapping) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  uint8_t bytes[14];
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_MAPPING, 2, LSBFIRST);
  bool read = reg.read(bytes, sizeof(bytes));
  if (read) {
    for (uint8_t i = 0; i < IQS7211E_MAX_ELECTRODES; i++) {
      if (bytes[i] >= IQS7211E_MAX_ELECTRODES &&
          bytes[i] != IQS7211E_CHANNEL_DISABLED) {
        read = false;
      }
    }
  }
  if (!finishSetting(read)) {
    return false;
  }
  memcpy(mapping, bytes, IQS7211E_MAX_ELECTRODES);
  return true;
}

/*!
 * @brief Allocate one two-engine sensing cycle (sections 7.1 and 12).
 * @param cycle Cycle index, 0-20.
 * @param channelA Logical channel using physical RX0-3, or 255 to disable.
 * @param channelB Logical channel using physical RX4-7, or 255 to disable.
 * @return True on success; requires an open beginConfiguration() group.
 * @note Channels are TX-major: TX index times RX count plus RX index.
 * Required control bytes are supplied automatically. The caller must allocate
 * every active channel exactly once across the 21 cycles, then recalibrate.
 */
bool Adafruit_IQS7211E::setSensingCycle(uint8_t cycle, uint8_t channelA,
                                        uint8_t channelB) {
  if (!_configuring || cycle >= IQS7211E_SENSING_CYCLES ||
      (channelA >= IQS7211E_CHANNELS &&
       channelA != IQS7211E_CHANNEL_DISABLED) ||
      (channelB >= IQS7211E_CHANNELS &&
       channelB != IQS7211E_CHANNEL_DISABLED)) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  uint8_t receivers = getReceiverCount();
  if (_error != IQS7211E_ERROR_NONE) {
    return false;
  }
  uint8_t transmitters = getTransmitterCount();
  if (_error != IQS7211E_ERROR_NONE) {
    return false;
  }
  if (receivers < 1 || receivers > 8 || transmitters < 1 ||
      receivers + transmitters > IQS7211E_MAX_ELECTRODES ||
      receivers * transmitters > IQS7211E_CHANNELS) {
    return argumentError();
  }
  uint8_t mapping[IQS7211E_MAX_ELECTRODES];
  if (!getElectrodeMapping(mapping)) {
    return false;
  }
  // Section 7.1.2: the engines have separate RX banks but share one TX.
  if (channelA != IQS7211E_CHANNEL_DISABLED &&
      (channelA >= receivers * transmitters ||
       mapping[channelA % receivers] > 3)) {
    return argumentError();
  }
  if (channelB != IQS7211E_CHANNEL_DISABLED &&
      (channelB >= receivers * transmitters ||
       mapping[channelB % receivers] < 4 ||
       mapping[channelB % receivers] > 7)) {
    return argumentError();
  }
  if (channelA != IQS7211E_CHANNEL_DISABLED &&
      channelB != IQS7211E_CHANNEL_DISABLED &&
      channelA / receivers != channelB / receivers) {
    return argumentError();
  }
  // Each cycle occupies three bytes in a word-addressed memory map.
  uint8_t bytes[3] = {IQS7211E_CYCLE_CONTROL, channelA, channelB};
  for (uint8_t i = 0; i < 3; i++) {
    uint8_t offset = cycle * 3 + i;
    Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_CYCLES + offset / 2, 2,
                                LSBFIRST);
    uint16_t previous;
    if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
      return finishSetting(false, true);
    }
    Adafruit_BusIO_RegisterBits byte(&reg, 8, 8 * (offset % 2));
    if (!byte.write(bytes[i])) {
      return finishSetting(false, true);
    }
  }
  if (cycle == IQS7211E_SENSING_CYCLES - 1) {
    Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_CYCLES + 31, 2, LSBFIRST);
    Adafruit_BusIO_RegisterBits finalControl(&reg, 8, 8);
    if (!finalControl.write(IQS7211E_FINAL_CYCLE_CONTROL)) {
      return finishSetting(false, true);
    }
  }
  return finishSetting(true, true);
}

/*!
 * @brief Read one engine's logical channel allocation.
 * @param cycle Cycle index, 0-20.
 * @param engine Engine A or B.
 * @return Channel, or 255 if disabled or failed; check getLastError().
 */
uint8_t Adafruit_IQS7211E::getSensingCycleChannel(uint8_t cycle,
                                                  iqs7211e_engine_t engine) {
  if (cycle >= IQS7211E_SENSING_CYCLES) {
    argumentError();
    return IQS7211E_CHANNEL_DISABLED;
  }
  if (!prepareCommunication()) {
    return IQS7211E_CHANNEL_DISABLED;
  }
  uint8_t offset = cycle * 3 + 1 + engine;
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_CYCLES + offset / 2, 2,
                              LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return IQS7211E_CHANNEL_DISABLED;
  }
  if (offset % 2) {
    return value.bytes.high;
  }
  return value.bytes.low;
}
/*!
 * @brief Enable automatic recalibration.
 * @param enabled True to enable.
 * @param sensor Trackpad or ALP.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::enableAutoATI(bool enabled, iqs7211e_sensor_t sensor) {
  if (!prepareCommunication()) {
    return false;
  }
  uint8_t position = 2;
  if (sensor == IQS7211E_ALP) {
    position = 3;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_CONFIG, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits autoATI(&reg, 1, position);
  bool written = autoATI.write(enabled);
  return finishSetting(written, true);
}

/*!
 * @brief Select ALP Auto-Prox cycles.
 * @param mode LP1 or LP2.
 * @param cycles Cycle count or disabled.
 * @return True on success; see getLastError() on failure.
 * @note Use RDY and communication-request mode with Auto-Prox; firmware 1.1 can
 * otherwise stretch SCL for long periods.
 */
bool Adafruit_IQS7211E::setAutoProxCycles(iqs7211e_low_power_t mode,
                                          iqs7211e_autoprox_t cycles) {
  if (!prepareCommunication()) {
    return false;
  }
  uint8_t position = 2;
  if (mode == IQS7211E_LP2) {
    position = 5;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_ALP_HARDWARE, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits autoProx(&reg, 3, position);
  bool written = autoProx.write(cycles);
  return finishSetting(written, true);
}

/*!
 * @brief Set coordinate edge trimming.
 * @param axis X or Y.
 * @param trim Trim in coordinate units.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setEdgeTrim(iqs7211e_axis_t axis, uint8_t trim) {
  if (!prepareCommunication()) {
    return false;
  }
  uint8_t position = 0;
  if (axis == IQS7211E_AXIS_Y) {
    position = 8;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_EDGE_TRIM, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits edgeTrim(&reg, 8, position);
  bool written = edgeTrim.write(trim);
  return finishSetting(written, true);
}

/*!
 * @brief Configure output axes.
 * @param flipX Reverse X.
 * @param flipY Reverse Y.
 * @param swapXY Swap axes; true makes TX the X axis on this board.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setOrientation(bool flipX, bool flipY, bool swapXY) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_TRACKPAD, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits xReverse(&reg, 1, 0);
  Adafruit_BusIO_RegisterBits yReverse(&reg, 1, 1);
  Adafruit_BusIO_RegisterBits swap(&reg, 1, 2);
  bool written =
      xReverse.write(flipX) && yReverse.write(flipY) && swap.write(swapXY);
  return finishSetting(written, true);
}

/*!
 * @brief Set touch sensitivity and release hysteresis.
 * @param set Set threshold in 1/128 units, 1-255; lower is more sensitive.
 * @param clear Release threshold in 1/128 units, below set.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setTouchThresholds(uint8_t set, uint8_t clear) {
  if (set == 0 || clear >= set) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_THRESHOLDS, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setThreshold(&reg, 8, 0);
  Adafruit_BusIO_RegisterBits clearThreshold(&reg, 8, 8);
  bool written = setThreshold.write(set) && clearThreshold.write(clear);
  return finishSetting(written, true);
}

/*!
 * @brief Set ALP detection debounce.
 * @param setCycles Consecutive cycles before asserting ALP.
 * @param clearCycles Consecutive cycles before clearing ALP.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setALPDebounce(uint8_t setCycles, uint8_t clearCycles) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_ALP_DEBOUNCE, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setDebounce(&reg, 8, 0);
  Adafruit_BusIO_RegisterBits clearDebounce(&reg, 8, 8);
  bool written =
      setDebounce.write(setCycles) && clearDebounce.write(clearCycles);
  return finishSetting(written, true);
}

/*!
 * @brief Write the settings version marker.
 * @param major User-defined major settings version.
 * @param minor User-defined minor settings version.
 * @return True on success; see getLastError() on failure.
 * @note This is a user marker, independent of the library and firmware
 * versions.
 */
bool Adafruit_IQS7211E::setSettingsVersion(uint8_t major, uint8_t minor) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_SETTINGS_VERSION, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits majorVersion(&reg, 8, 8);
  Adafruit_BusIO_RegisterBits minorVersion(&reg, 8, 0);
  bool written = majorVersion.write(major) && minorVersion.write(minor);
  return finishSetting(written, true);
}

/*!
 * @brief Set a hardware report interval in milliseconds.
 * @param milliseconds Nonzero interval; actual timing is limited by scan
 * duration.
 * @param mode Charging mode, active by default.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setReportRate(uint16_t milliseconds,
                                      iqs7211e_mode_t mode) {
  if (milliseconds == 0) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_REPORT_RATE + mode, 2,
                              LSBFIRST);
  return finishSetting(reg.write(milliseconds), true);
}

/*!
 * @brief Set an automatic mode-transition timeout.
 * @param mode Active, idle-touch, idle or LP1; LP2 has no timeout.
 * @param seconds Seconds before the next mode; zero disables transitions.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setModeTimeout(iqs7211e_mode_t mode, uint16_t seconds) {
  if (mode == IQS7211E_MODE_LP2) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_MODE_TIMEOUT + mode, 2,
                              LSBFIRST);
  return finishSetting(reg.write(seconds), true);
}

/*!
 * @brief Set output coordinate ranges.
 * @param x Nonzero X coordinate range.
 * @param y Nonzero Y coordinate range.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setResolution(uint16_t x, uint16_t y) {
  if (x == 0 || y == 0) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register first(_i2c, IQS7211E_REG_X_RESOLUTION, 2, LSBFIRST);
  Adafruit_BusIO_Register second(_i2c, IQS7211E_REG_Y_RESOLUTION, 2, LSBFIRST);
  return finishSetting(first.write(x) && second.write(y), true);
}

/*!
 * @brief Set the dynamic IIR speed range.
 * @param bottom Bottom speed in coordinate units per report.
 * @param top Top speed, greater than bottom.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setDynamicFilterSpeeds(uint16_t bottom, uint16_t top) {
  if (top <= bottom) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register first(_i2c, IQS7211E_REG_FILTER_BOTTOM_SPEED, 2,
                                LSBFIRST);
  Adafruit_BusIO_Register second(_i2c, IQS7211E_REG_FILTER_TOP_SPEED, 2,
                                 LSBFIRST);
  return finishSetting(first.write(bottom) && second.write(top), true);
}

/*!
 * @brief Select the manual charging mode.
 * @param mode Requested mode; enableManualMode(true) makes it effective.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setPowerMode(iqs7211e_mode_t mode) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_SYSTEM, 2, LSBFIRST);
  iqs7211e_system_t previous;
  if (!reg.read(&previous.value) || previous.value == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  // Table A.6 mixes commands and persistent fields. Build a fresh command
  // so a read/modify/write cannot replay reset, reseed or ATI requests.
  iqs7211e_system_t command = {};
  command.bits.mode = previous.bits.mode;
  command.bits.txTest = previous.bits.txTest;
  command.bits.mode = mode;
  return finishSetting(reg.write(command.value), true);
}

/*!
 * @brief Enable the TX short test.
 * @param enabled True to enable the TX short diagnostic.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::enableTXTest(bool enabled) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_SYSTEM, 2, LSBFIRST);
  iqs7211e_system_t previous;
  if (!reg.read(&previous.value) || previous.value == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  // Table A.6 mixes commands and persistent fields. Build a fresh command
  // so a read/modify/write cannot replay reset, reseed or ATI requests.
  iqs7211e_system_t command = {};
  command.bits.mode = previous.bits.mode;
  command.bits.txTest = previous.bits.txTest;
  command.bits.txTest = enabled;
  return finishSetting(reg.write(command.value), true);
}

/*!
 * @brief Queue calibration without waiting for it to finish.
 * @param sensor Trackpad or ALP.
 * @return True on success; see getLastError() on failure.
 * @note End grouped settings first. TP ATI runs in trackpad modes; ALP ATI
 * requires LP1/LP2. No I2C access is available during ATI.
 */
bool Adafruit_IQS7211E::queueATI(iqs7211e_sensor_t sensor) {
  if (_configuring) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_SYSTEM, 2, LSBFIRST);
  iqs7211e_system_t previous;
  if (!reg.read(&previous.value) || previous.value == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  // Table A.6 mixes commands and persistent fields. Build a fresh command
  // so a read/modify/write cannot replay reset, reseed or ATI requests.
  iqs7211e_system_t command = {};
  command.bits.mode = previous.bits.mode;
  command.bits.txTest = previous.bits.txTest;
  if (sensor == IQS7211E_ALP) {
    command.bits.alpATI = true;
  } else {
    command.bits.trackpadATI = true;
  }
  return finishSetting(reg.write(command.value), true);
}

/*!
 * @brief Reseed the reference counts.
 * @param sensor Trackpad or ALP.
 * @return True on success; see getLastError() on failure.
 * @note End grouped settings first; leave the electrodes untouched.
 */
bool Adafruit_IQS7211E::reseed(iqs7211e_sensor_t sensor) {
  if (_configuring) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_SYSTEM, 2, LSBFIRST);
  iqs7211e_system_t previous;
  if (!reg.read(&previous.value) || previous.value == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  // Table A.6 mixes commands and persistent fields. Build a fresh command
  // so a read/modify/write cannot replay reset, reseed or ATI requests.
  iqs7211e_system_t command = {};
  command.bits.mode = previous.bits.mode;
  command.bits.txTest = previous.bits.txTest;
  if (sensor == IQS7211E_ALP) {
    command.bits.alpReseed = true;
  } else {
    command.bits.trackpadReseed = true;
  }
  return finishSetting(reg.write(command.value), true);
}

/*!
 * @brief Suspend the controller.
 * @return True on success; see getLastError() on failure.
 * @note End grouped settings first. Wake with an I2C address; begin() restores
 * defaults.
 */
bool Adafruit_IQS7211E::suspend() {
  if (_configuring) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_SYSTEM, 2, LSBFIRST);
  iqs7211e_system_t previous;
  if (!reg.read(&previous.value) || previous.value == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  // Table A.6 mixes commands and persistent fields. Build a fresh command
  // so a read/modify/write cannot replay reset, reseed or ATI requests.
  iqs7211e_system_t command = {};
  command.bits.mode = previous.bits.mode;
  command.bits.txTest = previous.bits.txTest;
  command.bits.suspend = true;
  return finishSetting(reg.write(command.value), true);
}

/*!
 * @brief Enable a reporting event.
 * @param event Selection.
 * @param enabled True to enable.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::enableEvent(iqs7211e_event_t event, bool enabled) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_CONFIG, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits enable(&reg, 1, event);
  bool written = enable.write(enabled);
  return finishSetting(written, true);
}

/*!
 * @brief Read whether a reporting event is enabled.
 * @param event Selection.
 * @return Enable state; false on failure, check getLastError().
 */
bool Adafruit_IQS7211E::getEventEnabled(iqs7211e_event_t event) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_CONFIG, 2, LSBFIRST);
  iqs7211e_bits_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return false;
  }
  switch (event) {
    case IQS7211E_EVENT_GESTURE:
      return value.bits.b9;
    case IQS7211E_EVENT_TRACKPAD:
      return value.bits.b10;
    case IQS7211E_EVENT_ATI:
      return value.bits.b11;
    case IQS7211E_EVENT_ALP:
      return value.bits.b13;
    case IQS7211E_EVENT_TOUCH:
      return value.bits.b14;
    default:
      return argumentError();
  }
}

/*!
 * @brief Enable a gesture.
 * @param gesture Selection.
 * @param enabled True to enable.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::enableGesture(iqs7211e_gesture_t gesture,
                                      bool enabled) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_GESTURE_ENABLE, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits enable(&reg, 1, gesture);
  bool written = enable.write(enabled);
  return finishSetting(written, true);
}

/*!
 * @brief Read whether a gesture is enabled.
 * @param gesture Selection.
 * @return Enable state; false on failure, check getLastError().
 */
bool Adafruit_IQS7211E::getGestureEnabled(iqs7211e_gesture_t gesture) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_GESTURE_ENABLE, 2, LSBFIRST);
  iqs7211e_bits_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return false;
  }
  switch (gesture) {
    case IQS7211E_GESTURE_SINGLE_TAP:
      return value.bits.b0;
    case IQS7211E_GESTURE_DOUBLE_TAP:
      return value.bits.b1;
    case IQS7211E_GESTURE_TRIPLE_TAP:
      return value.bits.b2;
    case IQS7211E_GESTURE_HOLD:
      return value.bits.b3;
    case IQS7211E_GESTURE_PALM:
      return value.bits.b4;
    case IQS7211E_GESTURE_SWIPE_X_POS:
      return value.bits.b8;
    case IQS7211E_GESTURE_SWIPE_X_NEG:
      return value.bits.b9;
    case IQS7211E_GESTURE_SWIPE_Y_POS:
      return value.bits.b10;
    case IQS7211E_GESTURE_SWIPE_Y_NEG:
      return value.bits.b11;
    case IQS7211E_GESTURE_SWIPE_HOLD_X_POS:
      return value.bits.b12;
    case IQS7211E_GESTURE_SWIPE_HOLD_X_NEG:
      return value.bits.b13;
    case IQS7211E_GESTURE_SWIPE_HOLD_Y_POS:
      return value.bits.b14;
    case IQS7211E_GESTURE_SWIPE_HOLD_Y_NEG:
      return value.bits.b15;
    default:
      return argumentError();
  }
}

/*!
 * @brief Enable an ALP receiver.
 * @param pin Physical receiver pin, 0-7.
 * @param enabled True to enable.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::enableALPReceiver(uint8_t pin, bool enabled) {
  if (pin > 7) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_ALP_SETUP, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits enable(&reg, 1, pin);
  bool written = enable.write(enabled);
  return finishSetting(written, true);
}

/*!
 * @brief Read whether an ALP receiver is enabled.
 * @param pin Physical receiver pin, 0-7.
 * @return Enable state; false on failure, check getLastError().
 */
bool Adafruit_IQS7211E::getALPReceiverEnabled(uint8_t pin) {
  if (pin > 7) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_ALP_SETUP, 2, LSBFIRST);
  iqs7211e_bits_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return false;
  }
  switch (pin) {
    case 0:
      return value.bits.b0;
    case 1:
      return value.bits.b1;
    case 2:
      return value.bits.b2;
    case 3:
      return value.bits.b3;
    case 4:
      return value.bits.b4;
    case 5:
      return value.bits.b5;
    case 6:
      return value.bits.b6;
    case 7:
      return value.bits.b7;
    default:
      return argumentError();
  }
}

/*!
 * @brief Enable an ALP transmitter.
 * @param pin Physical transmitter pin, 0-12.
 * @param enabled True to enable.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::enableALPTransmitter(uint8_t pin, bool enabled) {
  if (pin > 12) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_ALP_TRANSMITTERS, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits enable(&reg, 1, pin);
  bool written = enable.write(enabled);
  return finishSetting(written, true);
}

/*!
 * @brief Read whether an ALP transmitter is enabled.
 * @param pin Physical transmitter pin, 0-12.
 * @return Enable state; false on failure, check getLastError().
 */
bool Adafruit_IQS7211E::getALPTransmitterEnabled(uint8_t pin) {
  if (pin > 12) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_ALP_TRANSMITTERS, 2, LSBFIRST);
  iqs7211e_bits_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return false;
  }
  switch (pin) {
    case 0:
      return value.bits.b0;
    case 1:
      return value.bits.b1;
    case 2:
      return value.bits.b2;
    case 3:
      return value.bits.b3;
    case 4:
      return value.bits.b4;
    case 5:
      return value.bits.b5;
    case 6:
      return value.bits.b6;
    case 7:
      return value.bits.b7;
    case 8:
      return value.bits.b8;
    case 9:
      return value.bits.b9;
    case 10:
      return value.bits.b10;
    case 11:
      return value.bits.b11;
    case 12:
      return value.bits.b12;
    default:
      return argumentError();
  }
}

/** @brief Reject an invalid setting. @return Always false. */
bool Adafruit_IQS7211E::argumentError() {
  _error = IQS7211E_ERROR_ARGUMENT;
  if (_configuring) {
    _configurationFailed = true;
  }
  return false;
}

/** @brief Enter an available window without waiting in software.
 * @param requireInitialized False for identification and reset operations.
 * @return True if a setting transaction can start. */
bool Adafruit_IQS7211E::prepareCommunication(bool requireInitialized) {
  if (!_i2c || (requireInitialized && !_initialized && !_configuring)) {
    _error = IQS7211E_ERROR_NOT_INITIALIZED;
    return false;
  }
  if (_communicationRequest && !_configuring && digitalRead(_readyPin) != LOW) {
    if (!requestCommunication()) {
      return false;
    }
    _error = IQS7211E_ERROR_NOT_READY;
    return false; // Retry after RDY goes low; the request is asynchronous.
  }
  return true;
}

/** @brief Finish a checked configuration transaction.
 * @param success Whether the register access succeeded.
 * @param changed True for a write, which invalidates cached touch coordinates.
 * @return True if both the access and window termination succeeded. */
bool Adafruit_IQS7211E::finishSetting(bool success, bool changed) {
  if (changed) {
    _validData = false;
  }
  if (_configuring) {
    if (!success) {
      _configurationFailed = true;
    }
  } else {
    bool ended = endCommunication();
    success = success && ended;
  }
  if (!success) {
    _error = IQS7211E_ERROR_I2C;
    return false;
  }
  if (!_configuring || !_configurationFailed) {
    _error = IQS7211E_ERROR_NONE;
  }
  return true;
}

/*!
 * @brief Group settings in one communication window.
 * @return True if the window is open; false if already configuring or not
 * ready.
 * @note Call endConfiguration() even if a setter fails. This enables explicit
 * command termination and leaves it enabled. Do not delay between settings:
 * an idle window expires at the configured communication timeout.
 */
bool Adafruit_IQS7211E::beginConfiguration() {
  if (_configuring) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_CONFIG, 2, LSBFIRST);
  uint16_t value;
  if (!reg.read(&value) || value == IQS7211E_INVALID_WORD) {
    return finishSetting(false);
  }
  Adafruit_BusIO_RegisterBits termination(&reg, 1, 6);
  if (!termination.write(true)) {
    return finishSetting(false);
  }
  _commandTermination = true;
  _configuring = true;
  _configurationFailed = false;
  _validData = false;
  _error = IQS7211E_ERROR_NONE;
  return true;
}

/*!
 * @brief Release a grouped configuration so the controller can scan again.
 * @return False if any grouped setter failed or the end command failed.
 * @note This is not a rollback: earlier successful writes remain applied.
 */
bool Adafruit_IQS7211E::endConfiguration() {
  if (!_configuring) {
    return argumentError();
  }
  bool success = !_configurationFailed;
  iqs7211e_error_t error = _error;
  _configuring = false;
  bool ended = endCommunication();
  if (!ended) {
    _error = IQS7211E_ERROR_I2C;
  } else if (!success) {
    _error = error;
    if (_error == IQS7211E_ERROR_NONE) {
      _error = IQS7211E_ERROR_I2C;
    }
  } else {
    _error = IQS7211E_ERROR_NONE;
  }
  return success && ended;
}

/*!
 * @brief Request the next window when explicit communication requests are on.
 * @return True when already ready or the request was delivered; not a ready
 * flag.
 * @note Requires a wired RDY input. Wait for RDY low before retrying an
 * operation that returned IQS7211E_ERROR_NOT_READY. See section 11.1.3.
 */
bool Adafruit_IQS7211E::requestCommunication() {
  if (!_i2c) {
    _error = IQS7211E_ERROR_NOT_INITIALIZED;
    return false;
  }
  if (!_communicationRequest || _readyPin < 0) {
    return argumentError();
  }
  if (digitalRead(_readyPin) == LOW) {
    _error = IQS7211E_ERROR_NONE;
    return true;
  }
  Adafruit_BusIO_Register request(_i2c, IQS7211E_REG_END_COMMS);
  bool written = request.write((uint32_t)0, 1);
  _error = IQS7211E_ERROR_I2C;
  if (written) {
    _error = IQS7211E_ERROR_NONE;
  }
  return written;
}

/*!
 * @brief Select explicit end commands or STOP-terminated windows.
 * @param enabled True for explicit commands (the default).
 * @return True on success.
 * @note Disabling is rejected inside a grouped update or in request mode.
 * STOP mode cannot provide a coherent multi-transaction channel-bank read.
 */
bool Adafruit_IQS7211E::enableCommandTermination(bool enabled) {
  if (!enabled && (_configuring || _communicationRequest)) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_CONFIG, 2, LSBFIRST);
  uint16_t value;
  if (!reg.read(&value) || value == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits termination(&reg, 1, 6);
  bool written = termination.write(enabled);
  if (written) {
    _commandTermination = enabled;
  }
  return finishSetting(written, true);
}

/*!
 * @brief Select explicit requests instead of I2C clock stretching.
 * @param enabled True to request windows using RDY and the end-command address.
 * @return True on success; enabling requires RDY and command termination.
 * @note LP1/LP2 Auto-Prox on firmware 1.1 can stretch SCL for long periods.
 * Use this mode with RDY for those configurations (datasheet section 11.1.2).
 */
bool Adafruit_IQS7211E::enableCommunicationRequest(bool enabled) {
  if (enabled && (_readyPin < 0 || !_commandTermination)) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_CONFIG, 2, LSBFIRST);
  uint16_t value;
  if (!reg.read(&value) || value == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits request(&reg, 1, 4);
  bool written = request.write(enabled);
  if (written) {
    _communicationRequest = enabled;
  }
  return finishSetting(written, true);
}

/*!
 * @brief Read command termination from the controller.
 * @return Setting, or 0 on error; check getLastError().
 */
bool Adafruit_IQS7211E::getCommandTerminationEnabled() {
  if (!prepareCommunication()) {
    return (bool)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_CONFIG, 2, LSBFIRST);
  iqs7211e_config_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (bool)0;
  }
  return (bool)value.bits.commandTermination;
}

/*!
 * @brief Read explicit request mode from the controller.
 * @return Setting, or 0 on error; check getLastError().
 */
bool Adafruit_IQS7211E::getCommunicationRequestEnabled() {
  if (!prepareCommunication()) {
    return (bool)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_CONFIG, 2, LSBFIRST);
  iqs7211e_config_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (bool)0;
  }
  return (bool)value.bits.communicationRequest;
}

/*!
 * @brief Read selected manual power mode from the controller.
 * @return Setting, or 0 on error; check getLastError().
 */
iqs7211e_mode_t Adafruit_IQS7211E::getPowerMode() {
  if (!prepareCommunication()) {
    return (iqs7211e_mode_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_SYSTEM, 2, LSBFIRST);
  iqs7211e_system_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (iqs7211e_mode_t)0;
  }
  return (iqs7211e_mode_t)value.bits.mode;
}

/*!
 * @brief Read TX test mode from the controller.
 * @return Setting, or 0 on error; check getLastError().
 */
bool Adafruit_IQS7211E::getTXTestEnabled() {
  if (!prepareCommunication()) {
    return (bool)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_SYSTEM, 2, LSBFIRST);
  iqs7211e_system_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (bool)0;
  }
  return (bool)value.bits.txTest;
}

/*!
 * @brief Read trackpad flipX from the controller.
 * @return Setting, or 0 on error; check getLastError().
 */
bool Adafruit_IQS7211E::getFlipX() {
  if (!prepareCommunication()) {
    return (bool)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_TRACKPAD, 2, LSBFIRST);
  iqs7211e_trackpad_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (bool)0;
  }
  return (bool)value.bits.flipX;
}

/*!
 * @brief Read trackpad flipY from the controller.
 * @return Setting, or 0 on error; check getLastError().
 */
bool Adafruit_IQS7211E::getFlipY() {
  if (!prepareCommunication()) {
    return (bool)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_TRACKPAD, 2, LSBFIRST);
  iqs7211e_trackpad_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (bool)0;
  }
  return (bool)value.bits.flipY;
}

/*!
 * @brief Read trackpad swapXY from the controller.
 * @return Setting, or 0 on error; check getLastError().
 */
bool Adafruit_IQS7211E::getSwapXY() {
  if (!prepareCommunication()) {
    return (bool)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_TRACKPAD, 2, LSBFIRST);
  iqs7211e_trackpad_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (bool)0;
  }
  return (bool)value.bits.swapXY;
}

/*!
 * @brief Read trackpad receivers from the controller.
 * @return Setting, or 0 on error; check getLastError().
 */
uint8_t Adafruit_IQS7211E::getReceiverCount() {
  if (!prepareCommunication()) {
    return (uint8_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_TRACKPAD, 2, LSBFIRST);
  iqs7211e_trackpad_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)0;
  }
  return (uint8_t)value.bits.receivers;
}

/*!
 * @brief Read TransmitterCount from the controller.
 * @return Setting, or 0 on error; check getLastError().
 */
uint8_t Adafruit_IQS7211E::getTransmitterCount() {
  if (!prepareCommunication()) {
    return (uint8_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_TX_COUNT, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)0;
  }
  return (uint8_t)value.bytes.low;
}

/*!
 * @brief Read DynamicFilterBottomSpeed from the controller.
 * @return Setting, or 0 on error; check getLastError().
 */
uint16_t Adafruit_IQS7211E::getDynamicFilterBottomSpeed() {
  if (!prepareCommunication()) {
    return (uint16_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_FILTER_BOTTOM_SPEED, 2,
                              LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint16_t)0;
  }
  return (uint16_t)value.value;
}

/*!
 * @brief Read DynamicFilterTopSpeed from the controller.
 * @return Setting, or 0 on error; check getLastError().
 */
uint16_t Adafruit_IQS7211E::getDynamicFilterTopSpeed() {
  if (!prepareCommunication()) {
    return (uint16_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_FILTER_TOP_SPEED, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint16_t)0;
  }
  return (uint16_t)value.value;
}

/*!
 * @brief Read SettingsMajorVersion from the controller.
 * @return Setting, or 0 on error; check getLastError().
 */
uint8_t Adafruit_IQS7211E::getSettingsMajorVersion() {
  if (!prepareCommunication()) {
    return (uint8_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_SETTINGS_VERSION, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)0;
  }
  return (uint8_t)value.bytes.high;
}

/*!
 * @brief Read SettingsMinorVersion from the controller.
 * @return Setting, or 0 on error; check getLastError().
 */
uint8_t Adafruit_IQS7211E::getSettingsMinorVersion() {
  if (!prepareCommunication()) {
    return (uint8_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_SETTINGS_VERSION, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)0;
  }
  return (uint8_t)value.bytes.low;
}

/*!
 * @brief Read ALPSetDebounce from the controller.
 * @return Setting, or 0 on error; check getLastError().
 */
uint8_t Adafruit_IQS7211E::getALPSetDebounce() {
  if (!prepareCommunication()) {
    return (uint8_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_ALP_DEBOUNCE, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)0;
  }
  return (uint8_t)value.bytes.low;
}

/*!
 * @brief Read ALPClearDebounce from the controller.
 * @return Setting, or 0 on error; check getLastError().
 */
uint8_t Adafruit_IQS7211E::getALPClearDebounce() {
  if (!prepareCommunication()) {
    return (uint8_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_ALP_DEBOUNCE, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)0;
  }
  return (uint8_t)value.bytes.high;
}

/*!
 * @brief Read XResolution from the controller.
 * @return Setting, or 0 on error; check getLastError().
 */
uint16_t Adafruit_IQS7211E::getXResolution() {
  if (!prepareCommunication()) {
    return (uint16_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_X_RESOLUTION, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint16_t)0;
  }
  return (uint16_t)value.value;
}

/*!
 * @brief Read YResolution from the controller.
 * @return Setting, or 0 on error; check getLastError().
 */
uint16_t Adafruit_IQS7211E::getYResolution() {
  if (!prepareCommunication()) {
    return (uint16_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_Y_RESOLUTION, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint16_t)0;
  }
  return (uint16_t)value.value;
}

/*!
 * @brief Read TouchSetThreshold from the controller.
 * @return Setting, or 255 on error; check getLastError().
 */
uint8_t Adafruit_IQS7211E::getTouchSetThreshold() {
  if (!prepareCommunication()) {
    return (uint8_t)255;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_THRESHOLDS, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)255;
  }
  return (uint8_t)value.bytes.low;
}

/*!
 * @brief Read TouchClearThreshold from the controller.
 * @return Setting, or 255 on error; check getLastError().
 */
uint8_t Adafruit_IQS7211E::getTouchClearThreshold() {
  if (!prepareCommunication()) {
    return (uint8_t)255;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_THRESHOLDS, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)255;
  }
  return (uint8_t)value.bytes.high;
}

/*!
 * @brief Read requested report interval in milliseconds from the controller.
 * @param mode Charging mode.
 * @return Setting, or 0 on error; check getLastError().
 */
uint16_t Adafruit_IQS7211E::getReportRate(iqs7211e_mode_t mode) {
  if (!prepareCommunication()) {
    return (uint16_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_REPORT_RATE + mode, 2,
                              LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint16_t)0;
  }
  return (uint16_t)value.value;
}

/*!
 * @brief Read mode timeout in seconds (zero disables transitions) from the
 * controller.
 * @param mode Active, idle-touch, idle or LP1; LP2 has no timeout.
 * @return Setting, or 0 on error; check getLastError().
 */
uint16_t Adafruit_IQS7211E::getModeTimeout(iqs7211e_mode_t mode) {
  if (mode == IQS7211E_MODE_LP2) {
    argumentError();
    return (uint16_t)0;
  }
  if (!prepareCommunication()) {
    return (uint16_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_MODE_TIMEOUT + mode, 2,
                              LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint16_t)0;
  }
  return (uint16_t)value.value;
}

/*!
 * @brief Read automatic ATI enable from the controller.
 * @param sensor Trackpad or ALP.
 * @return Setting, or 0 on error; check getLastError().
 */
bool Adafruit_IQS7211E::getAutoATIEnabled(iqs7211e_sensor_t sensor) {
  if (!prepareCommunication()) {
    return (bool)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_CONFIG, 2, LSBFIRST);
  iqs7211e_config_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (bool)0;
  }
  if (sensor == IQS7211E_ALP) {
    return value.bits.alpAutoATI;
  }
  return value.bits.trackpadAutoATI;
}

/*!
 * @brief Read ALP Auto-Prox cycle selection from the controller.
 * @param mode LP1 or LP2.
 * @return Setting, or 0 on error; check getLastError().
 */
iqs7211e_autoprox_t Adafruit_IQS7211E::getAutoProxCycles(
    iqs7211e_low_power_t mode) {
  if (!prepareCommunication()) {
    return (iqs7211e_autoprox_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_ALP_HARDWARE, 2, LSBFIRST);
  iqs7211e_hardware_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (iqs7211e_autoprox_t)0;
  }
  if (mode == IQS7211E_LP2) {
    return (iqs7211e_autoprox_t)value.bits.lp2AutoProx;
  }
  return (iqs7211e_autoprox_t)value.bits.lp1AutoProx;
}

/*!
 * @brief Read edge trimming in coordinate units from the controller.
 * @param axis X or Y.
 * @return Setting, or 0 on error; check getLastError().
 */
uint8_t Adafruit_IQS7211E::getEdgeTrim(iqs7211e_axis_t axis) {
  if (!prepareCommunication()) {
    return (uint8_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_EDGE_TRIM, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)0;
  }
  if (axis == IQS7211E_AXIS_Y) {
    return value.bytes.high;
  }
  return value.bytes.low;
}

// Getters use checked word snapshots because 0xEEEE is an invalid-window
// response, not a valid field value. RegisterBits alone would hide that error.
// Field setters check the word before asking RegisterBits to preserve its
// other fields. These reads stay in the same command-terminated window.

/*!
 * @brief Set ATI Coarse Divider.
 * @param value Divider/multiplier, 1-31.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return True on success; see getLastError() on failure.
 * @note Recalibrate after changing calibration or analog settings.
 */
bool Adafruit_IQS7211E::setATICoarseDivider(uint8_t value,
                                            iqs7211e_sensor_t sensor) {
  if (value < 1 || value > 31) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  uint8_t address = IQS7211E_REG_ATI;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_ATI;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 5, 0);
  return finishSetting(setting.write(value), true);
}

/*!
 * @brief Read ATI Coarse Divider from the controller.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return Divider/multiplier, 1-31. Returns 0 on error; check getLastError().
 */
uint8_t Adafruit_IQS7211E::getATICoarseDivider(iqs7211e_sensor_t sensor) {
  if (!prepareCommunication()) {
    return (uint8_t)0;
  }
  uint8_t address = IQS7211E_REG_ATI;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_ATI;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  iqs7211e_gain_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)0;
  }
  return (uint8_t)value.bits.coarseDivider;
}

/*!
 * @brief Set ATI Coarse Multiplier.
 * @param value Divider/multiplier, 1-15.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return True on success; see getLastError() on failure.
 * @note Recalibrate after changing calibration or analog settings.
 */
bool Adafruit_IQS7211E::setATICoarseMultiplier(uint8_t value,
                                               iqs7211e_sensor_t sensor) {
  if (value < 1 || value > 15) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  uint8_t address = IQS7211E_REG_ATI;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_ATI;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 4, 5);
  return finishSetting(setting.write(value), true);
}

/*!
 * @brief Read ATI Coarse Multiplier from the controller.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return Divider/multiplier, 1-15. Returns 0 on error; check getLastError().
 */
uint8_t Adafruit_IQS7211E::getATICoarseMultiplier(iqs7211e_sensor_t sensor) {
  if (!prepareCommunication()) {
    return (uint8_t)0;
  }
  uint8_t address = IQS7211E_REG_ATI;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_ATI;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  iqs7211e_gain_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)0;
  }
  return (uint8_t)value.bits.multiplier;
}

/*!
 * @brief Set ATI Fine Divider.
 * @param value Divider/multiplier, 1-31.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return True on success; see getLastError() on failure.
 * @note Recalibrate after changing calibration or analog settings.
 */
bool Adafruit_IQS7211E::setATIFineDivider(uint8_t value,
                                          iqs7211e_sensor_t sensor) {
  if (value < 1 || value > 31) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  uint8_t address = IQS7211E_REG_ATI;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_ATI;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 5, 9);
  return finishSetting(setting.write(value), true);
}

/*!
 * @brief Read ATI Fine Divider from the controller.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return Divider/multiplier, 1-31. Returns 0 on error; check getLastError().
 */
uint8_t Adafruit_IQS7211E::getATIFineDivider(iqs7211e_sensor_t sensor) {
  if (!prepareCommunication()) {
    return (uint8_t)0;
  }
  uint8_t address = IQS7211E_REG_ATI;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_ATI;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  iqs7211e_gain_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)0;
  }
  return (uint8_t)value.bits.fineDivider;
}

/*!
 * @brief Set ATI compensation divider.
 * @param value Compensation divider, 1-255.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setATICompensationDivider(uint8_t value,
                                                  iqs7211e_sensor_t sensor) {
  if (value == 0) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  uint8_t address = IQS7211E_REG_ATI_COMPENSATION;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_ATI_COMPENSATION;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 8, 0);
  return finishSetting(setting.write(value), true);
}

/*!
 * @brief Read ATI compensation divider from the controller.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return Compensation divider, 1-255. Returns 0 on error; check
 * getLastError().
 */
uint8_t Adafruit_IQS7211E::getATICompensationDivider(iqs7211e_sensor_t sensor) {
  if (!prepareCommunication()) {
    return (uint8_t)0;
  }
  uint8_t address = IQS7211E_REG_ATI_COMPENSATION;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_ATI_COMPENSATION;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)0;
  }
  return (uint8_t)value.bytes.low;
}

/*!
 * @brief Set ATI drift limit.
 * @param value Allowed ATI drift in counts.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setATIDriftLimit(uint8_t value,
                                         iqs7211e_sensor_t sensor) {
  if (!prepareCommunication()) {
    return false;
  }
  uint8_t address = IQS7211E_REG_ATI_COMPENSATION;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_ATI_COMPENSATION;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 8, 8);
  return finishSetting(setting.write(value), true);
}

/*!
 * @brief Read ATI drift limit from the controller.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return Allowed ATI drift in counts. Returns 0 on error; check
 * getLastError().
 */
uint8_t Adafruit_IQS7211E::getATIDriftLimit(iqs7211e_sensor_t sensor) {
  if (!prepareCommunication()) {
    return (uint8_t)0;
  }
  uint8_t address = IQS7211E_REG_ATI_COMPENSATION;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_ATI_COMPENSATION;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)0;
  }
  return (uint8_t)value.bytes.high;
}

/*!
 * @brief Set ATI target.
 * @param counts Nonzero target count.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setATITarget(uint16_t counts,
                                     iqs7211e_sensor_t sensor) {
  if (counts == 0) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  uint8_t address = IQS7211E_REG_ATI_TARGET;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_ATI_TARGET;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  return finishSetting(reg.write(counts), true);
}

/*!
 * @brief Read ATI target from the controller.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return Nonzero target count. Returns 0 on error; check getLastError().
 */
uint16_t Adafruit_IQS7211E::getATITarget(iqs7211e_sensor_t sensor) {
  if (!prepareCommunication()) {
    return (uint16_t)0;
  }
  uint8_t address = IQS7211E_REG_ATI_TARGET;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_ATI_TARGET;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint16_t)0;
  }
  return (uint16_t)value.value;
}

/*!
 * @brief Set negative count change that triggers re-ATI.
 * @param counts Count delta.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setMinimumCountReATI(uint16_t counts) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_MINIMUM_COUNT, 2, LSBFIRST);
  return finishSetting(reg.write(counts), true);
}

/*!
 * @brief Read negative count change that triggers re-ATI from the controller.
 * @return Count delta. Returns 0 on error; check getLastError().
 */
uint16_t Adafruit_IQS7211E::getMinimumCountReATI() {
  if (!prepareCommunication()) {
    return (uint16_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_MINIMUM_COUNT, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint16_t)0;
  }
  return (uint16_t)value.value;
}

/*!
 * @brief Set ALP compensation.
 * @param value Compensation value, 0-1023.
 * @param engine Proximity engine A or B.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setALPCompensation(iqs7211e_engine_t engine,
                                           uint16_t value) {
  if (value > 1023) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_ALP_COMPENSATION + engine, 2,
                              LSBFIRST);
  return finishSetting(reg.write(value), true);
}

/*!
 * @brief Read ALP compensation from the controller.
 * @param engine Proximity engine A or B.
 * @return Compensation value, 0-1023. Returns 0 on error; check getLastError().
 */
uint16_t Adafruit_IQS7211E::getALPCompensation(iqs7211e_engine_t engine) {
  if (!prepareCommunication()) {
    return (uint16_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_ALP_COMPENSATION + engine, 2,
                              LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint16_t)0;
  }
  return (uint16_t)value.value;
}

/*!
 * @brief Set reference update interval.
 * @param seconds Seconds.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setReferenceUpdateTime(uint8_t seconds) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_REFERENCE_TIMING, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 8, 8);
  return finishSetting(setting.write(seconds), true);
}

/*!
 * @brief Read reference update interval from the controller.
 * @return Seconds. Returns 0 on error; check getLastError().
 */
uint8_t Adafruit_IQS7211E::getReferenceUpdateTime() {
  if (!prepareCommunication()) {
    return (uint8_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_REFERENCE_TIMING, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)0;
  }
  return (uint8_t)value.bytes.high;
}

/*!
 * @brief Set failed ATI retry interval.
 * @param seconds Seconds, 0-60.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setATIRetryTime(uint8_t seconds) {
  if (seconds > 60) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_REFERENCE_TIMING, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 8, 0);
  return finishSetting(setting.write(seconds), true);
}

/*!
 * @brief Read failed ATI retry interval from the controller.
 * @return Seconds, 0-60. Returns 0 on error; check getLastError().
 */
uint8_t Adafruit_IQS7211E::getATIRetryTime() {
  if (!prepareCommunication()) {
    return (uint8_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_REFERENCE_TIMING, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)0;
  }
  return (uint8_t)value.bytes.low;
}

/*!
 * @brief Set communication-window timeout.
 * @param milliseconds Nonzero milliseconds.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setCommunicationTimeout(uint16_t milliseconds) {
  if (milliseconds == 0) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_COMMUNICATION_TIMEOUT, 2,
                              LSBFIRST);
  return finishSetting(reg.write(milliseconds), true);
}

/*!
 * @brief Read communication-window timeout from the controller.
 * @return Nonzero milliseconds. Returns 0 on error; check getLastError().
 */
uint16_t Adafruit_IQS7211E::getCommunicationTimeout() {
  if (!prepareCommunication()) {
    return (uint16_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_COMMUNICATION_TIMEOUT, 2,
                              LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint16_t)0;
  }
  return (uint16_t)value.value;
}

/*!
 * @brief Set manual charging-mode control.
 * @param enabled True to enable.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::enableManualMode(bool enabled) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_CONFIG, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 1, 7);
  return finishSetting(setting.write(enabled), true);
}

/*!
 * @brief Read manual charging-mode control from the controller.
 * @return True to enable. Returns 0 on error; check getLastError().
 */
bool Adafruit_IQS7211E::getManualModeEnabled() {
  if (!prepareCommunication()) {
    return (bool)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_CONFIG, 2, LSBFIRST);
  iqs7211e_config_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (bool)0;
  }
  return (bool)value.bits.manualMode;
}

/*!
 * @brief Set event-only reporting.
 * @param enabled True to enable.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::enableEventMode(bool enabled) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_CONFIG, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 1, 8);
  return finishSetting(setting.write(enabled), true);
}

/*!
 * @brief Read event-only reporting from the controller.
 * @return True to enable. Returns 0 on error; check getLastError().
 */
bool Adafruit_IQS7211E::getEventModeEnabled() {
  if (!prepareCommunication()) {
    return (bool)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_CONFIG, 2, LSBFIRST);
  iqs7211e_config_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (bool)0;
  }
  return (bool)value.bits.eventMode;
}

/*!
 * @brief Set the watchdog.
 * @param enabled True to enable.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::enableWatchdog(bool enabled) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_CONFIG, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 1, 5);
  return finishSetting(setting.write(enabled), true);
}

/*!
 * @brief Read the watchdog from the controller.
 * @return True to enable. Returns 0 on error; check getLastError().
 */
bool Adafruit_IQS7211E::getWatchdogEnabled() {
  if (!prepareCommunication()) {
    return (bool)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_CONFIG, 2, LSBFIRST);
  iqs7211e_config_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (bool)0;
  }
  return (bool)value.bits.watchdog;
}

/*!
 * @brief Set oscillator selection.
 * @param oscillator 14 MHz or 18 MHz selection.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setOscillator(iqs7211e_oscillator_t oscillator) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_OSCILLATOR, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 1, 4);
  return finishSetting(setting.write(oscillator), true);
}

/*!
 * @brief Read oscillator selection from the controller.
 * @return 14 MHz or 18 MHz selection. Returns 0 on error; check getLastError().
 */
iqs7211e_oscillator_t Adafruit_IQS7211E::getOscillator() {
  if (!prepareCommunication()) {
    return (iqs7211e_oscillator_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_OSCILLATOR, 2, LSBFIRST);
  iqs7211e_oscillator_settings_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (iqs7211e_oscillator_t)0;
  }
  return (iqs7211e_oscillator_t)value.bits.oscillator;
}

/*!
 * @brief Set oscillator adjustment.
 * @param adjustment Frequency adjustment encoding, 0-15 (Table A.8).
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setOscillatorAdjustment(uint8_t adjustment) {
  if (adjustment > 15) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_OSCILLATOR, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 4, 0);
  return finishSetting(setting.write(adjustment), true);
}

/*!
 * @brief Read oscillator adjustment from the controller.
 * @return Frequency adjustment encoding, 0-15 (Table A.8). Returns 0 on error;
 * check getLastError().
 */
uint8_t Adafruit_IQS7211E::getOscillatorAdjustment() {
  if (!prepareCommunication()) {
    return (uint8_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_OSCILLATOR, 2, LSBFIRST);
  iqs7211e_oscillator_settings_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)0;
  }
  return (uint8_t)value.bits.adjustment;
}

/*!
 * @brief Set ALP sensing method.
 * @param method Self- or mutual-capacitive selection.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setALPSensingMethod(iqs7211e_sensing_t method) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_ALP_SETUP, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 1, 8);
  return finishSetting(setting.write(method), true);
}

/*!
 * @brief Read ALP sensing method from the controller.
 * @return Self- or mutual-capacitive selection. Returns 0 on error; check
 * getLastError().
 */
iqs7211e_sensing_t Adafruit_IQS7211E::getALPSensingMethod() {
  if (!prepareCommunication()) {
    return (iqs7211e_sensing_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_ALP_SETUP, 2, LSBFIRST);
  iqs7211e_bits_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (iqs7211e_sensing_t)0;
  }
  return (iqs7211e_sensing_t)value.bits.b8;
}

/*!
 * @brief Set ALP count filtering.
 * @param enabled True to enable.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::enableALPCountFilter(bool enabled) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_ALP_SETUP, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 1, 9);
  return finishSetting(setting.write(enabled), true);
}

/*!
 * @brief Read ALP count filtering from the controller.
 * @return True to enable. Returns 0 on error; check getLastError().
 */
bool Adafruit_IQS7211E::getALPCountFilterEnabled() {
  if (!prepareCommunication()) {
    return (bool)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_ALP_SETUP, 2, LSBFIRST);
  iqs7211e_bits_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (bool)0;
  }
  return (bool)value.bits.b9;
}

/*!
 * @brief Set ALP threshold.
 * @param counts Raw counts.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setALPThreshold(uint16_t counts) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_ALP_THRESHOLD, 2, LSBFIRST);
  return finishSetting(reg.write(counts), true);
}

/*!
 * @brief Read ALP threshold from the controller.
 * @return Raw counts. Returns 0 on error; check getLastError().
 */
uint16_t Adafruit_IQS7211E::getALPThreshold() {
  if (!prepareCommunication()) {
    return (uint16_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_ALP_THRESHOLD, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint16_t)0;
  }
  return (uint16_t)value.value;
}

/*!
 * @brief Set ALP count filter beta.
 * @param beta Beta numerator, 0-255; filter weight is beta/256.
 * @param mode LP1 or LP2 settings.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setALPCountBeta(iqs7211e_low_power_t mode,
                                        uint8_t beta) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_ALP_FILTER + mode, 2,
                              LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 8, 0);
  return finishSetting(setting.write(beta), true);
}

/*!
 * @brief Read ALP count filter beta from the controller.
 * @param mode LP1 or LP2 settings.
 * @return Beta numerator, 0-255; filter weight is beta/256. Returns 0 on error;
 * check getLastError().
 */
uint8_t Adafruit_IQS7211E::getALPCountBeta(iqs7211e_low_power_t mode) {
  if (!prepareCommunication()) {
    return (uint8_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_ALP_FILTER + mode, 2,
                              LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)0;
  }
  return (uint8_t)value.bytes.low;
}

/*!
 * @brief Set ALP reference filter beta.
 * @param beta Beta numerator, 0-255; filter weight is beta/256.
 * @param mode LP1 or LP2 settings.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setALPReferenceBeta(iqs7211e_low_power_t mode,
                                            uint8_t beta) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_ALP_FILTER + mode, 2,
                              LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 8, 8);
  return finishSetting(setting.write(beta), true);
}

/*!
 * @brief Read ALP reference filter beta from the controller.
 * @param mode LP1 or LP2 settings.
 * @return Beta numerator, 0-255; filter weight is beta/256. Returns 0 on error;
 * check getLastError().
 */
uint8_t Adafruit_IQS7211E::getALPReferenceBeta(iqs7211e_low_power_t mode) {
  if (!prepareCommunication()) {
    return (uint8_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_ALP_FILTER + mode, 2,
                              LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)0;
  }
  return (uint8_t)value.bytes.high;
}

/*!
 * @brief Set conversion period.
 * @param period Frequency setting, 0-127 (section 5.2).
 * @param sensor Trackpad or alternate low-power sensor.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setConversionPeriod(uint8_t period,
                                            iqs7211e_sensor_t sensor) {
  if (period > 127) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  uint8_t address = IQS7211E_REG_CONVERSION;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_CONVERSION;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 8, 0);
  return finishSetting(setting.write(period), true);
}

/*!
 * @brief Read conversion period from the controller.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return Frequency setting, 0-127 (section 5.2). Returns 0 on error; check
 * getLastError().
 */
uint8_t Adafruit_IQS7211E::getConversionPeriod(iqs7211e_sensor_t sensor) {
  if (!prepareCommunication()) {
    return (uint8_t)0;
  }
  uint8_t address = IQS7211E_REG_CONVERSION;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_CONVERSION;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)0;
  }
  return (uint8_t)value.bytes.low;
}

/*!
 * @brief Set conversion fraction.
 * @param fraction Frequency setting, 0-127 (section 5.2).
 * @param sensor Trackpad or alternate low-power sensor.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setConversionFraction(uint8_t fraction,
                                              iqs7211e_sensor_t sensor) {
  if (fraction > 127) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  uint8_t address = IQS7211E_REG_CONVERSION;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_CONVERSION;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 8, 8);
  return finishSetting(setting.write(fraction), true);
}

/*!
 * @brief Read conversion fraction from the controller.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return Frequency setting, 0-127 (section 5.2). Returns 0 on error; check
 * getLastError().
 */
uint8_t Adafruit_IQS7211E::getConversionFraction(iqs7211e_sensor_t sensor) {
  if (!prepareCommunication()) {
    return (uint8_t)0;
  }
  uint8_t address = IQS7211E_REG_CONVERSION;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_CONVERSION;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)0;
  }
  return (uint8_t)value.bytes.high;
}

/*!
 * @brief Set NM static mode.
 * @param enabled True to enable.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::enableNMStatic(bool enabled, iqs7211e_sensor_t sensor) {
  if (!prepareCommunication()) {
    return false;
  }
  uint8_t address = IQS7211E_REG_HARDWARE;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_HARDWARE;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 1, 15);
  return finishSetting(setting.write(enabled), true);
}

/*!
 * @brief Read NM static mode from the controller.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return True to enable. Returns 0 on error; check getLastError().
 */
bool Adafruit_IQS7211E::getNMStaticEnabled(iqs7211e_sensor_t sensor) {
  if (!prepareCommunication()) {
    return (bool)0;
  }
  uint8_t address = IQS7211E_REG_HARDWARE;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_HARDWARE;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  iqs7211e_hardware_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (bool)0;
  }
  return (bool)value.bits.nmStatic;
}

/*!
 * @brief Set 0.5 V discharge.
 * @param enabled True to enable.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::enableHalfVoltDischarge(bool enabled,
                                                iqs7211e_sensor_t sensor) {
  if (!prepareCommunication()) {
    return false;
  }
  uint8_t address = IQS7211E_REG_HARDWARE;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_HARDWARE;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 1, 14);
  return finishSetting(setting.write(enabled), true);
}

/*!
 * @brief Read 0.5 V discharge from the controller.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return True to enable. Returns 0 on error; check getLastError().
 */
bool Adafruit_IQS7211E::getHalfVoltDischargeEnabled(iqs7211e_sensor_t sensor) {
  if (!prepareCommunication()) {
    return (bool)0;
  }
  uint8_t address = IQS7211E_REG_HARDWARE;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_HARDWARE;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  iqs7211e_hardware_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (bool)0;
  }
  return (bool)value.bits.halfVoltDischarge;
}

/*!
 * @brief Set the RF filter.
 * @param enabled True to enable.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::enableRFFilter(bool enabled, iqs7211e_sensor_t sensor) {
  if (!prepareCommunication()) {
    return false;
  }
  uint8_t address = IQS7211E_REG_HARDWARE;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_HARDWARE;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 1, 13);
  return finishSetting(setting.write(enabled), true);
}

/*!
 * @brief Read the RF filter from the controller.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return True to enable. Returns 0 on error; check getLastError().
 */
bool Adafruit_IQS7211E::getRFFilterEnabled(iqs7211e_sensor_t sensor) {
  if (!prepareCommunication()) {
    return (bool)0;
  }
  uint8_t address = IQS7211E_REG_HARDWARE;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_HARDWARE;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  iqs7211e_hardware_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (bool)0;
  }
  return (bool)value.bits.rfFilter;
}

/*!
 * @brief Set Charge Capacitor.
 * @param capacitor 40 pF or 80 pF.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setChargeCapacitor(iqs7211e_capacitor_t capacitor,
                                           iqs7211e_sensor_t sensor) {
  if (!prepareCommunication()) {
    return false;
  }
  uint8_t address = IQS7211E_REG_HARDWARE;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_HARDWARE;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 1, 12);
  return finishSetting(setting.write(capacitor), true);
}

/*!
 * @brief Read Charge Capacitor from the controller.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return 40 pF or 80 pF. Returns 0 on error; check getLastError().
 */
iqs7211e_capacitor_t Adafruit_IQS7211E::getChargeCapacitor(
    iqs7211e_sensor_t sensor) {
  if (!prepareCommunication()) {
    return (iqs7211e_capacitor_t)0;
  }
  uint8_t address = IQS7211E_REG_HARDWARE;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_HARDWARE;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  iqs7211e_hardware_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (iqs7211e_capacitor_t)0;
  }
  return (iqs7211e_capacitor_t)value.bits.capacitor;
}

/*!
 * @brief Set Opamp Bias.
 * @param bias Bias current selection.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setOpampBias(iqs7211e_bias_t bias,
                                     iqs7211e_sensor_t sensor) {
  if (!prepareCommunication()) {
    return false;
  }
  uint8_t address = IQS7211E_REG_HARDWARE;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_HARDWARE;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 2, 10);
  return finishSetting(setting.write(bias), true);
}

/*!
 * @brief Read Opamp Bias from the controller.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return Bias current selection. Returns 0 on error; check getLastError().
 */
iqs7211e_bias_t Adafruit_IQS7211E::getOpampBias(iqs7211e_sensor_t sensor) {
  if (!prepareCommunication()) {
    return (iqs7211e_bias_t)0;
  }
  uint8_t address = IQS7211E_REG_HARDWARE;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_HARDWARE;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  iqs7211e_hardware_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (iqs7211e_bias_t)0;
  }
  return (iqs7211e_bias_t)value.bits.opampBias;
}

/*!
 * @brief Set Count Limit.
 * @param limit Maximum conversion count selection.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setCountLimit(iqs7211e_count_limit_t limit,
                                      iqs7211e_sensor_t sensor) {
  if (!prepareCommunication()) {
    return false;
  }
  uint8_t address = IQS7211E_REG_HARDWARE;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_HARDWARE;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 2, 8);
  return finishSetting(setting.write(limit), true);
}

/*!
 * @brief Read Count Limit from the controller.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return Maximum conversion count selection. Returns 0 on error; check
 * getLastError().
 */
iqs7211e_count_limit_t Adafruit_IQS7211E::getCountLimit(
    iqs7211e_sensor_t sensor) {
  if (!prepareCommunication()) {
    return (iqs7211e_count_limit_t)0;
  }
  uint8_t address = IQS7211E_REG_HARDWARE;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_HARDWARE;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  iqs7211e_hardware_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (iqs7211e_count_limit_t)0;
  }
  return (iqs7211e_count_limit_t)value.bits.countLimit;
}

/*!
 * @brief Set Initial Delay.
 * @param cycles Initial conversion delay selection.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setInitialDelay(iqs7211e_init_delay_t cycles,
                                        iqs7211e_sensor_t sensor) {
  if (!prepareCommunication()) {
    return false;
  }
  uint8_t address = IQS7211E_REG_HARDWARE;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_HARDWARE;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 2, 0);
  return finishSetting(setting.write(cycles), true);
}

/*!
 * @brief Read Initial Delay from the controller.
 * @param sensor Trackpad or alternate low-power sensor.
 * @return Initial conversion delay selection. Returns 0 on error; check
 * getLastError().
 */
iqs7211e_init_delay_t Adafruit_IQS7211E::getInitialDelay(
    iqs7211e_sensor_t sensor) {
  if (!prepareCommunication()) {
    return (iqs7211e_init_delay_t)0;
  }
  uint8_t address = IQS7211E_REG_HARDWARE;
  if (sensor == IQS7211E_ALP) {
    address = IQS7211E_REG_ALP_HARDWARE;
  }
  Adafruit_BusIO_Register reg(_i2c, address, 2, LSBFIRST);
  iqs7211e_hardware_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (iqs7211e_init_delay_t)0;
  }
  return (iqs7211e_init_delay_t)value.bits.initialDelay;
}

/*!
 * @brief Set maximum simultaneous touches.
 * @param touches One or two fingers.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setMaxTouches(uint8_t touches) {
  if (touches < 1 || touches > IQS7211E_MAX_TOUCHES) {
    return argumentError();
  }
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_TX_COUNT, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 8, 8);
  return finishSetting(setting.write(touches), true);
}

/*!
 * @brief Read maximum simultaneous touches from the controller.
 * @return One or two fingers. Returns 0 on error; check getLastError().
 */
uint8_t Adafruit_IQS7211E::getMaxTouches() {
  if (!prepareCommunication()) {
    return (uint8_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_TX_COUNT, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)0;
  }
  return (uint8_t)value.bytes.high;
}

/*!
 * @brief Set the moving-average coordinate filter.
 * @param enabled True to enable.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::enableMAVFilter(bool enabled) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_TRACKPAD, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 1, 5);
  return finishSetting(setting.write(enabled), true);
}

/*!
 * @brief Read the moving-average coordinate filter from the controller.
 * @return True to enable. Returns 0 on error; check getLastError().
 */
bool Adafruit_IQS7211E::getMAVFilterEnabled() {
  if (!prepareCommunication()) {
    return (bool)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_TRACKPAD, 2, LSBFIRST);
  iqs7211e_trackpad_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (bool)0;
  }
  return (bool)value.bits.mavFilter;
}

/*!
 * @brief Set the IIR coordinate filter.
 * @param enabled True to enable.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::enableIIRFilter(bool enabled) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_TRACKPAD, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 1, 3);
  return finishSetting(setting.write(enabled), true);
}

/*!
 * @brief Read the IIR coordinate filter from the controller.
 * @return True to enable. Returns 0 on error; check getLastError().
 */
bool Adafruit_IQS7211E::getIIRFilterEnabled() {
  if (!prepareCommunication()) {
    return (bool)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_TRACKPAD, 2, LSBFIRST);
  iqs7211e_trackpad_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (bool)0;
  }
  return (bool)value.bits.iirFilter;
}

/*!
 * @brief Set static IIR damping instead of dynamic damping.
 * @param enabled True to enable.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::enableStaticIIR(bool enabled) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_TRACKPAD, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 1, 4);
  return finishSetting(setting.write(enabled), true);
}

/*!
 * @brief Read static IIR damping instead of dynamic damping from the
 * controller.
 * @return True to enable. Returns 0 on error; check getLastError().
 */
bool Adafruit_IQS7211E::getStaticIIREnabled() {
  if (!prepareCommunication()) {
    return (bool)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_TRACKPAD, 2, LSBFIRST);
  iqs7211e_trackpad_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (bool)0;
  }
  return (bool)value.bits.staticIIR;
}

/*!
 * @brief Set dynamic IIR bottom-speed beta.
 * @param beta Beta numerator, 0-255; weight is beta/256.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setDynamicFilterBeta(uint8_t beta) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_FILTER_BETA, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 8, 0);
  return finishSetting(setting.write(beta), true);
}

/*!
 * @brief Read dynamic IIR bottom-speed beta from the controller.
 * @return Beta numerator, 0-255; weight is beta/256. Returns 0 on error; check
 * getLastError().
 */
uint8_t Adafruit_IQS7211E::getDynamicFilterBeta() {
  if (!prepareCommunication()) {
    return (uint8_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_FILTER_BETA, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)0;
  }
  return (uint8_t)value.bytes.low;
}

/*!
 * @brief Set static IIR beta.
 * @param beta Beta numerator, 0-255; weight is beta/256.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setStaticFilterBeta(uint8_t beta) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_FILTER_BETA, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 8, 8);
  return finishSetting(setting.write(beta), true);
}

/*!
 * @brief Read static IIR beta from the controller.
 * @return Beta numerator, 0-255; weight is beta/256. Returns 0 on error; check
 * getLastError().
 */
uint8_t Adafruit_IQS7211E::getStaticFilterBeta() {
  if (!prepareCommunication()) {
    return (uint8_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_FILTER_BETA, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)0;
  }
  return (uint8_t)value.bytes.high;
}

/*!
 * @brief Set finger-splitting factor.
 * @param factor Split factor, 0-255; zero disables splitting.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setFingerSplitFactor(uint8_t factor) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_FINGER_SPLIT, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 8, 8);
  return finishSetting(setting.write(factor), true);
}

/*!
 * @brief Read finger-splitting factor from the controller.
 * @return Split factor, 0-255; zero disables splitting. Returns 0 on error;
 * check getLastError().
 */
uint8_t Adafruit_IQS7211E::getFingerSplitFactor() {
  if (!prepareCommunication()) {
    return (uint8_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_FINGER_SPLIT, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)0;
  }
  return (uint8_t)value.bytes.high;
}

/*!
 * @brief Set stationary-touch movement threshold.
 * @param distance Displacement in coordinate units.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setStationaryThreshold(uint8_t distance) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_FINGER_SPLIT, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 8, 0);
  return finishSetting(setting.write(distance), true);
}

/*!
 * @brief Read stationary-touch movement threshold from the controller.
 * @return Displacement in coordinate units. Returns 0 on error; check
 * getLastError().
 */
uint8_t Adafruit_IQS7211E::getStationaryThreshold() {
  if (!prepareCommunication()) {
    return (uint8_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_FINGER_SPLIT, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)0;
  }
  return (uint8_t)value.bytes.low;
}

/*!
 * @brief Set swipe angle limit.
 * @param tangent 64 times the tangent of the permitted angle (23 is about 20
 * degrees).
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setSwipeAngle(uint8_t tangent) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_GESTURE_SETTINGS, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 8, 0);
  return finishSetting(setting.write(tangent), true);
}

/*!
 * @brief Read swipe angle limit from the controller.
 * @return 64 times the tangent of the permitted angle (23 is about 20 degrees).
 * Returns 0 on error; check getLastError().
 */
uint8_t Adafruit_IQS7211E::getSwipeAngle() {
  if (!prepareCommunication()) {
    return (uint8_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_GESTURE_SETTINGS, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)0;
  }
  return (uint8_t)value.bytes.low;
}

/*!
 * @brief Set palm channel threshold.
 * @param channels Number of channels covered by a palm.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setPalmThreshold(uint8_t channels) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_GESTURE_SETTINGS, 2, LSBFIRST);
  uint16_t previous;
  if (!reg.read(&previous) || previous == IQS7211E_INVALID_WORD) {
    return finishSetting(false, true);
  }
  Adafruit_BusIO_RegisterBits setting(&reg, 8, 8);
  return finishSetting(setting.write(channels), true);
}

/*!
 * @brief Read palm channel threshold from the controller.
 * @return Number of channels covered by a palm. Returns 0 on error; check
 * getLastError().
 */
uint8_t Adafruit_IQS7211E::getPalmThreshold() {
  if (!prepareCommunication()) {
    return (uint8_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_GESTURE_SETTINGS, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint8_t)0;
  }
  return (uint8_t)value.bytes.high;
}

/*!
 * @brief Set Tap Time.
 * @param milliseconds Maximum tap duration in milliseconds.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setTapTime(uint16_t milliseconds) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_TAP_TIME, 2, LSBFIRST);
  return finishSetting(reg.write(milliseconds), true);
}

/*!
 * @brief Read Tap Time from the controller.
 * @return Maximum tap duration in milliseconds. Returns 0 on error; check
 * getLastError().
 */
uint16_t Adafruit_IQS7211E::getTapTime() {
  if (!prepareCommunication()) {
    return (uint16_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_TAP_TIME, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint16_t)0;
  }
  return (uint16_t)value.value;
}

/*!
 * @brief Set Air Time.
 * @param milliseconds Inter-tap air time in milliseconds.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setAirTime(uint16_t milliseconds) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_AIR_TIME, 2, LSBFIRST);
  return finishSetting(reg.write(milliseconds), true);
}

/*!
 * @brief Read Air Time from the controller.
 * @return Inter-tap air time in milliseconds. Returns 0 on error; check
 * getLastError().
 */
uint16_t Adafruit_IQS7211E::getAirTime() {
  if (!prepareCommunication()) {
    return (uint16_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_AIR_TIME, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint16_t)0;
  }
  return (uint16_t)value.value;
}

/*!
 * @brief Set Tap Distance.
 * @param distance Maximum tap displacement in coordinate units.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setTapDistance(uint16_t distance) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_TAP_DISTANCE, 2, LSBFIRST);
  return finishSetting(reg.write(distance), true);
}

/*!
 * @brief Read Tap Distance from the controller.
 * @return Maximum tap displacement in coordinate units. Returns 0 on error;
 * check getLastError().
 */
uint16_t Adafruit_IQS7211E::getTapDistance() {
  if (!prepareCommunication()) {
    return (uint16_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_TAP_DISTANCE, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint16_t)0;
  }
  return (uint16_t)value.value;
}

/*!
 * @brief Set Hold Time.
 * @param milliseconds Hold duration in milliseconds.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setHoldTime(uint16_t milliseconds) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_HOLD_TIME, 2, LSBFIRST);
  return finishSetting(reg.write(milliseconds), true);
}

/*!
 * @brief Read Hold Time from the controller.
 * @return Hold duration in milliseconds. Returns 0 on error; check
 * getLastError().
 */
uint16_t Adafruit_IQS7211E::getHoldTime() {
  if (!prepareCommunication()) {
    return (uint16_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_HOLD_TIME, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint16_t)0;
  }
  return (uint16_t)value.value;
}

/*!
 * @brief Set Swipe Time.
 * @param milliseconds Swipe duration in milliseconds.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setSwipeTime(uint16_t milliseconds) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_SWIPE_TIME, 2, LSBFIRST);
  return finishSetting(reg.write(milliseconds), true);
}

/*!
 * @brief Read Swipe Time from the controller.
 * @return Swipe duration in milliseconds. Returns 0 on error; check
 * getLastError().
 */
uint16_t Adafruit_IQS7211E::getSwipeTime() {
  if (!prepareCommunication()) {
    return (uint16_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_SWIPE_TIME, 2, LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint16_t)0;
  }
  return (uint16_t)value.value;
}

/*!
 * @brief Set initial swipe distance.
 * @param distance Coordinate units.
 * @param axis X or Y output axis.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setSwipeInitialDistance(iqs7211e_axis_t axis,
                                                uint16_t distance) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_SWIPE_INITIAL + axis, 2,
                              LSBFIRST);
  return finishSetting(reg.write(distance), true);
}

/*!
 * @brief Read initial swipe distance from the controller.
 * @param axis X or Y output axis.
 * @return Coordinate units. Returns 0 on error; check getLastError().
 */
uint16_t Adafruit_IQS7211E::getSwipeInitialDistance(iqs7211e_axis_t axis) {
  if (!prepareCommunication()) {
    return (uint16_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_SWIPE_INITIAL + axis, 2,
                              LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint16_t)0;
  }
  return (uint16_t)value.value;
}

/*!
 * @brief Set consecutive swipe distance.
 * @param distance Coordinate units.
 * @param axis X or Y output axis.
 * @return True on success; see getLastError() on failure.
 */
bool Adafruit_IQS7211E::setSwipeConsecutiveDistance(iqs7211e_axis_t axis,
                                                    uint16_t distance) {
  if (!prepareCommunication()) {
    return false;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_SWIPE_CONSECUTIVE + axis, 2,
                              LSBFIRST);
  return finishSetting(reg.write(distance), true);
}

/*!
 * @brief Read consecutive swipe distance from the controller.
 * @param axis X or Y output axis.
 * @return Coordinate units. Returns 0 on error; check getLastError().
 */
uint16_t Adafruit_IQS7211E::getSwipeConsecutiveDistance(iqs7211e_axis_t axis) {
  if (!prepareCommunication()) {
    return (uint16_t)0;
  }
  Adafruit_BusIO_Register reg(_i2c, IQS7211E_REG_SWIPE_CONSECUTIVE + axis, 2,
                              LSBFIRST);
  iqs7211e_word_t value;
  bool read = reg.read(&value.value);
  if (!finishSetting(read && value.value != IQS7211E_INVALID_WORD)) {
    return (uint16_t)0;
  }
  return (uint16_t)value.value;
}
