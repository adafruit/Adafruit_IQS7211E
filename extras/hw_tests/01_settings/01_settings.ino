#include <Adafruit_IQS7211E.h>

// Set READY_PIN to 2 to also test explicit requests with RDY wired to Metro D2.
// Keep the MCU pull-up disabled; RDY uses the board's 3.3 V pull-up.
const int16_t READY_PIN = -1;
Adafruit_IQS7211E trackpad;

void setup() {
  Serial.begin(115200);
  delay(250);
  Serial.print(F("IQS7211E settings hardware test\n\r"));
  if (!trackpad.begin(&Wire, READY_PIN)) {
    halt(F("begin"));
  }

  // Capture the writable map for comparison with the previous working profile.
  // ATI may adjust its gain registers, so compare those separately.
  Adafruit_I2CDevice debug(IQS7211E_I2C_ADDRESS);
  debug.begin(false);
  if (!trackpad.beginConfiguration()) {
    halt(F("snapshot window"));
  }
  uint16_t snapshot[94];
  for (uint8_t offset = 0; offset < 94; offset += 14) {
    uint8_t words = min((uint8_t)14, (uint8_t)(94 - offset));
    Adafruit_BusIO_Register reg(&debug, IQS7211E_REG_CONFIG_START + offset,
                                2, LSBFIRST);
    if (!reg.read((uint8_t *)(snapshot + offset), words * 2)) {
      trackpad.endConfiguration();
      halt(F("snapshot read"));
    }
  }
  if (!trackpad.endConfiguration()) {
    halt(F("snapshot close"));
  }
  Serial.print(F("REGISTERS"));
  for (uint8_t i = 0; i < 94; i++) {
    Serial.print(' ');
    Serial.print(snapshot[i], HEX);
  }
  Serial.print(F("\n\r"));

  if (trackpad.getReceiverCount() != 6 || trackpad.getTransmitterCount() != 7 ||
      trackpad.getMaxTouches() != 2 || trackpad.getXResolution() != 1000 ||
      trackpad.getYResolution() != 1000 || !trackpad.getSwapXY() ||
      trackpad.getFlipX() || trackpad.getFlipY() ||
      trackpad.getLastError() != IQS7211E_ERROR_NONE) {
    halt(F("layout defaults"));
  }
  Serial.print(F("PASS: layout defaults and hardware getters\n\r"));

  if (!trackpad.beginConfiguration()) {
    halt(F("settings window"));
  }
  // All trial changes are restored before scanning resumes. Shared-register
  // checks make sure one setting does not overwrite its neighboring field.
  if (!trackpad.setDynamicFilterBeta(19) ||
      trackpad.getDynamicFilterBeta() != 19 || trackpad.getStaticFilterBeta() != 128 ||
      !trackpad.setStaticFilterBeta(201) || trackpad.getStaticFilterBeta() != 201 ||
      trackpad.getDynamicFilterBeta() != 19 ||
      !trackpad.setDynamicFilterBeta(7) || !trackpad.setStaticFilterBeta(128)) {
    halt(F("filter fields"));
  }
  if (!trackpad.setTouchThresholds(30, 23) ||
      trackpad.getTouchSetThreshold() != 30 || trackpad.getTouchClearThreshold() != 23 ||
      !trackpad.setTouchThresholds(24, 20)) {
    halt(F("threshold fields"));
  }
  if (!trackpad.setATICoarseDivider(7, IQS7211E_ALP) ||
      trackpad.getATICoarseDivider(IQS7211E_ALP) != 7 ||
      trackpad.getATICoarseMultiplier(IQS7211E_ALP) != 1 ||
      trackpad.getATIFineDivider(IQS7211E_ALP) != 1 ||
      !trackpad.setATICoarseMultiplier(9, IQS7211E_ALP) ||
      trackpad.getATICoarseMultiplier(IQS7211E_ALP) != 9 ||
      !trackpad.setATIFineDivider(21, IQS7211E_ALP) ||
      trackpad.getATIFineDivider(IQS7211E_ALP) != 21 ||
      trackpad.getATICoarseDivider(IQS7211E_ALP) != 7 ||
      !trackpad.setATICoarseDivider(3, IQS7211E_ALP) ||
      !trackpad.setATICoarseMultiplier(1, IQS7211E_ALP) ||
      !trackpad.setATIFineDivider(1, IQS7211E_ALP)) {
    halt(F("ATI gain fields"));
  }
  if (!trackpad.setReportRate(75, IQS7211E_MODE_IDLE_TOUCH) ||
      trackpad.getReportRate(IQS7211E_MODE_IDLE_TOUCH) != 75 ||
      trackpad.getReportRate() != 20 ||
      !trackpad.setReportRate(50, IQS7211E_MODE_IDLE_TOUCH) ||
      !trackpad.setEdgeTrim(IQS7211E_AXIS_Y, 9) ||
      trackpad.getEdgeTrim(IQS7211E_AXIS_Y) != 9 ||
      trackpad.getEdgeTrim(IQS7211E_AXIS_X) != 0 ||
      !trackpad.setEdgeTrim(IQS7211E_AXIS_Y, 0)) {
    halt(F("mode and axis selectors"));
  }
  if (!trackpad.setAutoProxCycles(IQS7211E_LP1, IQS7211E_AUTOPROX_16) ||
      trackpad.getAutoProxCycles(IQS7211E_LP1) != IQS7211E_AUTOPROX_16 ||
      trackpad.getAutoProxCycles(IQS7211E_LP2) != IQS7211E_AUTOPROX_DISABLED ||
      trackpad.getCountLimit(IQS7211E_ALP) != IQS7211E_COUNT_1023 ||
      !trackpad.setAutoProxCycles(IQS7211E_LP1, IQS7211E_AUTOPROX_DISABLED) ||
      !trackpad.setCountLimit(IQS7211E_COUNT_2047, IQS7211E_ALP) ||
      trackpad.getCountLimit(IQS7211E_ALP) != IQS7211E_COUNT_2047 ||
      trackpad.getCountLimit() != IQS7211E_COUNT_1023 ||
      !trackpad.setCountLimit(IQS7211E_COUNT_1023, IQS7211E_ALP)) {
    halt(F("analog fields and sensor selector"));
  }
  if (!trackpad.enableGesture(IQS7211E_GESTURE_SWIPE_Y_NEG, true) ||
      !trackpad.getGestureEnabled(IQS7211E_GESTURE_SWIPE_Y_NEG) ||
      trackpad.getGestureEnabled(IQS7211E_GESTURE_SINGLE_TAP) ||
      !trackpad.enableGesture(IQS7211E_GESTURE_SWIPE_Y_NEG, false) ||
      !trackpad.enableEvent(IQS7211E_EVENT_ALP, true) ||
      !trackpad.getEventEnabled(IQS7211E_EVENT_ALP) ||
      trackpad.getEventEnabled(IQS7211E_EVENT_TOUCH) ||
      !trackpad.enableEvent(IQS7211E_EVENT_ALP, false)) {
    halt(F("gesture and event fields"));
  }
  if (trackpad.getSensingCycleChannel(0, IQS7211E_ENGINE_A) != 0 ||
      trackpad.getSensingCycleChannel(0, IQS7211E_ENGINE_B) != 3 ||
      trackpad.getSensingCycleChannel(20, IQS7211E_ENGINE_A) != 38 ||
      trackpad.getSensingCycleChannel(20, IQS7211E_ENGINE_B) != 41 ||
      !trackpad.endConfiguration()) {
    halt(F("cycle allocation"));
  }
  Serial.print(F("PASS: settings round trips preserve neighboring fields\n\r"));

  if (!trackpad.beginConfiguration()) {
    halt(F("invalid-setting window"));
  }
  if (trackpad.setSensingCycle(0, 3, 0) ||
      trackpad.getLastError() != IQS7211E_ERROR_ARGUMENT ||
      trackpad.endConfiguration()) {
    halt(F("invalid RX engine allocation accepted"));
  }
  if (trackpad.setTouchThresholds(20, 24) || trackpad.setConversionPeriod(128) ||
      trackpad.setATICoarseDivider(0) ||
      trackpad.getLastError() != IQS7211E_ERROR_ARGUMENT) {
    halt(F("invalid setting accepted"));
  }
  if (trackpad.getTouchSetThreshold() != 24 || trackpad.getTouchClearThreshold() != 20) {
    halt(F("invalid write changed hardware"));
  }
  Serial.print(F("PASS: invalid settings and RX engine allocation rejected\n\r"));

  if (!trackpad.enableCommandTermination(false) ||
      trackpad.getCommandTerminationEnabled() ||
      !trackpad.setFingerSplitFactor(2) || trackpad.getFingerSplitFactor() != 2 ||
      !trackpad.setFingerSplitFactor(1) ||
      !trackpad.enableCommandTermination(true) ||
      !trackpad.getCommandTerminationEnabled()) {
    halt(F("STOP/command termination transition"));
  }
  Serial.print(F("PASS: STOP and command termination\n\r"));

  if (READY_PIN < 0) {
    Serial.print(F("SKIP: explicit requests require RDY wired and READY_PIN set\n\r"));
    Serial.print(F("ALL TESTS PASSED\n\r"));
    return;
  }
  if (!trackpad.enableCommunicationRequest(true)) {
    halt(F("enable explicit requests"));
  }
  uint16_t frames = 0;
  uint32_t start = millis();
  while (frames < 100 && millis() - start < 15000) {
    if (trackpad.readData()) {
      frames++;
    } else if (trackpad.getLastError() != IQS7211E_ERROR_NOT_READY) {
      halt(F("request-mode frame"));
    }
    delay(5);
  }
  if (frames != 100) {
    halt(F("request-mode frame timeout"));
  }
  start = millis();
  while (!trackpad.enableCommunicationRequest(false)) {
    if (trackpad.getLastError() != IQS7211E_ERROR_NOT_READY || millis() - start > 1000) {
      halt(F("disable explicit requests"));
    }
    delay(5);
  }
  Serial.print(F("PASS: 100 RDY frames with explicit communication requests\n\r"));
  Serial.print(F("ALL TESTS PASSED\n\r"));
}

void loop() {}

void halt(const __FlashStringHelper *message) {
  Serial.print(F("FAIL: "));
  Serial.print(message);
  Serial.print(F("; error="));
  Serial.print((int)trackpad.getLastError());
  Serial.print(F("\n\r"));
  Adafruit_I2CDevice debug(IQS7211E_I2C_ADDRESS);
  debug.begin(false);
  Adafruit_BusIO_Register product(&debug, IQS7211E_REG_PRODUCT, 2, LSBFIRST);
  Adafruit_BusIO_Register config(&debug, IQS7211E_REG_CONFIG, 2, LSBFIRST);
  Adafruit_BusIO_Register end(&debug, IQS7211E_REG_END_COMMS);
  uint16_t id = product.read();
  int ready = digitalRead(2);
  uint16_t settings = config.read();
  end.write((uint32_t)0, 1);
  Serial.print(F("Diagnostic product/config/RDY during window: "));
  Serial.print(id);
  Serial.print('/');
  Serial.print(settings, HEX);
  Serial.print('/');
  Serial.print(ready);
  Serial.print(F("\n\r"));
  while (true) {
    delay(10);
  }
}
