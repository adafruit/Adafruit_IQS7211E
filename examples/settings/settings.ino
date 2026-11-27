#include <Adafruit_IQS7211E.h>

Adafruit_IQS7211E trackpad;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.print(F("IQS7211E runtime settings\n\r"));
  Serial.print(F("Leave the pad untouched during startup calibration.\n\r"));

  // begin() supplies the Adafruit board defaults and calibrates all 42 points.
  // No RDY connection is needed for this example.
  if (!trackpad.begin()) {
    halt(F("Initialization failed"));
  }

  // Group related changes so the chip resumes scanning after all are written.
  // These setters can also be called individually. Smaller touch thresholds
  // are more sensitive; the clear threshold must remain below the set threshold.
  if (!trackpad.beginConfiguration()) {
    halt(F("Cannot open settings window"));
  }
  bool configured = trackpad.setTouchThresholds(24, 20) &&
                    trackpad.setReportRate(20) &&
                    trackpad.setResolution(1000, 1000) &&
                    trackpad.setOrientation(false, false, true) &&
                    trackpad.setMaxTouches(2) &&
                    trackpad.enableMAVFilter(true) &&
                    trackpad.enableIIRFilter(true) &&
                    trackpad.enableStaticIIR(false) &&
                    trackpad.setDynamicFilterSpeeds(6, 124) &&
                    trackpad.setDynamicFilterBeta(7) &&
                    trackpad.setStaticFilterBeta(128) &&
                    trackpad.setFingerSplitFactor(1);
  // Always close the window, including when a setter fails.
  bool closed = trackpad.endConfiguration();
  if (!configured || !closed) {
    halt(F("Settings update failed"));
  }

  // Getters read the controller, rather than returning cached setter values.
  Serial.print(F("Touch set/clear (1/128 units): "));
  Serial.print(trackpad.getTouchSetThreshold());
  Serial.print('/');
  Serial.print(trackpad.getTouchClearThreshold());
  Serial.print(F("\n\rReport interval (ms): "));
  Serial.print(trackpad.getReportRate());
  Serial.print(F("\n\rDynamic/static IIR beta: "));
  Serial.print(trackpad.getDynamicFilterBeta());
  Serial.print('/');
  Serial.print(trackpad.getStaticFilterBeta());
  Serial.print(F("\n\rRX/TX count: "));
  Serial.print(trackpad.getReceiverCount());
  Serial.print('/');
  Serial.print(trackpad.getTransmitterCount());
  Serial.print(F("\n\rMove one or two fingers on the pad.\n\r"));

  // Calibration/analog settings also have setters and hardware getters:
  // setATITarget(), setATICoarseDivider(), setConversionPeriod(),
  // setChargeCapacitor(), and others accept IQS7211E_TRACKPAD or IQS7211E_ALP.
  // After changing those settings, call calibrate() with the pad untouched.
  // To change the physical layout, group setElectrodeCounts(),
  // setElectrodeMapping(), and setSensingCycle(), then close and calibrate.
}

void loop() {
  if (trackpad.readData()) {
    for (uint8_t finger = 0; finger < IQS7211E_MAX_TOUCHES; finger++) {
      if (trackpad.isTouched(finger)) {
        Serial.print(F("Finger "));
        Serial.print(finger + 1);
        Serial.print(F(": "));
        Serial.print(trackpad.getX(finger));
        Serial.print(F(", "));
        Serial.print(trackpad.getY(finger));
        Serial.print(F("\n\r"));
      }
    }
  }
  delay(20);
}

void halt(const __FlashStringHelper *message) {
  Serial.print(message);
  Serial.print(F("; error="));
  Serial.print((int)trackpad.getLastError());
  Serial.print(F("\n\r"));
  while (true) {
    delay(10);
  }
}
