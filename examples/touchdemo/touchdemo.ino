// Adafruit IQS7211E touch demo for the 7 TX by 6 RX trackpad.
// Connect power, ground, SDA and SCL. RDY is not connected by default.
// Install Adafruit BusIO as well as this library.
// All 42 points require Rev B or the RX6/TX3 trace swap on Rev A.

#include <Adafruit_IQS7211E.h>

Adafruit_IQS7211E trackpad;

void setup() {
  Serial.begin(115200);
  // Wait for the Serial Monitor on native-USB boards. Remove this wait when
  // running without a computer attached; it does not block on a Metro 328.
  while (!Serial) {
    delay(10);
  }
  delay(250);

  Serial.print(F("Adafruit IQS7211E touch demo\n\r"));
  Serial.print(F("42 points: Rev B or repaired Rev A wiring required.\n\r"));
  Serial.print(F("Keep the pad untouched during startup calibration.\n\r"));
  if (!trackpad.begin()) {
    Serial.print(F("Trackpad initialization failed; error="));
    Serial.print((int)trackpad.getLastError());
    Serial.print(F("\n\r"));
    while (true) {
      delay(10);
    }
  }

  // begin() loads the board's electrode map and calibrates all enabled channels.
  // Defaults: 20 ms reports, two fingers, X/Y ranges of 0-1000, no gestures.
  // Coordinates are output units, not millimeters. Strength is not pressure.
  // For a different display size: trackpad.setResolution(800, 480);
  // To require a stronger touch: trackpad.setTouchThresholds(32, 24);
  // To reverse the axes: trackpad.setOrientation(true, true);

  Serial.print(F("Firmware "));
  Serial.print(trackpad.getFirmwareMajor());
  Serial.print('.');
  Serial.print(trackpad.getFirmwareMinor());
  Serial.print(F("; I2C polling, no RDY pin\n\r"));
  Serial.print(F("Touch or slide one or two fingers; send c to recalibrate.\n\r"));
  Serial.print(F("Fingers\tSlot\tX\tY\tStrength\tArea\n\r"));
}

void loop() {
  if (Serial.available() && Serial.read() == 'c') {
    Serial.print(F("Calibrating: lift all fingers.\n\r"));
    delay(1000);
    if (!trackpad.calibrate()) {
      Serial.print(F("Calibration failed\n\r"));
    }
  }

  if (!trackpad.readData()) {
    Serial.print(F("Read failed; error="));
    Serial.print((int)trackpad.getLastError());
    Serial.print(F("\n\r"));
    delay(100);
    return;
  }

  static uint8_t previousTouches = 0;
  uint8_t touches = trackpad.getNumTouches();
  if (trackpad.tooManyFingers()) {
    Serial.print(F("Too many fingers\n\r"));
  } else if (touches == 0 && previousTouches != 0) {
    Serial.print(F("Released\n\r"));
  }

  // Slots retain their identity. If finger 0 lifts, finger 1 can remain in
  // slot 1 even though getNumTouches() now returns 1.
  for (uint8_t finger = 0; finger < IQS7211E_MAX_TOUCHES; finger++) {
    if (!trackpad.isTouched(finger)) {
      continue;
    }
    Serial.print(touches);
    Serial.print('\t');
    Serial.print(finger);
    Serial.print('\t');
    Serial.print(trackpad.getX(finger));
    Serial.print('\t');
    Serial.print(trackpad.getY(finger));
    Serial.print('\t');
    Serial.print(trackpad.getTouchStrength(finger));
    Serial.print('\t');
    Serial.print(trackpad.getArea(finger));
    Serial.print(F("\n\r"));
  }
  previousTouches = touches;
  delay(100);
}
