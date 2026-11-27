// Adafruit IQS7211E live XY display. Open extras/visualizer/index.html through
// the LAN serial monitor bridge. All 42 points use Rev B or repaired Rev A.
// Connect power, ground, SDA and SCL; connect RDY to Metro pin 2.
// RDY uses the breakout's 3.3 V pull-up; never enable INPUT_PULLUP on a 5 V host.
// The Metro's red LED on pin 13 blinks while a valid touch is detected.
// Serial commands: c recalibrates; d toggles raw D/count and R/reference rows.
// Threshold set/clear presets (units of 1/128): 1=32/16, 2=16/8, 3=64/32,
// 4=24/20. These commands do not change the finger-splitting setting.
// Diagnostic rows are TX-major (six RX values per TX) and slow the display.

#include <Adafruit_IQS7211E.h>

Adafruit_IQS7211E trackpad;
const int16_t READY_PIN = 2;
bool diagnostic = false;
uint32_t lastDiagnostic = 0;
void printChannels(char label, uint16_t* values);
void printError(const __FlashStringHelper* message);
void updateTouchLED(bool touched);

void setup() {
  Serial.begin(115200);
  delay(250);

  Serial.print(F("Adafruit IQS7211E live touch display\n\r"));
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.print(F("Leave the pad untouched during calibration.\n\r"));
  // The driver configures RDY as INPUT and checks it before reading each frame.
  if (!trackpad.begin(&Wire, READY_PIN)) {
    printError(F("ERROR: initialization failed: "));
    while (true) {
      delay(10);
    }
  }
  Serial.print(F("Ready: all 42 points calibrated; RDY on pin 2.\n\r"));
}

void loop() {
  char command = 0;
  if (Serial.available()) {
    command = Serial.read();
  }
  if (command == 'd') {
    diagnostic = !diagnostic;
    Serial.print(F("Diagnostic streaming toggled\n\r"));
  }
  if (command >= '1' && command <= '4') {
    uint8_t threshold = 32;
    uint8_t clearThreshold = 16;
    if (command == '2') {
      threshold = 16;
      clearThreshold = 8;
    } else if (command == '3') {
      threshold = 64;
      clearThreshold = 32;
    } else if (command == '4') {
      // Above the measured residual corner signal, with narrower hysteresis.
      threshold = 24;
      clearThreshold = 20;
    }
    if (!trackpad.setTouchThresholds(threshold, clearThreshold)) {
      Serial.print(F("ERROR: threshold write failed\n\r"));
    } else {
      Serial.print(F("Touch set threshold: "));
      Serial.print(trackpad.getTouchSetThreshold());
      Serial.print(F("; clear threshold: "));
      Serial.print(trackpad.getTouchClearThreshold());
      Serial.print(F("\n\r"));
    }
  }
  if (command == 'c') {
    updateTouchLED(false);
    Serial.print(F("Calibrating: lift all fingers.\n\r"));
    delay(1000);
    if (!trackpad.calibrate()) {
      printError(F("ERROR: calibration failed: "));
    }
  }

  if (diagnostic && millis() - lastDiagnostic >= 200) {
    lastDiagnostic = millis();
    uint16_t values[IQS7211E_CHANNELS];
    if (trackpad.readChannelCounts(values)) {
      printChannels('D', values);
    }
    if (trackpad.readChannelReferences(values)) {
      printChannels('R', values);
    }
  }

  if (!trackpad.readData()) {
    if (trackpad.getLastError() == IQS7211E_ERROR_NOT_READY) {
      // RDY high is normal between reports. Keep the last displayed touch.
      delay(1);
      return;
    }
    updateTouchLED(false);
    printError(F("ERROR: acquisition failed: "));
    delay(100);
    return;
  }
  if (trackpad.tooManyFingers()) {
    updateTouchLED(false);
    Serial.print(F("ERROR: too many fingers\n\r"));
    delay(20);
    return;
  }

  updateTouchLED(trackpad.isTouched(0) || trackpad.isTouched(1));

  // One complete acquisition per line, including no-touch frames:
  // T,count,x0,y0,strength0,area0,x1,y1,strength1,area1
  // Inactive slots have area 0. Coordinates range from 0 to 1000.
  Serial.print(F("T,"));
  Serial.print(trackpad.getNumTouches());
  for (uint8_t finger = 0; finger < IQS7211E_MAX_TOUCHES; finger++) {
    Serial.print(',');
    Serial.print(trackpad.getX(finger));
    Serial.print(',');
    Serial.print(trackpad.getY(finger));
    Serial.print(',');
    Serial.print(trackpad.getTouchStrength(finger));
    Serial.print(',');
    Serial.print(trackpad.getArea(finger));
  }
  Serial.print(F("\n\r"));
  delay(20);
}

void printChannels(char label, uint16_t* values) {
  Serial.print(label);
  for (uint8_t channel = 0; channel < IQS7211E_CHANNELS; channel++) {
    Serial.print(',');
    Serial.print(values[channel]);
  }
  Serial.print(F("\n\r"));
}

void updateTouchLED(bool touched) {
  static bool wasTouched = false;
  static bool ledOn = false;
  static uint32_t lastBlink = 0;

  if (!touched) {
    digitalWrite(LED_BUILTIN, LOW);
    wasTouched = false;
    ledOn = false;
    return;
  }

  uint32_t now = millis();
  if (!wasTouched) {
    // Light immediately on a new touch, including a short tap.
    ledOn = true;
    lastBlink = now;
  } else if (now - lastBlink >= 200) {
    // Blink without adding delays to acquisition or the visual display.
    ledOn = !ledOn;
    lastBlink = now;
  }
  digitalWrite(LED_BUILTIN, ledOn);
  wasTouched = true;
}

void printError(const __FlashStringHelper* message) {
  Serial.print(message);
  switch (trackpad.getLastError()) {
    case IQS7211E_ERROR_NONE:
      Serial.print(F("no error recorded"));
      break;
    case IQS7211E_ERROR_I2C:
      Serial.print(F("I2C transaction or invalid response"));
      break;
    case IQS7211E_ERROR_DEVICE:
      Serial.print(F("unexpected device ID"));
      break;
    case IQS7211E_ERROR_NOT_READY:
      Serial.print(F("RDY inactive"));
      break;
    case IQS7211E_ERROR_RESET:
      Serial.print(F("sensor reset; restart demo"));
      break;
    case IQS7211E_ERROR_ATI:
      Serial.print(F("ATI calibration error"));
      break;
    case IQS7211E_ERROR_TIMEOUT:
      Serial.print(F("calibration or initialization timed out"));
      break;
    case IQS7211E_ERROR_ARGUMENT:
      Serial.print(F("invalid argument"));
      break;
    case IQS7211E_ERROR_NOT_INITIALIZED:
      Serial.print(F("sensor not initialized; restart demo"));
      break;
    default:
      Serial.print(F("unknown error"));
      break;
  }
  Serial.print(F("\n\r"));
}
