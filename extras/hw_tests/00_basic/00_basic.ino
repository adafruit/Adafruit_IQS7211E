#include <Adafruit_IQS7211E.h>

Adafruit_IQS7211E trackpad;

void setup() {
  Serial.begin(115200);
  delay(250);

  Serial.print(F("Adafruit IQS7211E basic hardware test\n\r"));
  Serial.print(F("Leave the trackpad untouched during calibration.\n\r"));

  if (!trackpad.begin()) {
    halt(F("FAIL: begin"));
  }
  Serial.print(F("PASS: begin and calibration\n\r"));
  if (trackpad.getProductNumber() != IQS7211E_PRODUCT_NUMBER) {
    halt(F("FAIL: product number"));
  }
  Serial.print(F("PASS: product number 1112\n\r"));
  Serial.print(F("Firmware: "));
  Serial.print(trackpad.getFirmwareMajor());
  Serial.print('.');
  Serial.print(trackpad.getFirmwareMinor());
  Serial.print(F("\n\r"));

  if (!trackpad.reset()) {
    halt(F("FAIL: software reset"));
  }
  Serial.print(F("PASS: software reset\n\r"));
  if (!trackpad.begin()) {
    halt(F("FAIL: second begin"));
  }
  Serial.print(F("PASS: second begin and calibration\n\r"));

  // Every intersection must calibrate without reaching an ATI limit.
  uint16_t compensation[IQS7211E_CHANNELS];
  if (!trackpad.readATICompensation(compensation)) {
    halt(F("FAIL: compensation read"));
  }
  Serial.print(F("Compensation:"));
  for (uint8_t channel = 0; channel < IQS7211E_CHANNELS; channel++) {
    Serial.print(' ');
    Serial.print(compensation[channel]);
    if (compensation[channel] == 0 || compensation[channel] == 1023) {
      halt(F("FAIL: channel at ATI limit"));
    }
  }
  Serial.print(F("\n\rPASS: all 42 channels calibrated within ATI limits\n\r"));

  for (uint16_t sample = 0; sample < 500; sample++) {
    if (!trackpad.readData()) {
      halt(F("FAIL: polling frame"));
    }
    delay(20);
  }
  Serial.print(F("PASS: 500 coherent frames without RDY\n\r"));
  Serial.print(F("ALL TESTS PASSED\n\r"));
}

void loop() {}

void halt(const __FlashStringHelper *message) {
  Serial.print(message);
  Serial.print(F("; error="));
  Serial.print((int)trackpad.getLastError());
  Serial.print(F("\n\r"));
  Adafruit_I2CDevice debug(IQS7211E_I2C_ADDRESS);
  debug.begin(false);
  Adafruit_BusIO_Register system(&debug, IQS7211E_REG_SYSTEM, 2, LSBFIRST);
  Adafruit_BusIO_Register info(&debug, IQS7211E_REG_INFO, 2, LSBFIRST);
  Adafruit_BusIO_Register end(&debug, IQS7211E_REG_END_COMMS);
  Serial.print(F("System=0x"));
  Serial.print(system.read(), HEX);
  Serial.print(F(" Info=0x"));
  Serial.print(info.read(), HEX);
  Serial.print(F("\n\r"));
  end.write((uint32_t)0, 1);
  uint16_t values[IQS7211E_CHANNELS];
  if (trackpad.readATICompensation(values)) {
    Serial.print(F("Compensation:"));
    for (uint8_t i = 0; i < IQS7211E_CHANNELS; i++) {
      Serial.print(' ');
      Serial.print(values[i]);
    }
    Serial.print(F("\n\r"));
  }
  if (trackpad.readChannelCounts(values)) {
    Serial.print(F("Counts:"));
    for (uint8_t i = 0; i < IQS7211E_CHANNELS; i++) {
      Serial.print(' ');
      Serial.print(values[i]);
    }
    Serial.print(F("\n\r"));
  }
  while (true) {
    delay(10);
  }
}
