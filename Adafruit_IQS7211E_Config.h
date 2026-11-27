/*!
 * @file Adafruit_IQS7211E_Config.h
 * Default register profile for the Adafruit 7 TX by 6 RX trackpad.
 * Register meanings: Azoteq IQS7211E datasheet v1.6, section 12 and Appendix A.
 * Values are initial board tuning, not a universal IQS7211E configuration.
 * All 42 channels require Rev B or the RX6/TX3 trace swap on Rev A.
 */
#ifndef ADAFRUIT_IQS7211E_CONFIG_H
#define ADAFRUIT_IQS7211E_CONFIG_H

#include "Adafruit_IQS7211E.h"

/** Consecutive 16-bit words, registers 0x1F through 0x55. Stored in AVR flash.
 */
static const uint16_t iqs7211e_default_settings[] PROGMEM = {
    0,      // 0x1F: ALP A compensation; ALP is not used in active-only mode.
    0,      // 0x20: ALP B compensation.
    0x0A81, // 0x21: TP coarse divider 1, multiplier 4, fine divider 5.
    0x3206, // 0x22: Drift limit 50; divider 6 gives the repaired board ATI
            // range.
    300,    // 0x23: Trackpad ATI target count.
    50,     // 0x24: Negative count change that requests automatic re-ATI.
    0x0223, // 0x25: ALP coarse divider 3, multiplier 1, fine divider 1.
    0x1404, // 0x26: ALP drift limit 20, compensation divider 4.
    200,    // 0x27: ALP ATI target (inactive).
    20,     // 0x28: Active report interval, milliseconds (50 Hz).
    50,     // 0x29: Idle-touch report interval, milliseconds.
    20,     // 0x2A: Idle report interval, milliseconds.
    80,     // 0x2B: LP1 report interval, milliseconds (inactive).
    160,    // 0x2C: LP2 report interval, milliseconds (inactive).
    0,      // 0x2D: No automatic active-mode timeout.
    0,      // 0x2E: No automatic idle-touch timeout.
    0,      // 0x2F: No automatic idle timeout.
    0,      // 0x30: No automatic LP1 timeout.
    0x0805, // 0x31: Reference update 8 s, failed ATI retry 5 s.
    100,    // 0x32: Communication-window timeout, milliseconds.
    0,      // 0x33: Manual active mode; command bits initially clear.
    IQS7211E_CONFIG_POLLING, // 0x34: Explicit end, WDT, automatic TP ATI.
    0,                       // 0x35: 14 MHz oscillator, no adjustment.
    0x0377, // 0x36: ALP mutual mode, filtering, RX0/1/2/4/5/6 (inactive).
    0x1F88, // 0x37: ALP TX3/7/8/9/10/11/12 (inactive).
    0x1418, // 0x38: Direct-touch trial: set 24/128, clear 20/128.
    8,      // 0x39: ALP threshold, raw counts (inactive).
    0x0404, // 0x3A: ALP set/clear debounce 4 cycles (inactive).
    0x08DC, // 0x3B: ALP LP1 LTA beta 8, count beta 220.
    0x10F0, // 0x3C: ALP LP2 LTA beta 16, count beta 240.
    0x7F1A, // 0x3D: TP conversion period 26, fraction 127: 250 kHz.
    0x7F1A, // 0x3E: ALP conversion frequency: 250 kHz (inactive).
    0x1C03, // 0x3F: NM static off, 80 pF, 10 uA, 1023 max, 64 init cycles.
    0x1C93, // 0x40: Same analog settings; LP1/LP2 Auto-Prox both disabled.
    0x062C, // 0x41: 6 RX, MAV and dynamic IIR, TX columns, no flips.
    0x0207, // 0x42: Maximum 2 fingers, 7 TX.
    1000,   // 0x43: X output range.
    1000,   // 0x44: Y output range.
    6,      // 0x45: Dynamic-filter bottom speed, coordinate units/report.
    124,    // 0x46: Dynamic-filter top speed, coordinate units/report.
    0x8007, // 0x47: Static beta 128, dynamic bottom beta 7.
    0x0114, // 0x48: Gentler finger split factor 1, stationary threshold 20.
    0,      // 0x49: No X or Y edge trimming.
    2,      // 0x4A: Board settings version 0.2, all 42 points.
    0,      // 0x4B: Gestures disabled; touch coordinates remain available.
    150,    // 0x4C: Tap maximum touch time, milliseconds (inactive).
    150,    // 0x4D: Inter-tap wait, milliseconds (inactive).
    50,     // 0x4E: Tap motion limit, coordinate units (inactive).
    300,    // 0x4F: Hold time, milliseconds (inactive).
    150,    // 0x50: Swipe time, milliseconds (inactive).
    200,    // 0x51: Initial swipe X distance (inactive).
    200,    // 0x52: Initial swipe Y distance (inactive).
    100,    // 0x53: Consecutive swipe X distance (inactive).
    100,    // 0x54: Consecutive swipe Y distance (inactive).
    0x1E17, // 0x55: Palm threshold 30; swipe angle 23 (inactive).
};

static_assert(sizeof(iqs7211e_default_settings) == 110,
              "IQS7211E settings must cover registers 0x1F-0x55");

#endif
