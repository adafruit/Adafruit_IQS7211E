# Adafruit IQS7211E [![Arduino Library CI](https://github.com/adafruit/Adafruit_IQS7211E/actions/workflows/githubci.yml/badge.svg)](https://github.com/adafruit/Adafruit_IQS7211E/actions/workflows/githubci.yml) [![Documentation](https://img.shields.io/badge/documentation-doxygen-blue.svg)](https://adafruit.github.io/Adafruit_IQS7211E/html/index.html)

Arduino driver for the Azoteq IQS7211E capacitive trackpad controller, using
Adafruit BusIO. Reads up to two fingers with stable tracking slots, adjustable
coordinate ranges, touch thresholds, report intervals, and axis orientation.

**Board wiring:** The default profile scans all 42 sensing points in 21 cycles.
It requires Rev B or a Rev A board with the RX6/TX3 connections physically
exchanged. Bottom-to-top receiver pins are 0,1,2,6,4,5; left-to-right transmitter
pins are 12,11,10,9,8,7,3.

The default profile is specific to this electrode layout. Sensitivity depends
on the touch surface and overlay; bare soldermask on the diamond side can give
uneven tracking. Keep the final overlay in place during calibration, and
recalibrate after changing the touch surface. Simultaneous two-sided use is
not supported by this profile.

Install this folder in your Arduino libraries directory and install **Adafruit
BusIO** through Library Manager. Open **File > Examples > Adafruit IQS7211E >
touchdemo**, select the Metro 328/Arduino Uno, and use a 115200 baud Serial
Monitor. Connect power, ground, SDA and SCL. Leave the pad untouched during
startup calibration.

`begin()` uses I2C polling without RDY or an interrupt. `readData()` acquires
one complete frame; `getNumTouches()`, `isTouched()`, `getX()`, `getY()`,
`getTouchStrength()` and `getArea()` refer to that frame. Coordinates default
to 0–1000 on each axis. Strength is a raw capacitive signal, not pressure.
Finger slots retain their identity when another finger is lifted.

By default, the driver keeps the controller in active mode, with 20 ms reports. Polling
uses I2C clock stretching, so the bus can wait for the next communication
window. AVR Wire implementations with timeout support get a 250 ms bus timeout.
The optional RDY input uses the breakout's 3.3 V pull-up; keep the host's
internal pull-up disabled.

`begin()` applies the board defaults through the public configuration API.
There is no configuration header to edit: use setters and hardware-reading
getters for sensitivity, calibration, analog acquisition, power modes, timing,
filters, orientation, gestures, ALP sensing, and electrode allocation. The
**settings** example shows common adjustments. For example:

```cpp
trackpad.setTouchThresholds(24, 20);  // Set/clear fractions in 1/128 units
trackpad.setReportRate(20);          // Active interval in milliseconds
trackpad.setFingerSplitFactor(1);
uint8_t beta = trackpad.getDynamicFilterBeta();
```

Use `beginConfiguration()` and `endConfiguration()` to group related changes.
Always close the group, even when a setter fails; failed groups do not roll back
earlier writes. Electrode counts, mapping, and sensing cycles require a group.
Cycle engine A uses physical RX0–3 and engine B uses RX4–7, both on the same TX.
After changing electrodes or analog/calibration settings, close the group and
call `calibrate()` with the final touch surface in place and no fingers on it.
Settings are volatile; another `begin()` restores the defaults.
If startup reaches calibration but reports an ATI error, the configuration API
remains available for adjusting the settings and retrying `calibrate()`.

Setters return success, and `getLastError()` reports failures. Configuration
getters read the chip and return zero/false on failure, except touch-threshold
getters and cycle-channel getters, which return 255. Check the error immediately
when zero or 255 is also a valid setting. Request-based communication requires
RDY and explicit command termination; an operation returning
`IQS7211E_ERROR_NOT_READY` should be retried after RDY goes low. Low-power
Auto-Prox can cause long clock stretching on firmware 1.1, so use RDY and
`enableCommunicationRequest(true)` for those configurations. STOP termination
is available, but coherent multi-transaction channel-bank reads require command
termination. Register meanings and units follow the
[Azoteq datasheet, section 12 and Appendix A](https://www.azoteq.com/images/stories/pdf/IQS7211E_Datasheet_v1.6.pdf).

For a browser display, connect RDY to Metro pin 2 and upload the **visualdemo**
example. It checks RDY before reading frames, without installing an interrupt.
The Metro's red LED blinks while a touch is detected. Serve
`extras/visualizer/index.html` on the same host as the LAN serial monitor's
`/serial/` bridge, connected to the Metro at 115200 baud. It displays both finger
slots, motion trails, release, and stale-data status across the complete pad.
The browser needs the running serial bridge; opening the
HTML file alone does not connect to USB. The serial bridge is a separate tool
and is not included in this library.

MIT license; see LICENSE. Written by Adafruit.
