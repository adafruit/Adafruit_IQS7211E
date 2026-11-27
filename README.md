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

The driver keeps the controller in active mode, with 20 ms reports. Polling
uses I2C clock stretching, so the bus can wait for the next communication
window. AVR Wire implementations with timeout support get a 250 ms bus timeout.
The optional RDY input uses the breakout's 3.3 V pull-up; keep the host's
internal pull-up disabled.

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
