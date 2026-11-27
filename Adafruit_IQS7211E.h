/*!
 * @file Adafruit_IQS7211E.h
 *
 * Arduino driver for the Azoteq IQS7211E trackpad controller.
 * Written by Adafruit. MIT license; see LICENSE.
 */
#ifndef ADAFRUIT_IQS7211E_H
#define ADAFRUIT_IQS7211E_H

#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Arduino.h>

#define IQS7211E_I2C_ADDRESS 0x56    ///< Fixed seven-bit I2C address.
#define IQS7211E_PRODUCT_NUMBER 1112 ///< Expected product number.
#define IQS7211E_INVALID_WORD \
  0xEEEE                          ///< Device's invalid communication response.
#define IQS7211E_REG_PRODUCT 0x00 ///< Product number, 16 bits.
#define IQS7211E_REG_VERSION_MAJOR 0x01 ///< Firmware major version.
#define IQS7211E_REG_VERSION_MINOR 0x02 ///< Firmware minor version.
#define IQS7211E_REG_DATA 0x0A ///< Start of the coherent 28-byte result frame.
#define IQS7211E_REG_INFO 0x0F ///< Information flags.
#define IQS7211E_REG_CONFIG_START 0x1F ///< First writable configuration word.
#define IQS7211E_REG_ALP_COMPENSATION \
  0x1F ///< First ALP compensation register (A, then B).
#define IQS7211E_REG_ATI_COMPENSATION \
  0x22 ///< Trackpad drift limit and compensation divider.
#define IQS7211E_REG_MINIMUM_COUNT \
  0x24 ///< Negative delta that triggers automatic ATI.
#define IQS7211E_REG_ALP_ATI 0x25 ///< ALP coarse/fine dividers and multiplier.
#define IQS7211E_REG_ALP_ATI_COMPENSATION \
  0x26 ///< ALP drift limit and compensation divider.
#define IQS7211E_REG_ALP_ATI_TARGET 0x27 ///< ALP ATI target count.
#define IQS7211E_REG_MODE_TIMEOUT \
  0x2D ///< First mode timeout, active through LP1.
#define IQS7211E_REG_REFERENCE_TIMING \
  0x31 ///< Reference update and ATI retry seconds.
#define IQS7211E_REG_COMMUNICATION_TIMEOUT \
  0x32                               ///< Communication timeout in milliseconds.
#define IQS7211E_REG_OSCILLATOR 0x35 ///< Oscillator selection and adjustment.
#define IQS7211E_REG_ALP_SETUP \
  0x36 ///< ALP RX enables, sensing method and filter.
#define IQS7211E_REG_ALP_TRANSMITTERS 0x37 ///< ALP TX enables.
#define IQS7211E_REG_ALP_THRESHOLD 0x39 ///< ALP detection threshold in counts.
#define IQS7211E_REG_ALP_DEBOUNCE 0x3A  ///< ALP set and clear debounce cycles.
#define IQS7211E_REG_ALP_FILTER \
  0x3B ///< First ALP filter register (LP1, then LP2).
#define IQS7211E_REG_CONVERSION \
  0x3D ///< Trackpad conversion period and fraction.
#define IQS7211E_REG_ALP_CONVERSION \
  0x3E                                 ///< ALP conversion period and fraction.
#define IQS7211E_REG_HARDWARE 0x3F     ///< Trackpad analog hardware settings.
#define IQS7211E_REG_ALP_HARDWARE 0x40 ///< ALP analog hardware settings.
#define IQS7211E_REG_FILTER_BOTTOM_SPEED 0x45 ///< Dynamic IIR bottom speed.
#define IQS7211E_REG_FILTER_TOP_SPEED 0x46    ///< Dynamic IIR top speed.
#define IQS7211E_REG_FILTER_BETA 0x47 ///< Static and dynamic IIR beta values.
#define IQS7211E_REG_FINGER_SPLIT \
  0x48 ///< Finger split factor and stationary threshold.
#define IQS7211E_REG_EDGE_TRIM 0x49        ///< X and Y edge trim.
#define IQS7211E_REG_SETTINGS_VERSION 0x4A ///< User-defined settings version.
#define IQS7211E_REG_GESTURE_ENABLE 0x4B   ///< Gesture enable flags.
#define IQS7211E_REG_TAP_TIME 0x4C     ///< Maximum tap time in milliseconds.
#define IQS7211E_REG_AIR_TIME 0x4D     ///< Inter-tap air time in milliseconds.
#define IQS7211E_REG_TAP_DISTANCE 0x4E ///< Maximum tap displacement.
#define IQS7211E_REG_HOLD_TIME 0x4F    ///< Hold time in milliseconds.
#define IQS7211E_REG_SWIPE_TIME 0x50   ///< Swipe time in milliseconds.
#define IQS7211E_REG_SWIPE_INITIAL \
  0x51 ///< Initial X swipe distance, followed by Y.
#define IQS7211E_REG_SWIPE_CONSECUTIVE \
  0x53 ///< Consecutive X swipe distance, followed by Y.
#define IQS7211E_REG_GESTURE_SETTINGS 0x55 ///< Palm threshold and swipe angle.
#define IQS7211E_REG_ATI 0x21        ///< Trackpad ATI dividers and multiplier.
#define IQS7211E_REG_ATI_TARGET 0x23 ///< Trackpad ATI target count.
#define IQS7211E_REG_REPORT_RATE \
  0x28                           ///< Active report interval in milliseconds.
#define IQS7211E_REG_SYSTEM 0x33 ///< Reset, calibration and mode commands.
#define IQS7211E_REG_CONFIG 0x34 ///< Communication and power configuration.
#define IQS7211E_REG_THRESHOLDS 0x38 ///< Set/clear threshold multipliers.
#define IQS7211E_REG_TRACKPAD \
  0x41 ///< Receiver count, filtering and orientation.
#define IQS7211E_REG_TX_COUNT 0x42 ///< Transmitter and maximum finger counts.
#define IQS7211E_REG_X_RESOLUTION 0x43 ///< X coordinate output range.
#define IQS7211E_REG_Y_RESOLUTION 0x44 ///< Y coordinate output range.
#define IQS7211E_REG_MAPPING 0x56      ///< Physical electrode mapping.
#define IQS7211E_REG_CYCLES 0x5D       ///< 21 sensing-cycle allocations.
#define IQS7211E_REG_COUNTS 0xE0       ///< Extended channel count bank.
#define IQS7211E_REG_REFERENCES 0xE1   ///< Extended channel reference bank.
#define IQS7211E_REG_COMPENSATION 0xE3 ///< Extended ATI compensation bank.
#define IQS7211E_REG_END_COMMS 0xFF ///< End a command-terminated comms window.
#define IQS7211E_COMMAND_RESET \
  0x0200 ///< System-control software-reset command.
#define IQS7211E_COMMAND_ACK_RESET 0x0080 ///< Acknowledge reset in active mode.
#define IQS7211E_COMMAND_CALIBRATE \
  0x0020 ///< Queue trackpad ATI in active mode.
#define IQS7211E_CONFIG_POLLING \
  0x00E4 ///< Manual active mode, end command, WDT, automatic TP ATI.
#define IQS7211E_CHANNELS \
  42 ///< Active sensing points on Rev B or repaired Rev A.
#define IQS7211E_CHANNEL_DISABLED 255 ///< Unallocated sensing-cycle slot.
#define IQS7211E_MAX_TOUCHES 2        ///< Maximum supported fingers.
#define IQS7211E_MAX_ELECTRODES 13  ///< Number of physical sensor connections.
#define IQS7211E_SENSING_CYCLES 21  ///< Number of two-engine sensing cycles.
#define IQS7211E_CYCLE_CONTROL 0x05 ///< Required ordinary cycle control byte.
#define IQS7211E_FINAL_CYCLE_CONTROL \
  0x01 ///< Required final cycle control byte.

/** Sensing engine whose calibration or analog settings are selected. */
typedef enum {
  IQS7211E_TRACKPAD, ///< Trackpad channels.
  IQS7211E_ALP       ///< Alternate low-power proximity channel.
} iqs7211e_sensor_t;

/** Charging modes, matching System Control bits 2:0 (Table A.6). */
typedef enum {
  IQS7211E_MODE_ACTIVE = 0, ///< Full-speed trackpad scanning.
  IQS7211E_MODE_IDLE_TOUCH, ///< Trackpad scanning during stationary touch.
  IQS7211E_MODE_IDLE,       ///< Reduced-rate trackpad scanning.
  IQS7211E_MODE_LP1,        ///< First low-power ALP mode.
  IQS7211E_MODE_LP2         ///< Second low-power ALP mode.
} iqs7211e_mode_t;

/** Low-power filter or Auto-Prox setting bank. */
typedef enum {
  IQS7211E_LP1, ///< LP1 settings.
  IQS7211E_LP2  ///< LP2 settings.
} iqs7211e_low_power_t;

/** Coordinate axis. */
typedef enum {
  IQS7211E_AXIS_X, ///< X coordinate.
  IQS7211E_AXIS_Y  ///< Y coordinate.
} iqs7211e_axis_t;

/** Independent proximity engine. */
typedef enum {
  IQS7211E_ENGINE_A, ///< Physical RX0 through RX3.
  IQS7211E_ENGINE_B  ///< Physical RX4 through RX7.
} iqs7211e_engine_t;

/** Event source bit positions in Config Settings, Table A.7. */
typedef enum {
  IQS7211E_EVENT_GESTURE = 9,   ///< Enabled gesture recognized.
  IQS7211E_EVENT_TRACKPAD = 10, ///< Finger movement, arrival or release.
  IQS7211E_EVENT_ATI = 11,      ///< Trackpad or ALP recalibration.
  IQS7211E_EVENT_ALP = 13,      ///< ALP proximity/touch transition.
  IQS7211E_EVENT_TOUCH = 14 ///< Individual trackpad channel touch transition.
} iqs7211e_event_t;

/** Gesture enable bit positions, Table A.14. */
typedef enum {
  IQS7211E_GESTURE_SINGLE_TAP = 0,        ///< Single tap.
  IQS7211E_GESTURE_DOUBLE_TAP = 1,        ///< Double tap.
  IQS7211E_GESTURE_TRIPLE_TAP = 2,        ///< Triple tap.
  IQS7211E_GESTURE_HOLD = 3,              ///< Press and hold.
  IQS7211E_GESTURE_PALM = 4,              ///< Palm detection.
  IQS7211E_GESTURE_SWIPE_X_POS = 8,       ///< Swipe toward increasing X.
  IQS7211E_GESTURE_SWIPE_X_NEG = 9,       ///< Swipe toward decreasing X.
  IQS7211E_GESTURE_SWIPE_Y_POS = 10,      ///< Swipe toward increasing Y.
  IQS7211E_GESTURE_SWIPE_Y_NEG = 11,      ///< Swipe toward decreasing Y.
  IQS7211E_GESTURE_SWIPE_HOLD_X_POS = 12, ///< Swipe/hold toward increasing X.
  IQS7211E_GESTURE_SWIPE_HOLD_X_NEG = 13, ///< Swipe/hold toward decreasing X.
  IQS7211E_GESTURE_SWIPE_HOLD_Y_POS = 14, ///< Swipe/hold toward increasing Y.
  IQS7211E_GESTURE_SWIPE_HOLD_Y_NEG = 15  ///< Swipe/hold toward decreasing Y.
} iqs7211e_gesture_t;

/** Mutual-capacitance amplifier bias, Table A.12. */
typedef enum {
  IQS7211E_BIAS_2UA, ///< 2 microamperes.
  IQS7211E_BIAS_5UA, ///< 5 microamperes.
  IQS7211E_BIAS_7UA, ///< 7 microamperes.
  IQS7211E_BIAS_10UA ///< 10 microamperes.
} iqs7211e_bias_t;

/** Conversion count ceiling, Table A.12. */
typedef enum {
  IQS7211E_COUNT_1023, ///< Stop at 1023 counts.
  IQS7211E_COUNT_2047, ///< Stop at 2047 counts.
  IQS7211E_COUNT_4095, ///< Stop at 4095 counts.
  IQS7211E_COUNT_16384 ///< Stop at 16384 counts.
} iqs7211e_count_limit_t;

/** Initial conversion delay, Table A.12. */
typedef enum {
  IQS7211E_INIT_4,  ///< Four cycles.
  IQS7211E_INIT_16, ///< Sixteen cycles.
  IQS7211E_INIT_32, ///< Thirty-two cycles.
  IQS7211E_INIT_64  ///< Sixty-four cycles.
} iqs7211e_init_delay_t;

/** Hardware Auto-Prox cycles before a communication opportunity. */
typedef enum {
  IQS7211E_AUTOPROX_4,       ///< Four cycles.
  IQS7211E_AUTOPROX_8,       ///< Eight cycles.
  IQS7211E_AUTOPROX_16,      ///< Sixteen cycles.
  IQS7211E_AUTOPROX_32,      ///< Thirty-two cycles.
  IQS7211E_AUTOPROX_DISABLED ///< Disable Auto-Prox.
} iqs7211e_autoprox_t;

/** Main oscillator selection, Table A.8. */
typedef enum {
  IQS7211E_OSC_14MHZ, ///< 14 MHz, supply at least 1.71 V.
  IQS7211E_OSC_18MHZ  ///< 18 MHz, supply at least 2.2 V.
} iqs7211e_oscillator_t;

/** Internal charge-transfer capacitor, Table A.12. */
typedef enum {
  IQS7211E_CAP_40PF, ///< 40 pF.
  IQS7211E_CAP_80PF  ///< 80 pF.
} iqs7211e_capacitor_t;

/** ALP sensing method, Table A.9. */
typedef enum {
  IQS7211E_SELF_CAPACITIVE,  ///< Self-capacitive ALP sensor.
  IQS7211E_MUTUAL_CAPACITIVE ///< Mutual-capacitive ALP sensor.
} iqs7211e_sensing_t;

/** Outcome of the most recent device operation. */
typedef enum {
  IQS7211E_ERROR_NONE,   ///< Operation succeeded.
  IQS7211E_ERROR_I2C,    ///< Bus transaction failed or returned invalid data.
  IQS7211E_ERROR_DEVICE, ///< Product number did not identify an IQS7211E.
  IQS7211E_ERROR_NOT_READY, ///< Optional RDY input is inactive.
  IQS7211E_ERROR_RESET, ///< Device reset; call begin() to reload its settings.
  IQS7211E_ERROR_ATI,   ///< Electrode calibration reached a compensation limit.
  IQS7211E_ERROR_TIMEOUT,  ///< Initialization/calibration did not finish in
                           ///< time.
  IQS7211E_ERROR_ARGUMENT, ///< A numeric parameter is outside its allowed
                           ///< range.
  IQS7211E_ERROR_NOT_INITIALIZED ///< Call begin() before reading touch data.
} iqs7211e_error_t;

#pragma pack(push, 1)
/** Checked oscillator settings, Table A.8. */
typedef union {
  uint16_t value; ///< Complete register.
  struct {
    uint16_t adjustment : 4; ///< Frequency adjustment encoding.
    uint16_t oscillator : 1; ///< Oscillator selection.
    uint16_t reserved : 11;  ///< Reserved.
  } bits;                    ///< Named fields.
} iqs7211e_oscillator_settings_t;

/** Checked system-command snapshot, Table A.6. */
typedef union {
  uint16_t value; ///< Register 0x33, little endian.
  struct {
    uint16_t mode : 3;           ///< Selected manual charging mode.
    uint16_t trackpadReseed : 1; ///< Reseed trackpad references.
    uint16_t alpReseed : 1;      ///< Reseed ALP reference.
    uint16_t trackpadATI : 1; ///< Clears when trackpad calibration completes.
    uint16_t alpATI : 1;      ///< Queue ALP calibration.
    uint16_t acknowledgeReset : 1; ///< Clear the reset notification.
    uint16_t reserved8 : 1;        ///< Reserved.
    uint16_t reset : 1;            ///< Software reset after the window ends.
    uint16_t reserved10 : 1;       ///< Reserved.
    uint16_t suspend : 1;          ///< Suspend after the window ends.
    uint16_t reserved12 : 3;       ///< Reserved.
    uint16_t txTest : 1;           ///< TX short-test configuration.
  } bits; ///< Named command fields for the checked read.
} iqs7211e_system_t;

/** Checked word/byte snapshot for byte-sized settings (section 12).
 * Configuration getters check the full word for 0xEEEE before decoding it.
 */
typedef union {
  uint16_t value; ///< Complete little-endian register.
  struct {
    uint8_t low;  ///< Low byte, whose meaning depends on the named register.
    uint8_t high; ///< High byte, whose meaning depends on the named register.
  } bytes;        ///< Named byte layout.
} iqs7211e_word_t;

/** Checked ATI gain register, Table A.5. */
typedef union {
  uint16_t value; ///< Complete register.
  struct {
    uint16_t coarseDivider : 5; ///< Coarse fractional divider.
    uint16_t multiplier : 4;    ///< Coarse multiplier.
    uint16_t fineDivider : 5;   ///< Fine fractional divider.
    uint16_t reserved : 2;      ///< Reserved.
  } bits;                       ///< Named fields.
} iqs7211e_gain_t;

/** Checked communication/event configuration, Table A.7. */
typedef union {
  uint16_t value; ///< Complete register.
  struct {
    uint16_t reserved0 : 2;            ///< Reserved.
    uint16_t trackpadAutoATI : 1;      ///< Automatic trackpad calibration.
    uint16_t alpAutoATI : 1;           ///< Automatic ALP calibration.
    uint16_t communicationRequest : 1; ///< Explicit request instead of stretch.
    uint16_t watchdog : 1;             ///< Watchdog enabled.
    uint16_t commandTermination : 1;   ///< 0xFF terminates the window.
    uint16_t manualMode : 1;           ///< Host selects charging modes.
    uint16_t eventMode : 1;            ///< Report only enabled events.
    uint16_t gestureEvent : 1;         ///< Gesture event enabled.
    uint16_t trackpadEvent : 1;        ///< Trackpad movement event enabled.
    uint16_t atiEvent : 1;             ///< Calibration event enabled.
    uint16_t reserved12 : 1;           ///< Reserved.
    uint16_t alpEvent : 1;             ///< ALP event enabled.
    uint16_t touchEvent : 1; ///< Individual-channel touch event enabled.
    uint16_t reserved15 : 1; ///< Reserved.
  } bits;                    ///< Named fields.
} iqs7211e_config_t;

/** Checked hardware configuration, Table A.12. */
typedef union {
  uint16_t value; ///< Complete register.
  struct {
    uint16_t initialDelay : 2;      ///< Initial cycle delay encoding.
    uint16_t lp1AutoProx : 3;       ///< ALP-only LP1 Auto-Prox encoding.
    uint16_t lp2AutoProx : 3;       ///< ALP-only LP2 Auto-Prox encoding.
    uint16_t countLimit : 2;        ///< Maximum-count encoding.
    uint16_t opampBias : 2;         ///< Amplifier bias encoding.
    uint16_t capacitor : 1;         ///< Internal capacitor selection.
    uint16_t rfFilter : 1;          ///< RF filter enabled.
    uint16_t halfVoltDischarge : 1; ///< Discharge to 0.5 V rather than 0 V.
    uint16_t nmStatic : 1;          ///< NM input static mode.
  } bits;                           ///< Named fields.
} iqs7211e_hardware_t;

/** Checked trackpad configuration, Table A.13. */
typedef union {
  uint16_t value; ///< Complete register.
  struct {
    uint16_t flipX : 1;     ///< Reverse X.
    uint16_t flipY : 1;     ///< Reverse Y.
    uint16_t swapXY : 1;    ///< Swap output axes.
    uint16_t iirFilter : 1; ///< IIR enabled.
    uint16_t staticIIR : 1; ///< Fixed IIR damping instead of dynamic.
    uint16_t mavFilter : 1; ///< Moving average enabled.
    uint16_t reserved : 2;  ///< Reserved.
    uint16_t receivers : 8; ///< Number of logical receivers.
  } bits;                   ///< Named fields.
} iqs7211e_trackpad_t;

/** Checked bit-addressed register for electrode and gesture selections.
 * A named wire layout avoids shifting an unchecked I2C error into a setting.
 */
typedef union {
  uint16_t value; ///< Complete register.
  struct {
    uint16_t b0 : 1;  ///< Bit 0.
    uint16_t b1 : 1;  ///< Bit 1.
    uint16_t b2 : 1;  ///< Bit 2.
    uint16_t b3 : 1;  ///< Bit 3.
    uint16_t b4 : 1;  ///< Bit 4.
    uint16_t b5 : 1;  ///< Bit 5.
    uint16_t b6 : 1;  ///< Bit 6.
    uint16_t b7 : 1;  ///< Bit 7.
    uint16_t b8 : 1;  ///< Bit 8.
    uint16_t b9 : 1;  ///< Bit 9.
    uint16_t b10 : 1; ///< Bit 10.
    uint16_t b11 : 1; ///< Bit 11.
    uint16_t b12 : 1; ///< Bit 12.
    uint16_t b13 : 1; ///< Bit 13.
    uint16_t b14 : 1; ///< Bit 14.
    uint16_t b15 : 1; ///< Bit 15.
  } bits; ///< Bit positions named by the method's enum or electrode index.
} iqs7211e_bits_t;

/** Information word, datasheet v1.6 Table A.3, least significant field first.
 */
typedef struct {
  uint16_t chargingMode : 3; ///< 0 active, 1 idle-touch, 2 idle, 3 LP1, 4 LP2.
  uint16_t atiError : 1;     ///< Trackpad calibration failed.
  uint16_t reATI : 1;        ///< Trackpad calibration just completed.
  uint16_t alpATIError : 1;  ///< ALP calibration failed.
  uint16_t alpReATI : 1;     ///< ALP calibration just completed.
  uint16_t showReset : 1;    ///< Settings must be reloaded after a reset.
  uint16_t fingers : 2;      ///< Number of detected fingers.
  uint16_t movement : 1;     ///< At least one finger is moving.
  uint16_t reserved11 : 1;   ///< Reserved, must not be interpreted.
  uint16_t tooManyFingers : 1; ///< More than the configured finger limit.
  uint16_t reserved13 : 1;     ///< Reserved, must not be interpreted.
  uint16_t alpOutput : 1;      ///< Alternate low-power proximity output.
  uint16_t reserved15 : 1;     ///< Reserved, must not be interpreted.
} iqs7211e_info_t;

/** One tracked finger's little-endian wire record. */
typedef struct {
  uint16_t x;        ///< X in configured coordinate units.
  uint16_t y;        ///< Y in configured coordinate units.
  uint16_t strength; ///< Sum of channel deltas associated with this finger.
  uint16_t area; ///< Number of sensing channels associated with this finger.
} iqs7211e_finger_t;

/** One complete result frame, registers 0x0A through 0x17. */
typedef union {
  uint8_t bytes[28]; ///< Bytes read in one I2C transaction.
  struct {
    int16_t relativeX; ///< Relative movement with one active finger.
    int16_t relativeY; ///< Relative movement with one active finger.
    int16_t gestureX;  ///< Gesture engine's X result.
    int16_t gestureY;  ///< Gesture engine's Y result.
    uint16_t gestures; ///< Gesture flags; not enabled by the default profile.
    iqs7211e_info_t info; ///< Status belonging to these same coordinates.
    iqs7211e_finger_t
        finger[2]; ///< Stable finger slots, not compacted on release.
  } data;          ///< Named wire layout for little-endian Arduino targets.
} iqs7211e_frame_t;
#pragma pack(pop)

static_assert(sizeof(iqs7211e_info_t) == 2, "IQS7211E status layout");
static_assert(sizeof(iqs7211e_frame_t) == 28, "IQS7211E result layout");

/** I2C driver with a default profile for Rev B or repaired Rev A trackpads.
 */
class Adafruit_IQS7211E {
 public:
  Adafruit_IQS7211E();
  ~Adafruit_IQS7211E();
  /** @brief Copying is disabled because this object owns its bus interface.
   * @param other Source object, unused because copying is forbidden. */
  Adafruit_IQS7211E(const Adafruit_IQS7211E& other) = delete;
  /** @brief Assignment is disabled because this object owns its bus interface.
   * @param other Source object, unused because assignment is forbidden.
   * @return No value; this operation is deleted. */
  Adafruit_IQS7211E& operator=(const Adafruit_IQS7211E& other) = delete;

  bool begin(TwoWire* wire = &Wire, int16_t readyPin = -1);
  uint16_t getProductNumber();
  int16_t getFirmwareMajor();
  int16_t getFirmwareMinor();
  bool reset();
  bool calibrate(uint16_t timeoutMS = 3000);
  bool readData();
  uint8_t getNumTouches() const;
  bool isTouched(uint8_t finger = 0) const;
  uint16_t getX(uint8_t finger = 0) const;
  uint16_t getY(uint8_t finger = 0) const;
  uint16_t getTouchStrength(uint8_t finger = 0) const;
  uint16_t getArea(uint8_t finger = 0) const;
  bool tooManyFingers() const;
  iqs7211e_error_t getLastError() const;

  bool setResolution(uint16_t x, uint16_t y);
  uint16_t getXResolution();
  uint16_t getYResolution();
  bool setTouchThresholds(uint8_t set, uint8_t clear);
  uint8_t getTouchSetThreshold();
  uint8_t getTouchClearThreshold();
  bool setReportRate(uint16_t milliseconds,
                     iqs7211e_mode_t mode = IQS7211E_MODE_ACTIVE);
  uint16_t getReportRate(iqs7211e_mode_t mode = IQS7211E_MODE_ACTIVE);
  bool setOrientation(bool flipX, bool flipY, bool swapXY = true);
  bool getFlipX();
  bool getFlipY();
  bool getSwapXY();

  bool beginConfiguration();
  bool endConfiguration();
  bool requestCommunication();

  bool setALPCompensation(iqs7211e_engine_t engine, uint16_t value);
  uint16_t getALPCompensation(iqs7211e_engine_t engine);
  bool setATICoarseDivider(uint8_t value,
                           iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  uint8_t getATICoarseDivider(iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  bool setATICoarseMultiplier(uint8_t value,
                              iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  uint8_t getATICoarseMultiplier(iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  bool setATIFineDivider(uint8_t value,
                         iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  uint8_t getATIFineDivider(iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  bool setATICompensationDivider(uint8_t value,
                                 iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  uint8_t getATICompensationDivider(
      iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  bool setATIDriftLimit(uint8_t counts,
                        iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  uint8_t getATIDriftLimit(iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  bool setATITarget(uint16_t counts,
                    iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  uint16_t getATITarget(iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  bool setMinimumCountReATI(uint16_t counts);
  uint16_t getMinimumCountReATI();
  bool enableAutoATI(bool enabled,
                     iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  bool getAutoATIEnabled(iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  bool queueATI(iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  bool reseed(iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);

  bool setModeTimeout(iqs7211e_mode_t mode, uint16_t seconds);
  uint16_t getModeTimeout(iqs7211e_mode_t mode);
  bool setReferenceUpdateTime(uint8_t seconds);
  uint8_t getReferenceUpdateTime();
  bool setATIRetryTime(uint8_t seconds);
  uint8_t getATIRetryTime();
  bool setCommunicationTimeout(uint16_t milliseconds);
  uint16_t getCommunicationTimeout();
  bool setPowerMode(iqs7211e_mode_t mode);
  iqs7211e_mode_t getPowerMode();
  bool enableManualMode(bool enabled);
  bool getManualModeEnabled();
  bool enableEventMode(bool enabled);
  bool getEventModeEnabled();
  bool enableEvent(iqs7211e_event_t event, bool enabled);
  bool getEventEnabled(iqs7211e_event_t event);
  bool enableWatchdog(bool enabled);
  bool getWatchdogEnabled();
  bool enableCommandTermination(bool enabled);
  bool getCommandTerminationEnabled();
  bool enableCommunicationRequest(bool enabled);
  bool getCommunicationRequestEnabled();
  bool enableTXTest(bool enabled);
  bool getTXTestEnabled();
  bool suspend();

  bool setOscillator(iqs7211e_oscillator_t oscillator);
  iqs7211e_oscillator_t getOscillator();
  bool setOscillatorAdjustment(uint8_t adjustment);
  uint8_t getOscillatorAdjustment();
  bool setALPSensingMethod(iqs7211e_sensing_t method);
  iqs7211e_sensing_t getALPSensingMethod();
  bool enableALPCountFilter(bool enabled);
  bool getALPCountFilterEnabled();
  bool enableALPReceiver(uint8_t pin, bool enabled);
  bool getALPReceiverEnabled(uint8_t pin);
  bool enableALPTransmitter(uint8_t pin, bool enabled);
  bool getALPTransmitterEnabled(uint8_t pin);
  bool setALPThreshold(uint16_t counts);
  uint16_t getALPThreshold();
  bool setALPDebounce(uint8_t setCycles, uint8_t clearCycles);
  uint8_t getALPSetDebounce();
  uint8_t getALPClearDebounce();
  bool setALPCountBeta(iqs7211e_low_power_t mode, uint8_t beta);
  uint8_t getALPCountBeta(iqs7211e_low_power_t mode);
  bool setALPReferenceBeta(iqs7211e_low_power_t mode, uint8_t beta);
  uint8_t getALPReferenceBeta(iqs7211e_low_power_t mode);

  bool setConversionPeriod(uint8_t period,
                           iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  uint8_t getConversionPeriod(iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  bool setConversionFraction(uint8_t fraction,
                             iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  uint8_t getConversionFraction(iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  bool enableNMStatic(bool enabled,
                      iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  bool getNMStaticEnabled(iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  bool enableHalfVoltDischarge(bool enabled,
                               iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  bool getHalfVoltDischargeEnabled(
      iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  bool enableRFFilter(bool enabled,
                      iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  bool getRFFilterEnabled(iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  bool setChargeCapacitor(iqs7211e_capacitor_t capacitor,
                          iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  iqs7211e_capacitor_t getChargeCapacitor(
      iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  bool setOpampBias(iqs7211e_bias_t bias,
                    iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  iqs7211e_bias_t getOpampBias(iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  bool setCountLimit(iqs7211e_count_limit_t limit,
                     iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  iqs7211e_count_limit_t getCountLimit(
      iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  bool setInitialDelay(iqs7211e_init_delay_t cycles,
                       iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  iqs7211e_init_delay_t getInitialDelay(
      iqs7211e_sensor_t sensor = IQS7211E_TRACKPAD);
  bool setAutoProxCycles(iqs7211e_low_power_t mode, iqs7211e_autoprox_t cycles);
  iqs7211e_autoprox_t getAutoProxCycles(iqs7211e_low_power_t mode);

  bool setElectrodeCounts(uint8_t receivers, uint8_t transmitters);
  uint8_t getReceiverCount();
  uint8_t getTransmitterCount();
  bool setMaxTouches(uint8_t touches);
  uint8_t getMaxTouches();
  bool enableMAVFilter(bool enabled);
  bool getMAVFilterEnabled();
  bool enableIIRFilter(bool enabled);
  bool getIIRFilterEnabled();
  bool enableStaticIIR(bool enabled);
  bool getStaticIIREnabled();
  bool setDynamicFilterSpeeds(uint16_t bottom, uint16_t top);
  uint16_t getDynamicFilterBottomSpeed();
  uint16_t getDynamicFilterTopSpeed();
  bool setDynamicFilterBeta(uint8_t beta);
  uint8_t getDynamicFilterBeta();
  bool setStaticFilterBeta(uint8_t beta);
  uint8_t getStaticFilterBeta();
  bool setFingerSplitFactor(uint8_t factor);
  uint8_t getFingerSplitFactor();
  bool setStationaryThreshold(uint8_t distance);
  uint8_t getStationaryThreshold();
  bool setEdgeTrim(iqs7211e_axis_t axis, uint8_t trim);
  uint8_t getEdgeTrim(iqs7211e_axis_t axis);
  bool setSettingsVersion(uint8_t major, uint8_t minor);
  uint8_t getSettingsMajorVersion();
  uint8_t getSettingsMinorVersion();
  bool setElectrodeMapping(const uint8_t mapping[IQS7211E_MAX_ELECTRODES]);
  bool getElectrodeMapping(uint8_t mapping[IQS7211E_MAX_ELECTRODES]);
  bool setSensingCycle(uint8_t cycle, uint8_t channelA, uint8_t channelB);
  uint8_t getSensingCycleChannel(uint8_t cycle, iqs7211e_engine_t engine);

  bool enableGesture(iqs7211e_gesture_t gesture, bool enabled);
  bool getGestureEnabled(iqs7211e_gesture_t gesture);
  bool setTapTime(uint16_t milliseconds);
  uint16_t getTapTime();
  bool setAirTime(uint16_t milliseconds);
  uint16_t getAirTime();
  bool setTapDistance(uint16_t distance);
  uint16_t getTapDistance();
  bool setHoldTime(uint16_t milliseconds);
  uint16_t getHoldTime();
  bool setSwipeTime(uint16_t milliseconds);
  uint16_t getSwipeTime();
  bool setSwipeInitialDistance(iqs7211e_axis_t axis, uint16_t distance);
  uint16_t getSwipeInitialDistance(iqs7211e_axis_t axis);
  bool setSwipeConsecutiveDistance(iqs7211e_axis_t axis, uint16_t distance);
  uint16_t getSwipeConsecutiveDistance(iqs7211e_axis_t axis);
  bool setSwipeAngle(uint8_t tangent);
  uint8_t getSwipeAngle();
  bool setPalmThreshold(uint8_t channels);
  uint8_t getPalmThreshold();
  bool readChannelCounts(uint16_t counts[IQS7211E_CHANNELS]);
  bool readChannelReferences(uint16_t references[IQS7211E_CHANNELS]);
  bool readATICompensation(uint16_t compensation[IQS7211E_CHANNELS]);

 private:
  bool prepareCommunication(bool requireInitialized = true);
  bool finishSetting(bool success, bool changed = false);
  bool argumentError();
  bool endCommunication();
  bool readChannelBank(uint8_t bank, uint16_t* values);
  Adafruit_I2CDevice* _i2c = nullptr;
  int16_t _readyPin = -1;
  bool _initialized = false;
  bool _configuring = false;
  bool _configurationFailed = false;
  bool _commandTermination = true;
  bool _communicationRequest = false;
  bool _validData = false;
  iqs7211e_error_t _error = IQS7211E_ERROR_NOT_INITIALIZED;
  iqs7211e_frame_t _frame = {};
};

#endif
