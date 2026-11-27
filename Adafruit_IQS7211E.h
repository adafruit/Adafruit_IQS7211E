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

/** Outcome of the most recent acquisition or initialization. */
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
/** Checked system-command snapshot; only TP ATI is inspected here. */
typedef union {
  uint16_t value; ///< Register 0x33, little endian.
  struct {
    uint16_t lowerCommands : 5; ///< Mode, reseed and reserved fields.
    uint16_t trackpadATI : 1;   ///< Clears when trackpad calibration completes.
    uint16_t upperCommands : 10; ///< Remaining independent command fields.
  } bits;                        ///< Named command fields for the checked read.
} iqs7211e_system_t;

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
  bool setReportRate(uint16_t milliseconds);
  uint16_t getReportRate();
  bool setOrientation(bool flipX, bool flipY, bool swapXY = true);
  bool readChannelCounts(uint16_t counts[IQS7211E_CHANNELS]);
  bool readChannelReferences(uint16_t references[IQS7211E_CHANNELS]);
  bool readATICompensation(uint16_t compensation[IQS7211E_CHANNELS]);

 private:
  bool loadSettings();
  bool endCommunication();
  bool readChannelBank(uint8_t bank, uint16_t* values);
  Adafruit_I2CDevice* _i2c = nullptr;
  int16_t _readyPin = -1;
  bool _initialized = false;
  bool _validData = false;
  iqs7211e_error_t _error = IQS7211E_ERROR_NOT_INITIALIZED;
  iqs7211e_frame_t _frame = {};
};

#endif
