/*
  NAME:
  gbj_si70

  DESCRIPTION:
  Library for humidity and temperature sensors SI70xx on two-wire (I2C) bus.
  - Library has been inspired by the Adafruit's library Adafruit_SI7021
    but totally redesigned and rewritten.

  LICENSE:
  This program is free software; you can redistribute it and/or modify
  it under the terms of the license GNU GPL v3 http://www.gnu.org/licenses/gpl-3.0.html
  (related to original code) and MIT License (MIT) for added code.

  CREDENTIALS:
  Author: Libor Gabaj
  GitHub: https://github.com/mrkaleArduinoLib/gbj_si70.git

  CREDITS:
  Adafruit
  https://github.com/adafruit/Adafruit_SI7021.git
 */
#ifndef GBJ_SI70_H
#define GBJ_SI70_H
#define GBJ_SI70_VERSION "GBJ_SI70 1.0.0"

#if defined(__AVR__)
  #if ARDUINO >= 100
    #include <Arduino.h>
  #else
    #include <WProgram.h>
  #endif
  #include <inttypes.h>
  #include <Wire.h>
#elif defined(PARTICLE)
  #include <Particle.h>
#endif

#include "gbj_twowire.h"

#define GBJ_SI70_ADDRESS              0x40  // Hardware address
#define GBJ_SI70_CONVERSION_TIME      25    // Time delay needed for measurement conversion in milliseconds

// Commands
#define GBJ_SI70_MEASURE_RH_HOLD      0xE5    // Measure Relative Humidity, Hold Master Mode
#define GBJ_SI70_MEASURE_RH_NOHOLD    0xF5    // Measure Relative Humidity, No Hold Master Mode
#define GBJ_SI70_MEASURE_TEMP_HOLD    0xE3    // Measure Temperature, Hold Master Mode
#define GBJ_SI70_MEASURE_TEMP_NOHOLD  0xF3    // Measure Temperature, No Hold Master Mode
#define GBJ_SI70_READ_TEMP_FROM_RH    0xE0    // Read Temperature Value from Previous RH Measurement
#define GBJ_SI70_RESET                0xFE    // Reset
#define GBJ_SI70_REG_RHT_WRITE        0xE6    // Write RH/T User Register 1
#define GBJ_SI70_REG_RHT_READ         0xE7    // Read RH/T User Register 1
#define GBJ_SI70_REG_HEATER_WRITE     0x51    // Write Heater Control Register
#define GBJ_SI70_REG_HEATER_READ      0x11    // Read Heater Control Register
#define GBJ_SI70_READ_SNA             0xFA0F  // Read Electronic ID 1st Byte
#define GBJ_SI70_READ_SNB             0xFCC9  // Read Electronic ID 2nd Byte
#define GBJ_SI70_READ_FW_REVISION     0x84B8  // Read Firmware Revision
#define GBJ_SI70_LOCK_BYTE_WRITE      0xC556
#define GBJ_SI70_LOCK_BYTE_VALUE      0x00

// Control registers reset settings
#define GBJ_SI70_RST_REG_USER         0x3A    // Reset Settings = 0011_1010 (datasheet Register 1. User Register 1)
#define GBJ_SI70_RST_REG_HEATER       0x00    // Reset Settings = 0000_0000 (datasheet Register 2. Heater Control Register)

// Firmware revision
#define GBJ_SI70_FW_VERSION_10        0xFF    // Firmware version 1.0, A-10 w/o heater control register
#define GBJ_SI70_FW_VERSION_20        0x20    // Firmware version 2.0, A-20 with heater control register

// Silicon Labs device types and revisions
#define GBJ_SI70_TYPE_SAMPLE1         0x00    // Engineering samples
#define GBJ_SI70_TYPE_SAMPLE2         0xFF    // Engineering samples
#define GBJ_SI70_TYPE_7013            0x0D    // Si7013
#define GBJ_SI70_TYPE_7020            0x14    // Si7020
#define GBJ_SI70_TYPE_7021            0x15    // SI70

// Heater current limits in milliampers
#define GBJ_SI70_HEATER_CURRENT_MIN   3.09  // Minimal limit at heater level B0000
#define GBJ_SI70_HEATER_CURRENT_MAX  94.20  // Maximal limit at heater level B1111

// Measurement resolution in bits
#define GBJ_SI70_RES_T14_RH12         14
#define GBJ_SI70_RES_T13_RH10         13
#define GBJ_SI70_RES_T12_RH8          12
#define GBJ_SI70_RES_T11_RH11         11

// Error codes
#define GBJ_SI70_ERR_RESET            255   // Sensor reset failure
#define GBJ_SI70_ERR_FIRMWARE         254   // Firmware revision reading failure
#define GBJ_SI70_ERR_SERIAL_A         253   // Serial number upper double word reading failure
#define GBJ_SI70_ERR_SERIAL_B         252   // Serial number upper double word reading failure
#define GBJ_SI70_ERR_REG_RHT_READ     251   // Reading RH/T User Register 1 failure
#define GBJ_SI70_ERR_REG_HEATER_READ  250   // Reading Heater Control Register failure
#define GBJ_SI70_ERR_MEASURE_RHUM     249   // Measuring relative humidity failure
#define GBJ_SI70_ERR_MEASURE_TEMP     248   // Measuring temperature failure


class gbj_si70 : public gbj_twowire
{
public:
//------------------------------------------------------------------------------
// Public methods
//------------------------------------------------------------------------------


/*
  Initialize two wire bus and sensor with parameters stored by constructor.

  DESCRIPTION:
  The method sanitizes and stores input parameters to the class instance object,
  which determines the operation modus of the sensor.

  PARAMETERS:
  busStop - Flag about releasing the bus after end of data transmission.
            - Data type: boolean
            - Default value: true
            - Limited range: true, false

  RETURN:
  Result code.
*/
  uint8_t begin(bool busStop = true);


/*
  Reset sensor.

  DESCRIPTION:
  The method resets the sensor and sets control registers to their reset settings
  values.

  PARAMETERS: none

  RETURN:
  Result code.
*/
  uint8_t reset();


/*
  Write lock byte.

  DESCRIPTION:
  The method writes the lock byte to the non-volatile memory of a sensor. It
  is enough to do it just once for every sensor.

  PARAMETERS: none

  RETURN:
  Result code.
*/
  uint8_t writeLockByte();


/*
  Measure relative humidity.

  DESCRIPTION:
  The method measures relative humidity as well as temperature, but returns
  the humidity only. The temperature can be returned by another method.

  PARAMETERS:
  holdMasterMode  - Flag about active hold master mode at measuring.
                    - Data type: boolean
                    - Default value: false
                    - Limited range: true, false
  RETURN:
  Relative humidity in per cents or error code GBJ_SI70_ERR_MEASURE_RHUM.
*/
  float measureHumidity(bool holdMasterMode = false);


/*
  Measure relative humidity and retrieve temperature.

  DESCRIPTION:
  The method is overloaded one. It measures either relative humidity solely or
  temperature on the same time and retrives it through input parameter.

  PARAMETERS:
  temperature - Pointer to a float temperature variable.
                - Data type: integer
                - Default value: none
                - Limited range: system specific address space

  holdMasterMode  - Flag about active hold master mode at measuring.
                    - Data type: boolean
                    - Default value: false
                    - Limited range: true, false
  RETURN:
  Relative humidity in per cents and temperature in centigrades or error code
  GBJ_SI70_ERR_MEASURE_RHUM.
*/
  float measureHumidity(float *temperature, bool holdMasterMode = false);


  /*
    Measure temperature.

    DESCRIPTION:
    The method measures temperature only.

    PARAMETERS:
    holdMasterMode  - Flag about active hold master mode at measuring.
                      - Data type: boolean
                      - Default value: false
                      - Limited range: true, false
    RETURN:
    Temperature in centigrade or error code GBJ_SI70_ERR_MEASURE_TEMP.
  */
  float measureTemperature(bool holdMasterMode = false);


//------------------------------------------------------------------------------
// Public setters - they usually return result code.
//------------------------------------------------------------------------------
  uint8_t setHeaterEnabled();     // Turn on sensor's heater
  uint8_t setHeaterDisabled();    // Turn off sensor's heater
  uint8_t setResolutionTemp14();  // Set temperature resolution to 14 bits
  uint8_t setResolutionTemp13();  // Set temperature resolution to 13 bits
  uint8_t setResolutionTemp12();  // Set temperature resolution to 12 bits
  uint8_t setResolutionTemp11();  // Set temperature resolution to 11 bits
  uint8_t setResolutionRhum12();  // Set humidity resolution to 12 bits
  uint8_t setResolutionRhum11();  // Set humidity resolution to 11 bits
  uint8_t setResolutionRhum10();  // Set humidity resolution to 10 bits
  uint8_t setResolutionRhum8();   // Set humidity resolution to 8 bits


/*
  Set measurement resolution for temperature and relative humidity at once.

  DESCRIPTION:
  The method sets the bit resolution by input parameter.
  The resolution is determined by corresponding macro but in fact it is the bit
  resolution for temperature.

  PARAMETERS:
  resolution  - Desired measurement resolution
              - Data type: non-negative integer
              - Default value: GBJ_SI70_RES_T14_RH12
              - Limited range: GBJ_SI70_RES_T11_RH11 ~ GBJ_SI70_RES_T14_RH12

  RETURN:
  Result code.
*/
  uint8_t setResolution(uint8_t resolution = GBJ_SI70_RES_T14_RH12);


/*
  Set heater level if the heater control register is enabled.

  DESCRIPTION:
  The method sets the heater level if the heater is enabled and determines by it
  the heating current. The heater level is determined by lower 4 bits in the
  heater control register.

  PARAMETERS:
  heaterLevel - Desired heater level.
              - Data type: non-negative integer
              - Default value: 0x0
              - Limited range: 0x0 ~ 0xF

  RETURN:
  Result code.
*/
  uint8_t setHeaterLevel(uint8_t heaterLevel = 0x0);  // Set heater level in range 0x0 ~ 0xF


//------------------------------------------------------------------------------
// Public getters
//------------------------------------------------------------------------------
  bool     getVddStatus();          // Flag about correct operating voltage
  //
  bool     getHeaterEnabled();      // Flag about enabling the sensor's heater
  uint8_t  getHeaterLevel();        // Heater level
  float    getHeaterCurrent();      // Heater current in milliampers
  //
  uint32_t getSerialUpper();        // Serial number upper double word
  uint32_t getSerialLower();        // Serial number lower double word
  //
  uint8_t  getFwRevision();         // Firmware revision
  uint8_t  getDeviceType();         // Device type identification
  uint8_t  getResolutionTemp();     // Temperature resolution in bits
  uint8_t  getResolutionRhum();     // Relative humidity resolution in bits


private:
//------------------------------------------------------------------------------
// Private attributes
//------------------------------------------------------------------------------
  bool     _userRegEnabled;   // Flag about initialization (reading) the user register
  bool     _heaterRegEnabled; // Flag about initialization (reading) the heater register
  uint8_t  _tempResolution[4] = {14, 12, 13, 11}; // List of temperature resolutions
  uint8_t  _rhumResolution[4] = {12,  8, 10, 11}; // List of humidity resolutions
  uint8_t  _fwRevision;       // Firmware revision
  uint8_t  _userRegister;     // User register 1
  uint8_t  _heaterRegister;   // Heater register
  uint8_t  _heaterLevel;      // Heater level as 4 lower bytes in the register
  uint32_t _serialSNA;        // Upper 4 bytes of serial number
  uint32_t _serialSNB;        // Lower 4 bytes of serial number


//------------------------------------------------------------------------------
// Private methods
//------------------------------------------------------------------------------

/*
  Calculate CRC8 checksum for 32-bit integer

  DESCRIPTION:
  The method calculates CRC8 checksum of long integer by 9-bit polynomial
  x^8+x^5+x^4+x^0 (0x131) with initialization 0x00 according to the datasheet.

  PARAMETERS:
  data  - Number for which the CRC to be calculated
          - Data type: non-negative integer
          - Default value: none
          - Limited range: 0 ~ 2^32 - 1

  RETURN:
  CRC8 checksum.
*/
  uint8_t crc8(uint32_t data);


/*
  Validate 32-bit integer by provided CRC8 checksum.

  DESCRIPTION:
  The method checks whether provided CRC8 checksum is valid for input long
  integer.

  PARAMETERS:
  data  - Number for which the CRC to be calculated
          - Data type: non-negative integer
          - Default value: none
          - Limited range: 0 ~ 2^32 - 1

  crc   - Expected CRC8 checksum.
          - Data type: non-negative integer
          - Default value: none
          - Limited range: 0 ~ 255

  RETURN:
  Flag about correct checksum.
*/
  bool checkCrc8(uint32_t data, uint8_t crc);


/*
  Calculate resolution code from user register byte.

  DESCRIPTION:
  The method calculates resolution code from D7 and D0 bit of the user register
  byte.

  PARAMETERS: none

  RETURN:
  Resolution code (0 ~ 3).
*/
  uint8_t resolution();


/*
  Read device revision.

  DESCRIPTION:
  The method determines the device revision by presence of the heater control
  register. It write to it a non-zero value, reads it back, and compares written
  and read value.

  PARAMETERS: none

  RETURN:
  Result code.
*/
  uint8_t readDeviceRevision();


/*
  Read firmware revision byte.

  DESCRIPTION:
  The method reads firmware revision byte from the sensor and stores it in the
  class instance object.

  PARAMETERS: none

  RETURN:
  Result code.
*/
  uint8_t readFwRevision();


/*
  Read electronic serial number.

  DESCRIPTION:
  The method reads all bytes constituting a serial number, checks them with
  crc codes read from the sensor, and stores it in the class instance object.

  PARAMETERS: none

  RETURN:
  Result code.
*/
  uint8_t readSerialNumber();


/*
  Read user register.

  DESCRIPTION:
  The method reads the user register byte and parses relevant data to the
  individual items. Then all of them stores in the class instance object.

  PARAMETERS: none

  RETURN:
  Result code.
*/
  uint8_t readUserRegister();


/*
  Write user register.

  DESCRIPTION:
  The method writes the user register byte stored in the class instance object
  to the user register.

  PARAMETERS: none

  RETURN:
  Result code.
*/
  uint8_t writeUserRegister();


/*
  Read heater register.

  DESCRIPTION:
  The method reads the heater register byte and parses relevant data to the
  heater level. Then stores it in the class instance object.

  PARAMETERS: none

  RETURN:
  Result code.
*/
  uint8_t readHeaterRegister();


/*
  Write heater register.

  DESCRIPTION:
  The method writes the heater register byte stored in the class instance object
  to the heater register.

  PARAMETERS: none

  RETURN:
  Result code.
*/
  uint8_t writeHeaterRegister();


/*
  Set sensor's heater status.

  DESCRIPTION:
  The method sets the heater status bit HTRE (D2) in the user register byte.

  PARAMETERS:
  status  - Desired flag about heater enabling
          - Data type: bool
          - Default value: none
          - Limited range: true, false

  RETURN:
  Result code.
*/
  uint8_t setHeaterStatus(bool status);


/*
  Set sensor's resolution.

  DESCRIPTION:
  The method sets the resolution bits RES1 (D7) and RES0 (D0) in the user
  register byte.

  PARAMETERS:
  bitRes1 - Desired flag about setting upper resolution bit (true == 1, false == 0)
            - Data type: bool
            - Default value: none
            - Limited range: true, false

  bitRes0 - Desired flag about setting lower resolution bit (true == 1, false == 0)
            - Data type: bool
            - Default value: none
            - Limited range: true, false

  RETURN:
  Result code.
*/
  uint8_t setBitResolution(bool bitRes1, bool bitRes0);


/*
  Measure or retrieve temperature.

  DESCRIPTION:
  The method measures or retrives temperature from the sensor.

  PARAMETERS:
  command - Reading command for the temperature.
            - Data type: non-negative integer
            - Default value: none
            - Limited range: 0 ~ 0xFF
  RETURN:
  Temperature in centigrade or error code GBJ_SI70_ERR_MEASURE_TEMP.
*/
  float readTemperature(uint8_t command);


/*
  Calculate temperature.

  DESCRIPTION:
  The method wraps a formula for calculating temperature in centigrade from
  16-bit word.

  PARAMETERS:
  wordMeasure - Measured binary word.
                - Data type: integer
                - Default value: none
                - Limited range: 0 ~ 0xFFFF
  RETURN:
  Temperature in centigrade.
*/
  float calculateTemperature(uint16_t wordMeasure);


/*
  Calculate relative humidity.

  DESCRIPTION:
  The method wraps a formula for calculating relative humidity in per-cents
  from 16-bit word.

  PARAMETERS:
  wordMeasure - Measured binary word.
                - Data type: integer
                - Default value: none
                - Limited range: 0 ~ 0xFFFF
  RETURN:
  Relative humidity in per-cents.
*/
  float calculateHumidity(uint16_t wordMeasure);

};

#endif
