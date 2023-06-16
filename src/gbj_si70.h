/*
  NAME:
  gbjSI70

  DESCRIPTION:
  Library for humidity and temperature sensors SI70xx on two-wire (I2C) bus.

  LICENSE:
  This program is free software; you can redistribute it and/or modify
  it under the terms of the license GNU GPL v3
  http://www.gnu.org/licenses/gpl-3.0.html (related to original code) and MIT
  License (MIT) for added code.

  CREDENTIALS:
  Author: Libor Gabaj
  GitHub: https://github.com/mrkaleArduinoLib/gbj_si70.git
 */
#ifndef GBJ_SI70_H
#define GBJ_SI70_H

#include "gbj_twowire.h"

class gbj_si70 : public gbj_twowire
{
public:
  enum DeviceTypes : uint8_t
  {
    // Engineering samples
    TYPE_SAMPLE1 = 0x00,
    // Engineering samples
    TYPE_SAMPLE2 = 0xFF,
    // Si7013
    TYPE_7013 = 0x0D,
    // Si7020
    TYPE_7020 = 0x14,
    // SI70
    TYPE_7021 = 0x15,
  };
  enum FirmwareVersions : uint8_t
  {
    // Firmware version 1.0, A-10 without heater control register
    FW_VERSION_10 = 0xFF,
    // Firmware version 2.0, A-20 with heater control register
    FW_VERSION_20 = 0x20,
  };

  gbj_si70(ClockSpeeds clockSpeed = ClockSpeeds::CLOCK_100KHZ,
           uint8_t pinSDA = 4,
           uint8_t pinSCL = 5)
    : gbj_twowire(clockSpeed, pinSDA, pinSCL){};

  /*
    Initialize sensor.

    DESCRIPTION:
    The method sanitizes and stores input parameters to the class instance
    object, which determines the operation modus of the sensor.

    PARAMETERS:
    holdMasterMode - Flag about blocking (holding) serial clock line during
    measurement. At no holding master mode other communication on the bus can be
    performed.
      - Data type: boolean
      - Default value: false
      - Limited range: true, false

    RETURN: Result code
  */
  inline ResultCodes begin(bool holdMasterMode = true)
  {
    if (isError(gbj_twowire::begin()))
    {
      return getLastResult();
    }
    if (isError(setAddress(Addresses::ADDRESS)))
    {
      return getLastResult();
    }
    setUseValuesMax();
    setHoldMasterMode(holdMasterMode);
    userReg_.value = 0;
    if (isError(reset()))
    {
      return getLastResult();
    }
    bool origBusStop = getBusStop();
    setBusRepeat();
    if (isError(readFwRevision()))
    {
      return getLastResult();
    }
    setBusStopFlag(origBusStop);
    return readSerialNumber();
  }

  /*
    Reset sensor.

    DESCRIPTION:
    The method resets the sensor and sets control registers to their reset
    settings values.

    PARAMETERS: none

    RETURN: Result code
  */
  inline ResultCodes reset()
  {
    if (isError(busSend(Commands::CMD_RESET)))
    {
      return getLastResult();
    }
    // Power-up reset
    if (userReg_.value == 0)
    {
      wait(getUseValuesTyp() ? Timing::TIMING_POWERUP_TYP
                             : Timing::TIMING_POWERUP_MAX);
    }
    // Software reset
    else
    {
      wait(getUseValuesTyp() ? Timing::TIMING_RESET_TYP
                             : Timing::TIMING_RESET_MAX);
    }
    // Check user register 1 reset value
    if (isError(readUserRegister()))
    {
      return getLastResult();
    }
    if (userReg_.value != Resetting::RESET_REG_USER)
    {
      return setLastResult(static_cast<ResultCodes>(ResultCodes::ERROR_RESET));
    }
    // Turn off heater
    return setHeaterDisabled();
  }

  /*
    Write lock byte.

    DESCRIPTION:
    The method writes the lock byte to the non-volatile memory of a sensor. It
    is enough to do it just once for every sensor.

    PARAMETERS: none

    RETURN: Result code
  */
  inline ResultCodes writeLockByte()
  {
    bool origBusStop = getBusStop();
    setBusRepeat();
    if (isError(busSend(Commands::CMD_LOCK_BYTE_WRITE)))
    {
      return getLastResult();
    }
    setBusStopFlag(origBusStop);
    return busSend(Commands::CMD_LOCK_BYTE_VALUE);
  }

  /*
    Measure relative humidity or retrieve temperature as well.

    DESCRIPTION:
    The particular method measures either relative humidity solely or
    temperature on the same time and retrieves it through input parameter.

    PARAMETERS:
    temperature - Referenced variable for placing a temperature value.
      - Data type: float
      - Default value: none
      - Limited range: sensor specific

    RETURN: Relative humidity in per cents and temperature in centigrades or
    bad measure value. Methods set error code as well.
  */
  float measureHumidity();
  inline float measureHumidity(float &temperature)
  {
    bool origBusStop = getBusStop();
    setBusRepeat();
    float humidity = measureHumidity();
    setBusStopFlag(origBusStop);
    temperature = readTemperature(Commands::CMD_READ_TEMP_FROM_RH);
    return humidity;
  }

  /*
    Measure temperature.

    DESCRIPTION:
    The method measures temperature only.

    PARAMETERS: none

    RETURN: Temperature in centigrades or erroneous value PARAM_BAD_RHT. Methods
    set error code as well.
  */
  inline float measureTemperature()
  {
    return getHoldMasterMode()
             ? readTemperature(Commands::CMD_MEASURE_TEMP_HOLD)
             : readTemperature(Commands::CMD_MEASURE_TEMP_NOHOLD);
  }

  // Setters
  inline void setUseValuesTyp() { status_.useValuesTyp = true; }
  inline void setUseValuesMax() { status_.useValuesTyp = false; };
  // Turn on sensor's heater
  inline ResultCodes setHeaterEnabled() { return setHeaterStatus(true); }
  // Turn off sensor's heater
  inline ResultCodes setHeaterDisabled() { return setHeaterStatus(false); }
  // Set resolution
  inline ResultCodes setResolutionTemp14()
  {
    return setBitResolution(false, false);
  }
  inline ResultCodes setResolutionTemp13()
  {
    return setBitResolution(true, false);
  }
  inline ResultCodes setResolutionTemp12()
  {
    return setBitResolution(false, true);
  }
  inline ResultCodes setResolutionTemp11()
  {
    return setBitResolution(true, true);
  }
  inline ResultCodes setResolutionRhum12() { return setResolutionTemp14(); }
  inline ResultCodes setResolutionRhum10() { return setResolutionTemp13(); }
  inline ResultCodes setResolutionRhum11() { return setResolutionTemp11(); }
  inline ResultCodes setResolutionRhum8() { return setResolutionTemp12(); }
  //
  inline void setHoldMasterMode(bool holdMasterMode)
  {
    status_.holdMasterMode = holdMasterMode;
  }

  /*
    Set heater level if the heater control register is enabled.

    DESCRIPTION:
    The method sets the heater level if the heater is enabled and determines by
    it the heating current. The heater level is determined by lower 4 bits in
    the heater control register.

    PARAMETERS:
    heaterLevel - Desired heater level.
      - Data type: non-negative integer
      - Default value: none
      - Limited range: 0x00 ~ 0x0F

    RETURN: Result code
  */
  inline ResultCodes setHeaterLevel(uint8_t heaterLevel)
  {
    if (!heater_.enabled && isError(readHeaterRegister()))
    {
      return getLastResult();
    }
    // Write heater level for Heater [3:0] bits of heater control register byte
    if (getHeaterLevel() != heaterLevel)
    {
      // Reset heater level
      heater_.regValue &= B11110000;
      // Set heater level
      heater_.regValue |= heaterLevel;
      if (isError(writeHeaterRegister()))
      {
        return getLastResult();
      }
    }
    return getLastResult();
  }

  // Getters
  inline uint32_t getSNA() { return status_.serialSNA; }
  inline uint32_t getSNB() { return status_.serialSNB; }
  inline uint64_t getSerialNumber()
  {
    uint64_t serial;
    serial = status_.serialSNA;
    serial <<= 32;
    serial |= status_.serialSNB;
    return serial;
  }
  inline FirmwareVersions getFwRevision()
  {
    return static_cast<FirmwareVersions>(status_.fwRevision);
  }
  inline DeviceTypes getDeviceType()
  {
    return static_cast<DeviceTypes>(status_.serialSNB >> 24);
  }
  inline bool getHoldMasterMode() { return status_.holdMasterMode; }

  // Flag about correct operating voltage
  inline bool getVddStatus()
  {
    // Always read user register, because the voltage status is updated after
    // each measurement
    if (isError(readUserRegister()))
    {
      return false;
    }
    uint8_t status = (userReg_.value >> 6) & B1;
    return (status == 0);
  }

  // Flag about enabling the sensor's heater
  inline bool getHeaterEnabled()
  {
    if (isError(reloadUserRegister()))
    {
      return false;
    }
    // Separate heater status from HTRE (D2) bit of user register byte
    uint8_t status = (userReg_.value >> 2) & B1;
    return (status == 1);
  }

  // Heater current in milliampers
  inline float getHeaterCurrent()
  {
    uint8_t heaterLevel = getHeaterLevel();
    return isSuccess()
             ? (heaterLevel / 0xF) * (heater_.currentMax - heater_.currentMin) +
                 heater_.currentMin
             : 0.0;
  }

  // Heater level
  inline uint8_t getHeaterLevel()
  {
    if (!heater_.enabled && isError(readHeaterRegister()))
    {
      return getLastResult();
    }
    return heater_.regValue & B1111;
  }

  // Temperature resolution in bits
  inline uint8_t getResolutionTemp()
  {
    uint8_t resIdx = resolution();
    return resolusion_.tempBits[isSuccess(reloadUserRegister()) ? resIdx : 0];
  }

  // Relative humidity resolution in bits
  inline uint8_t getResolutionRhum()
  {
    uint8_t resIdx = resolution();
    return resolusion_.rhumBits[isSuccess(reloadUserRegister()) ? resIdx : 0];
  }

  // Bad measurement value
  inline float getErrorRHT()
  {
    return static_cast<float>(Params::PARAM_BAD_RHT);
  }

private:
  enum Addresses
  {
    // Hardware address
    ADDRESS = 0x40,
  };
  enum Commands : uint16_t
  {
    // Measure Relative Humidity, Hold Master Mode
    CMD_MEASURE_RH_HOLD = 0xE5,
    // Measure Relative Humidity, No Hold Master Mode
    CMD_MEASURE_RH_NOHOLD = 0xF5,
    // Measure Temperature, Hold Master Mode
    CMD_MEASURE_TEMP_HOLD = 0xE3,
    // Measure Temperature, No Hold Master Mode
    CMD_MEASURE_TEMP_NOHOLD = 0xF3,
    // Read Temperature Value from Previous RH Measurement
    CMD_READ_TEMP_FROM_RH = 0xE0,
    // Soft reset
    CMD_RESET = 0xFE,
    // Write RH/T User Register 1
    CMD_REG_RHT_WRITE = 0xE6,
    // Read RH/T User Register 1
    CMD_REG_RHT_READ = 0xE7,
    // Write Heater Control Register
    CMD_REG_HEATER_WRITE = 0x51,
    // Read Heater Control Register
    CMD_REG_HEATER_READ = 0x11,
    // Read Electronic ID 1st Byte
    CMD_READ_SNA = 0xFA0F,
    // Read Electronic ID 2nd Byte
    CMD_READ_SNB = 0xFCC9,
    // Read Firmware Revision
    CMD_READ_FW_REVISION = 0x84B8,
    CMD_LOCK_BYTE_WRITE = 0xC556,
    CMD_LOCK_BYTE_VALUE = 0x00,
  };
  // Resolutions in bits
  enum Resolutions : uint8_t
  {
    RESOLUTION_T14_RH12 = 14,
    RESOLUTION_T13_RH10 = 13,
    RESOLUTION_T12_RH8 = 12,
    RESOLUTION_T11_RH11 = 11,
  };
  enum Timing : uint8_t
  {
    TIMING_POWERUP_MAX = 80,
    TIMING_POWERUP_TYP = 80,
    TIMING_RESET_MAX = 15,
    TIMING_RESET_TYP = 5,
  };
  // Control registers reset settings
  enum Resetting : uint8_t
  {
    // Reset Settings = 0011_1010 (datasheet Register 1. User Register 1)
    RESET_REG_USER = 0x3A,
    // Reset Settings = 0000_0000 (datasheet Register 2. Heater Control
    // Register)
    RESET_REG_HEATER = 0x00,
  };
  enum Params : uint8_t
  {
    // Number of repeating action at wrong CRC
    PARAM_CRC_CHECKS = 3,
    // Unreasonable wrong relative humidity or temperature value
    PARAM_BAD_RHT = 255,
  };
  struct Status
  {
    // Firmware revision
    uint8_t fwRevision;
    // Upper 4 bytes of serial number
    uint32_t serialSNA;
    // Lower 4 bytes of serial number
    uint32_t serialSNB;
    // Flag about active hold master mode at measuring
    bool holdMasterMode;
    // Flag about using typical values from datasheet
    bool useValuesTyp;
  } status_;
  // Parameters of user register
  struct UserReg
  {
    // Flag about initialization (reading) the user register
    bool read;
    // Value of user register 1
    uint8_t value;
  } userReg_;
  // Params for sensor heater
  struct Heater
  {
    // Flag about initialization (reading) the heater register
    bool enabled;
    // Heater register value
    uint8_t regValue;
    // Heater level as 4 lower bits in the register
    uint8_t level;
    // Minimal limit at heater level B0000
    float currentMin = 3.09;
    // Maximal limit at heater level B1111
    float currentMax = 94.20;
  } heater_;
  // Indexes by resolution bits D7 and D0 value in user register
  struct Resolution
  {
    // Temperature resolutions in bits
    uint8_t tempBits[4] = {
      14,
      12,
      13,
      11,
    };
    // Humidity resolutions in bits
    uint8_t rhumBits[4] = {
      12,
      8,
      10,
      11,
    };
    // Maximal conversion times of temperature in milliseconds
    uint8_t tempConvTimeMax[4] = {
      11,
      4,
      6,
      3,
    };
    // Typical conversion times of temperature in milliseconds
    uint8_t tempConvTimeTyp[4] = {
      7,
      3,
      4,
      2,
    };
    // Maximal conversion times of humidity in milliseconds
    uint8_t rhumConvTimeMax[4] = {
      12,
      3,
      5,
      7,
    };
    // Typical conversion times of humidity in milliseconds
    uint8_t rhumConvTimeTyp[4] = {
      10,
      3,
      4,
      6,
    };
  } resolusion_;
  inline bool getUseValuesTyp() { return status_.useValuesTyp; }

  inline uint8_t getConversionTimeTempTyp()
  {
    return resolusion_
      .tempConvTimeTyp[isSuccess(reloadUserRegister()) ? resolution() : 0];
  }
  inline uint8_t getConversionTimeTempMax()
  {
    return resolusion_
      .tempConvTimeMax[isSuccess(reloadUserRegister()) ? resolution() : 0];
  }
  inline uint8_t getConversionTimeTemp()
  {
    return getUseValuesTyp() ? getConversionTimeTempTyp()
                             : getConversionTimeTempMax();
  }

  inline uint8_t getConversionTimeRhumTyp()
  {
    return resolusion_
      .rhumConvTimeTyp[isSuccess(reloadUserRegister()) ? resolution() : 0];
  }
  inline uint8_t getConversionTimeRhumMax()
  {
    return resolusion_
      .rhumConvTimeMax[isSuccess(reloadUserRegister()) ? resolution() : 0];
  }
  inline uint8_t getConversionTimeRhum()
  {
    return getUseValuesTyp() ? getConversionTimeRhumTyp()
                             : getConversionTimeRhumMax();
  }

  /*
    Validate byte array by CRC

    DESCRIPTION:
    The method checks whether provided CRC8 checksum is valid for input byte
    array.
    - The method utilizes CRC8 checksum with polynom x^8+x^5+x^4+1.
    - For 2 LSB items (bytes) of provided array the CRC is calculated and
    compared to the 3rd item.

    PARAMETERS:
    byteArray - Pointer to an array of bytes
      - Data type: pointer
      - Default value: none
      - Limited range: none

    byteCnt - Number of bytes to be checked
      - Data type: non-negative integer
      - Default value: 2
      - Limited range: 0 ~ 255

    RETURN: Flag about correct checksum
  */
  inline bool checkCrc8(uint8_t *byteArray, uint8_t byteCnt = 2)
  {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < byteCnt; i++)
    {
      crc ^= byteArray[i];
      for (int8_t b = 7; b >= 0; b--)
      {
        if (crc & 0x80)
        {
          crc = (crc << 1) ^ 0x31;
        }
        else
        {
          crc = (crc << 1);
        }
      }
    }
    return crc == byteArray[byteCnt];
  }

  /*
    Calculate resolution code from user register byte.

    DESCRIPTION:
    The method calculates resolution code from D7 and D0 bit of the user
    register byte.

    PARAMETERS: none

    RETURN: Resolution code (0 ~ 3).
  */
  inline uint8_t resolution()
  {
    // Separate resolution from RES1 (D7) and RES0 (D0) bit of user register
    uint8_t res0 = (userReg_.value >> 0) & B1;
    uint8_t res1 = (userReg_.value >> 7) & B1;
    return (res1 << 1) | res0;
  }

  /*
    Read device revision.

    DESCRIPTION:
    The method determines the device revision by presence of the heater control
    register. It write to it a non-zero value, reads it back, and compares
    written and read value.

    PARAMETERS: none

    RETURN: Result code
  */
  inline ResultCodes readDeviceRevision()
  {
    bool origBusStop = getBusStop();
    setBusRepeat();
    if (isError(busSend(Commands::CMD_READ_FW_REVISION)))
    {
      return setLastResult(ResultCodes::ERROR_FIRMWARE);
    }
    uint8_t data[1];
    setBusStopFlag(origBusStop);
    if (isError(busReceive(data, sizeof(data) / sizeof(data[0]))))
    {
      return setLastResult(ResultCodes::ERROR_FIRMWARE);
    }
    status_.fwRevision = data[0];
    return getLastResult();
  }

  /*
    Read firmware revision byte.

    DESCRIPTION:
    The method reads firmware revision byte from the sensor and stores it in the
    class instance object.

    PARAMETERS: none

    RETURN: Result code
  */
  inline ResultCodes readFwRevision()
  {
    bool origBusStop = getBusStop();
    setBusRepeat();
    if (isError(busSend(Commands::CMD_READ_FW_REVISION)))
    {
      return setLastResult(ResultCodes::ERROR_FIRMWARE);
    }
    uint8_t data[1];
    setBusStopFlag(origBusStop);
    if (isError(busReceive(data, sizeof(data) / sizeof(data[0]))))
    {
      return setLastResult(ResultCodes::ERROR_FIRMWARE);
    }
    status_.fwRevision = data[0];
    return getLastResult();
  }

  /*
    Read electronic serial number.

    DESCRIPTION:
    The method reads all bytes constituting a serial number, checks them with
    crc codes read from the sensor, and stores it in the class instance object.

    PARAMETERS: none

    RETURN: Result code
  */
  ResultCodes readSerialNumber();

  /*
    Read user register.

    DESCRIPTION:
    The method reads the user register byte and parses relevant data to the
    individual items. Then all of them stores in the class instance object.

    PARAMETERS: none

    RETURN: Result code
  */
  inline ResultCodes readUserRegister()
  {
    bool origBusStop = getBusStop();
    setBusRepeat();
    if (isError(busSend(Commands::CMD_REG_RHT_READ)))
    {
      return setLastResult(ResultCodes::ERROR_REGISTER);
    }
    uint8_t data[1];
    setBusStopFlag(origBusStop);
    if (isError(busReceive(data, sizeof(data) / sizeof(data[0]))))
    {
      return setLastResult(ResultCodes::ERROR_REGISTER);
    }
    userReg_.value = data[0];
    userReg_.read = true;
    return getLastResult();
  }

  /*
    Read user register if needed.

    DESCRIPTION:
    The method reads the user register if internal flag is reset.

    PARAMETERS: none

    RETURN: Result code
  */
  inline ResultCodes reloadUserRegister()
  {
    return userReg_.read ? getLastResult() : readUserRegister();
  }

  /*
    Write user register.

    DESCRIPTION:
    The method writes the user register byte stored in the class instance object
    to the user register.

    PARAMETERS: none

    RETURN: Result code
  */
  inline ResultCodes writeUserRegister()
  {
    if (isError(busSend(Commands::CMD_REG_RHT_WRITE, userReg_.value)))
    {
      return getLastResult();
    }
    // Reread the user register the next time for sure
    userReg_.read = false;
    return getLastResult();
  }

  /*
    Read heater register.

    DESCRIPTION:
    The method reads the heater register byte and parses relevant data to the
    heater level. Then stores it in the class instance object.

    PARAMETERS: none

    RETURN: Result code
  */
  inline ResultCodes readHeaterRegister()
  {
    bool origBusStop = getBusStop();
    setBusRepeat();
    if (isError(busSend(Commands::CMD_REG_HEATER_READ)))
    {
      return setLastResult(ResultCodes::ERROR_REGISTER);
    }
    uint8_t data[1];
    setBusStopFlag(origBusStop);
    if (isError(busReceive(data, sizeof(data) / sizeof(data[0]))))
    {
      return setLastResult(ResultCodes::ERROR_REGISTER);
    }
    heater_.regValue = data[0];
    heater_.enabled = true;
    return getLastResult();
  }

  /*
    Write heater register.

    DESCRIPTION:
    The method writes the heater register byte stored in the class instance
    object to the heater register.

    PARAMETERS: none

    RETURN: Result code
  */
  inline ResultCodes writeHeaterRegister()
  {
    if (isError(busSend(CMD_REG_HEATER_WRITE, heater_.regValue)))
    {
      return getLastResult();
    }
    // Reread the heater register the next time for sure
    heater_.enabled = false;
    return getLastResult();
  }

  /*
    Set sensor's heater status.

    DESCRIPTION:
    The method sets the heater status bit HTRE (D2) in the user register byte.

    PARAMETERS:
    status - Desired flag about heater enabling
      - Data type: bool
      - Default value: none
      - Limited range: true, false

    RETURN: Result code
  */
  inline ResultCodes setHeaterStatus(bool status)
  {
    if (isError(reloadUserRegister()))
    {
      return getLastResult();
    }
    // Write heater status for HTRE (D2) bit of user register byte if needed
    if (!getHeaterEnabled() && status)
    {
      // Set HTRE to 1
      userReg_.value |= B00000100;
      return writeUserRegister();
    }
    if (getHeaterEnabled() && !status)
    {
      // Set HTRE to 0
      userReg_.value &= B11111011;
      return writeUserRegister();
    }
    return getLastResult();
  }

  /*
    Set sensor's resolution.

    DESCRIPTION:
    The method sets the resolution bits RES1 (D7) and RES0 (D0) in the user
    register byte.

    PARAMETERS:
    bitRes1 - Desired flag about setting upper resolution bit (true == 1, false
    == 0)
      - Data type: bool
      - Default value: none
      - Limited range: true, false

    bitRes0 - Desired flag about setting lower resolution bit (true == 1, false
    == 0)
      - Data type: bool
      - Default value: none
      - Limited range: true, false

    RETURN: Result code
  */
  inline ResultCodes setBitResolution(bool bitRes1, bool bitRes0)
  {
    if (isError(reloadUserRegister()))
    {
      return getLastResult();
    }
    // Determine resolution code
    uint8_t code = ((bitRes1 ? B1 : B0) << 1) | (bitRes0 ? B1 : B0);
    // Write resolution bits RES1 (D7) and RES0 (D0) to user register byte
    if (resolution() != code)
    {
      // Set RES0
      if (bitRes0)
      {
        // Set RES0 to 1
        userReg_.value |= B00000001;
      }
      else
      {
        // Set RES0 to 0
        userReg_.value &= B11111110;
      }
      // Set RES1
      if (bitRes1)
      {
        // Set RES1 to 1
        userReg_.value |= B10000000;
      }
      else
      {
        // Set RES1 to 0
        userReg_.value &= B01111111;
      }
      return writeUserRegister();
    }
    return getLastResult();
  }

  /*
    Measure or retrieve temperature.

    DESCRIPTION:
    The method measures or retrieves temperature from the sensor.

    PARAMETERS:
    command - Reading command for the temperature.
      - Data type: non-negative integer
      - Default value: none
      - Limited range: 0x00 ~ 0xFF

    RETURN: Temperature in centigrade or bad measurement value.
  */
  float readTemperature(Commands command);

  /*
    Calculate temperature.

    DESCRIPTION:
    The method wraps a formula for calculating temperature in centigrade from
    16-bit word.

    PARAMETERS:
    wordMeasure - Measured binary word.
      - Data type: integer
      - Default value: none
      - Limited range: 0x0000 ~ 0xFFFF

    RETURN: Temperature in centigrade.
  */
  inline float calculateTemperature(uint16_t wordMeasure)
  {
    float temperature = static_cast<float>(wordMeasure);
    temperature *= 175.72;
    temperature /= 65536.0;
    temperature -= 46.85;
    return temperature;
  }

  /*
    Calculate relative humidity.

    DESCRIPTION:
    The method wraps a formula for calculating relative humidity in per-cents
    from 16-bit word.

    PARAMETERS:
    wordMeasure - Measured binary word.
      - Data type: integer
      - Default value: none
      - Limited range: 0x0000 ~ 0xFFFF

    RETURN: Relative humidity in per-cents truncated to range 0 - 100 °C.
  */
  inline float calculateHumidity(uint16_t wordMeasure)
  {
    float humidity = static_cast<float>(wordMeasure);
    humidity *= 125.0;
    humidity /= 65536.0;
    humidity -= 6.0;
    return constrain(humidity, 0.0, 100.0);
  }
};

#endif
