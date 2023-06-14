#include "gbj_si70.h"

float gbj_si70::readTemperature(Commands command)
{
  uint8_t data[3];
  for (uint8_t i = 0; i < Params::PARAM_CRC_CHECKS; i++)
  {
    if (getHoldMasterMode())
    {
      if (isError(busReceive(command, data, sizeof(data) / sizeof(data[0]))))
      {
        break;
      }
    }
    else
    {
      while (busReceive(command, data, sizeof(data) / sizeof(data[0])) ==
             ResultCodes::ERROR_RCV_DATA)
      {
        wait(getConversionTimeTemp());
      }
      if (isError())
      {
        break;
      }
    }
    // Test status bits (last 2 from LSB) and CRC, calculate without status bits
    if ((data[1] & B11) == B00 &&
        (command == Commands::CMD_READ_TEMP_FROM_RH || checkCrc8(data)))
    {
      return calculateTemperature((data[0] << 8) | (data[1] & 0xFC));
    }
  }
  setLastResult(isSuccess() ? ResultCodes::ERROR_MEASURE : getLastResult());
  return getErrorRHT();
}

float gbj_si70::measureHumidity()
{
  uint8_t data[3];
  for (uint8_t i = 0; i < Params::PARAM_CRC_CHECKS; i++)
  {
    if (getHoldMasterMode())
    {
      if (isError(busReceive(Commands::CMD_MEASURE_RH_HOLD,
                             data,
                             sizeof(data) / sizeof(data[0]))))
      {
        break;
      }
    }
    else
    {
      while (busReceive(Commands::CMD_MEASURE_RH_NOHOLD,
                        data,
                        sizeof(data) / sizeof(data[0])) ==
             ResultCodes::ERROR_RCV_DATA)
      {
        wait(getConversionTimeRhum());
      }
      if (isError())
      {
        break;
      }
    }
    // Test status bits (last 2 from LSB) and CRC, calculate without status bits
    if ((data[1] & B11) == B10 && checkCrc8(data))
    {
      return calculateHumidity((data[0] << 8) | (data[1] & 0xFC));
    }
  }
  setLastResult(isSuccess() ? ResultCodes::ERROR_MEASURE : getLastResult());
  return getErrorRHT();
}

gbj_si70::ResultCodes gbj_si70::readSerialNumber()
{
  // Ask for SNA bytes
  {
    uint8_t data[8];
    // Read and validate SNA - 4 upper bytes of serial number
    if (isError(busReceive(
          Commands::CMD_READ_SNA, data, sizeof(data) / sizeof(data[0]))))
    {
      return setLastResult(ResultCodes::ERROR_SN);
    }
    _status.serialSNA = 0x00000000;
    /* From SNA_3 to SNA_0.
      After each SNA byte the CRC byte follows, i.e., there are 4 pairs of
      SNA-CRC bytes.
    */
    for (uint8_t i = 0; i < (sizeof(data) / sizeof(data[0]) / 2); i++)
    {
      _status.serialSNA <<= 8;
      _status.serialSNA |= data[2 * i];
      if (!checkCrc8(&data[2 * i], 1))
      {
        return setLastResult(ResultCodes::ERROR_SN);
      }
    }
  }
  // Ask for SNB bytes
  {
    // Read and validate SNB - 4 lower bytes of serial number
    uint8_t data[6];
    if (isError(busReceive(
          Commands::CMD_READ_SNB, data, sizeof(data) / sizeof(data[0]))))
    {
      return setLastResult(ResultCodes::ERROR_SN);
    }
    _status.serialSNB = 0x00000000;
    /* From SNB_3 to SNB_0.
      After each pair of SNA bytes the CRC byte follows, i.e., there are 2
      tripples of SNB-SNB-CRC bytes.
    */
    for (uint8_t i = 0; i < (sizeof(data) / sizeof(data[0]) / 3); i++)
    {
      _status.serialSNB <<= 8;
      _status.serialSNB |= data[3 * i];
      _status.serialSNB <<= 8;
      _status.serialSNB |= data[3 * i + 1];
      if (!checkCrc8(&data[3 * i]))
      {
        return setLastResult(ResultCodes::ERROR_SN);
      }
    }
  }
  return getLastResult();
}
