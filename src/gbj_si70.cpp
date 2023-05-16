#include "gbj_si70.h"

float gbj_si70::readTemperature(Commands command)
{
  bool origBusStop = getBusStop();
  uint8_t data[3];
  uint16_t wordMeasure;
  for (uint8_t i = 0; i < Params::PARAM_CRC_CHECKS; i++)
  {
    if (getHoldMasterMode())
    {
      setBusRepeat();
      if (isError(busSend(command)))
      {
        break;
      }
      setBusStopFlag(origBusStop);
      if (isError(busReceive(data, sizeof(data) / sizeof(data[0]))))
      {
        break;
      }
    }
    else
    {
      setBusRepeat();
      uint32_t waitNACK = (getUseValuesTyp() ? getConversionTimeTempTyp()
                                             : getConversionTimeTempMax());
      if (isError(busSend(command)))
      {
        break;
      }
      setBusStopFlag(origBusStop);
      while (busReceive(data, sizeof(data) / sizeof(data[0])) ==
             ResultCodes::ERROR_RCV_DATA)
      {
        wait(waitNACK);
      }
      if (isError())
      {
        break;
      }
    }
    wordMeasure = data[0] << 8;
    wordMeasure |= data[1];
    // if (checkCrc8((uint32_t) wordMeasure, data[2])) return
    // calculateTemperature(wordMeasure);
    if (data[0] == data[2] ||
        checkCrc8(static_cast<uint32_t>(wordMeasure), data[2]))
    {
      return calculateTemperature(wordMeasure);
    }
  }
  setLastResult(getLastResult() == ResultCodes::SUCCESS
                  ? ResultCodes::ERROR_MEASURE
                  : getLastResult());
  return static_cast<float>(Params::PARAM_BAD_RHT);
}

float gbj_si70::measureHumidity()
{
  bool origBusStop = getBusStop();
  uint8_t data[3];
  uint16_t wordMeasure;
  for (uint8_t i = 0; i < Params::PARAM_CRC_CHECKS; i++)
  {
    if (getHoldMasterMode())
    {
      setBusRepeat();
      if (isError(busSend(Commands::CMD_MEASURE_RH_HOLD)))
      {
        break;
      }
      setBusStopFlag(origBusStop);
      if (isError(busReceive(data, sizeof(data) / sizeof(data[0]))))
      {
        break;
      }
    }
    else
    {
      setBusRepeat();
      uint32_t waitNACK = (getUseValuesTyp() ? getConversionTimeRhumTyp()
                                             : getConversionTimeRhumMax());
      if (isError(busSend(Commands::CMD_MEASURE_RH_NOHOLD)))
      {
        break;
      }
      setBusStopFlag(origBusStop);
      while (busReceive(data, sizeof(data) / sizeof(data[0])) ==
             ResultCodes::ERROR_RCV_DATA)
      {
        wait(waitNACK);
      }
      if (isError())
      {
        break;
      }
    }
    wordMeasure = data[0] << 8;
    wordMeasure |= data[1];
    if (data[0] == data[2] ||
        checkCrc8(static_cast<uint32_t>(wordMeasure), data[2]))
    {
      return calculateHumidity(wordMeasure);
    }
  }
  setLastResult(getLastResult() == ResultCodes::SUCCESS
                  ? ResultCodes::ERROR_MEASURE
                  : getLastResult());
  return static_cast<float>(Params::PARAM_BAD_RHT);
}

gbj_si70::ResultCodes gbj_si70::readSerialNumber()
{
  bool origBusStop = getBusStop();
  setBusRepeat();
  // Ask for SNA bytes
  {
    uint8_t data[8];
    if (isError(busSend(Commands::CMD_READ_SNA)))
    {
      return setLastResult(ResultCodes::ERROR_SN);
    }
    // Read and validate SNA - 4 upper bytes of serial number
    if (isError(busReceive(data, sizeof(data) / sizeof(data[0]))))
    {
      return setLastResult(ResultCodes::ERROR_SN);
    }
    _status.serialSNA = 0x00000000;
    /* From SNA_3 to SNA_0
      After each SNA byte the CRC byte follows, i.e., there are 4 pairs of
      SNA-CRC bytes.
    */
    for (uint8_t i = 0; i < (sizeof(data) / sizeof(data[0]) / 2); i++)
    {
      _status.serialSNA <<= 8;
      _status.serialSNA |= data[2 * i];
      if (!checkCrc8(_status.serialSNA, data[2 * i + 1]))
      {
        return setLastResult(ResultCodes::ERROR_SN);
      }
    }
  }
  // Ask for SNB bytes
  {
    if (isError(busSend(Commands::CMD_READ_SNB)))
      return setLastResult(ResultCodes::ERROR_SN);
    // Read and validate SNB - 4 lower bytes of serial number
    uint8_t data[6];
    setBusStopFlag(origBusStop);
    if (isError(busReceive(data, sizeof(data) / sizeof(data[0]))))
    {
      return setLastResult(ResultCodes::ERROR_SN);
    }
    _status.serialSNB = 0x00000000;
    /* From SNB_3 to SNB_0
      After each pair of SNA bytes the CRC byte follows, i.e., there are 2
      tripples of SNB-SNB-CRC bytes.
    */
    for (uint8_t i = 0; i < (sizeof(data) / sizeof(data[0]) / 3); i++)
    {
      _status.serialSNB <<= 8;
      _status.serialSNB |= data[3 * i];
      _status.serialSNB <<= 8;
      _status.serialSNB |= data[3 * i + 1];
      if (!checkCrc8(_status.serialSNB, data[3 * i + 2]))
        return setLastResult(ResultCodes::ERROR_SN);
    }
  }
  return getLastResult();
}
