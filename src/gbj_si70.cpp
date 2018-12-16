#include "gbj_si70.h"
const String gbj_si70::VERSION = "GBJ_SI70 1.0.0";


uint8_t gbj_si70::begin(bool holdMasterMode)
{
  if (gbj_twowire::begin()) return getLastResult();
  if (setAddress()) return getLastResult();
  setUseValuesMax();
  setHoldMasterMode(holdMasterMode);
  if (reset()) return getLastResult();
  bool origBusStop = getBusStop();
  setBusRpte();
  if (readFwRevision()) return getLastResult();
  setBusStopFlag(origBusStop);
  if (readSerialNumber()) return getLastResult();
  return getLastResult();
}


uint8_t gbj_si70::reset()
{
  if (busSend(CMD_RESET)) return getLastResult();
  wait(getUseValuesTyp() ? TIMING_RESET_TYP : TIMING_RESET_MAX);
  // Check user register 1 reset value
  if (readUserRegister()) return getLastResult();
  if (_userReg.value != RESET_REG_USER) return setLastResult(ERROR_RESET);
  if (setHeaterDisabled()) return getLastResult(); // Turn off heater
  return getLastResult();
}


uint8_t gbj_si70::writeLockByte()
{
  bool origBusStop = getBusStop();
  setBusRpte();
  if (busSend(CMD_LOCK_BYTE_WRITE)) return getLastResult();
  setBusStopFlag(origBusStop);
  if (busSend(CMD_LOCK_BYTE_VALUE)) return getLastResult();
  return getLastResult();
}


float gbj_si70::measureHumidity()
{
  if (getHoldMasterMode() && busSend(CMD_MEASURE_RH_HOLD)) return setLastResult(ERROR_MEASURE_RHUM);
  if (!getHoldMasterMode() && busSend(CMD_MEASURE_RH_NOHOLD)) return setLastResult(ERROR_MEASURE_RHUM);
  // Read measuring and checksum bytes
  wait(getUseValuesTyp() ? getConversionTimeRhumTyp() : getConversionTimeRhumMax());
  uint8_t data[3];
  if (busReceive(data, sizeof(data)/sizeof(data[0]))) return setLastResult(ERROR_MEASURE_RHUM);
  uint16_t wordMeasure;
  wordMeasure = data[0] << 8;   // MSB
  wordMeasure |= data[1];       // LSB
  if (!checkCrc8((uint32_t) wordMeasure, data[2])) return setLastResult(ERROR_MEASURE_RHUM);
  // Convert measured humidity to percentage of relative humidity
  return calculateHumidity(wordMeasure);
}


float gbj_si70::measureHumidity(float *temperature)
{
  bool origBusStop = getBusStop();
  setBusRpte();
  float humidity = measureHumidity();
  setBusStopFlag(origBusStop);
  *temperature = readTemperature(CMD_READ_TEMP_FROM_RH);
  return humidity;
}


float gbj_si70::measureTemperature()
{
  if (getHoldMasterMode())
  {
    return readTemperature(CMD_MEASURE_TEMP_HOLD);
  }
  else
  {
    return readTemperature(CMD_MEASURE_TEMP_NOHOLD);
  }
}


//-------------------------------------------------------------------------
// Setters
//-------------------------------------------------------------------------
uint8_t gbj_si70::setResolution(uint8_t resolution)
{
  initLastResult();
  switch (resolution)
  {
    case RESOLUTION_T11_RH11:
      setResolutionTemp11();
      break;

    case RESOLUTION_T12_RH8:
      setResolutionTemp12();
      break;

    case RESOLUTION_T13_RH10:
      setResolutionTemp13();
      break;

    default:
      setResolutionTemp14();
      break;
  }
  return getLastResult();
}


uint8_t gbj_si70::setHeaterLevel(uint8_t heaterLevel)
{
  if (!_heater.enabled)
  {
    if (readHeaterRegister()) return getLastResult();
  }
  // Write heater level for Heater [3:0] bits of heater control register byte if needed
  if (getHeaterLevel() != heaterLevel)
  {
    _heater.regValue &= B11110000;     // Reset heater level
    _heater.regValue |= heaterLevel;   // Set heater level
    if (writeHeaterRegister()) return getLastResult();
  }
  return getLastResult();
}


//------------------------------------------------------------------------------
// Getters
//------------------------------------------------------------------------------
bool gbj_si70::getVddStatus()
{
  // Always read user register, because the voltage status is update after
  // each measurement
  if (readUserRegister()) return false;
  uint8_t status = (_userReg.value >> 6) & B1;
  return (status == 0);
}


bool gbj_si70::getHeaterEnabled()
{
  if (!_userReg.read)
  {
    if (readUserRegister()) return false;
  }
  // Separate heater status from HTRE (D2) bit of user register byte
  uint8_t status = (_userReg.value >> 2) & B1;
  return (status == 1);
}


uint8_t gbj_si70::getHeaterLevel()
{
  if (!_heater.enabled)
  {
    if (readHeaterRegister()) return getLastResult();
  }
  return _heater.regValue & B1111;
}


float gbj_si70::getHeaterCurrent()
{
  uint8_t heaterLevel = getHeaterLevel();
  if (isError())
  {
    return 0.0;
  }
  else
  {
    return (heaterLevel / 0xF) \
    * (_heater.currentMax - _heater.currentMin) \
    + _heater.currentMin;
  }
}


uint64_t gbj_si70::getSerialNumber()
{
  uint64_t serial;
  serial = _status.serialSNA;
  serial <<= 32;
  serial |= _status.serialSNB;
  return serial;
}


uint8_t gbj_si70::getResolutionTemp()
{
  if (!_userReg.read)
  {
    if (readUserRegister()) return getLastResult();
  }
  return _resolusion.tempBits[resolution()];
}


uint8_t gbj_si70::getConversionTimeTempMax()
{
  if (!_userReg.read)
  {
    if (readUserRegister()) return getLastResult();
  }
  return _resolusion.tempConvTimeMax[resolution()];
}


uint8_t gbj_si70::getConversionTimeTempTyp()
{
  if (!_userReg.read)
  {
    if (readUserRegister()) return getLastResult();
  }
  return _resolusion.tempConvTimeTyp[resolution()];
}


uint8_t gbj_si70::getResolutionRhum()
{
  if (!_userReg.read)
  {
    if (readUserRegister()) return getLastResult();
  }
  return _resolusion.rhumBits[resolution()];
}


uint8_t gbj_si70::getConversionTimeRhumMax()
{
  if (!_userReg.read)
  {
    if (readUserRegister()) return getLastResult();
  }
  return _resolusion.rhumConvTimeMax[resolution()];
}


uint8_t gbj_si70::getConversionTimeRhumTyp()
{
  if (!_userReg.read)
  {
    if (readUserRegister()) return getLastResult();
  }
  return _resolusion.rhumConvTimeTyp[resolution()];
}


//------------------------------------------------------------------------------
// Auxilliary methods
//------------------------------------------------------------------------------
// Calculate checksum
uint8_t gbj_si70::crc8(uint32_t data)
{
  uint8_t crc = 0x00;                        // Initialization by datasheet
  for (uint8_t i = 4; i > 0; i--)
  {
    crc ^= (data >> (8 * (i - 1))) & 0xFF;  // Separate byte from MSB to LSB
    for (uint8_t j = 8; j > 0; j--)         // Bitwise division by polynomial
    {
      if (crc & 0x80) crc = (crc << 1) ^ 0x131; // Polynomial x^8+x^5+x^4+x^0 by datasheet
      else crc = (crc << 1);
    }
  }
  return crc;
}


// Validate checksum
bool gbj_si70::checkCrc8(uint32_t data, uint8_t crc)
{
  return crc == crc8(data);
}


// Calculate resolution code
uint8_t  gbj_si70::resolution()
{
  // Separate resolution from RES1 (D7) and RES0 (D0) bit of user register byte
  uint8_t res0 = (_userReg.value >> 0) & B1;
  uint8_t res1 = (_userReg.value >> 7) & B1;
  return (res1 << 1) | res0;
}


uint8_t gbj_si70::readFwRevision()
{
  bool origBusStop = getBusStop();
  setBusRpte();
  if (busSend(CMD_READ_FW_REVISION)) return setLastResult(ERROR_FIRMWARE);
  uint8_t data[1];
  setBusStopFlag(origBusStop);
  if (busReceive(data, sizeof(data)/sizeof(data[0]))) return setLastResult(ERROR_FIRMWARE);
  _status.fwRevision = data[0];
  return getLastResult();
}


uint8_t gbj_si70::readSerialNumber()
{
  bool origBusStop = getBusStop();
  setBusRpte();
  // Ask for SNA bytes
  {
    uint8_t data[8];
    if (busSend(CMD_READ_SNA)) return setLastResult(ERROR_SERIAL_A);
    // Read and validate SNA - 4 upper bytes of serial number
    if (busReceive(data, sizeof(data)/sizeof(data[0]))) return setLastResult(ERROR_SERIAL_A);
    _status.serialSNA = 0x00000000;
    /* From SNA_3 to SNA_0
      After each SNA byte the CRC byte follows, i.e., there are 4 pairs of
      SNA-CRC bytes.
    */
    for (uint8_t i = 0; i < (sizeof(data) / sizeof(data[0]) / 2); i++)
    {
      _status.serialSNA <<= 8;
      _status.serialSNA |= data[2*i];
      if (!checkCrc8(_status.serialSNA, data[2*i + 1])) return setLastResult(ERROR_SERIAL_A);
    }
  }
  // Ask for SNB bytes
  {
    if (busSend(CMD_READ_SNB)) return setLastResult(ERROR_SERIAL_B);
    // Read and validate SNB - 4 lower bytes of serial number
    uint8_t data[6];
    setBusStopFlag(origBusStop);
    if (busReceive(data, sizeof(data)/sizeof(data[0]))) return setLastResult(ERROR_SERIAL_B);
    _status.serialSNB = 0x00000000;
    /* From SNB_3 to SNB_0
      After each pair of SNA bytes the CRC byte follows, i.e., there are 2 tripples
      of SNB-SNB-CRC bytes.
    */
    for (uint8_t i = 0; i < (sizeof(data) / sizeof(data[0]) / 3); i++)
    {
      _status.serialSNB <<= 8;
      _status.serialSNB |= data[3*i];
      _status.serialSNB <<= 8;
      _status.serialSNB |= data[3*i + 1];
      if (!checkCrc8(_status.serialSNB, data[3*i + 2])) return setLastResult(ERROR_SERIAL_B);
    }
  }
  return getLastResult();
}


uint8_t gbj_si70::readUserRegister()
{
  bool origBusStop = getBusStop();
  setBusRpte();
  if (busSend(CMD_REG_RHT_READ)) return setLastResult(ERROR_REG_RHT_READ);
  uint8_t data[1];
  setBusStopFlag(origBusStop);
  if (busReceive(data, sizeof(data)/sizeof(data[0]))) return setLastResult(ERROR_REG_RHT_READ);
  _userReg.value = data[0];
  _userReg.read = true;
  return getLastResult();
}


uint8_t gbj_si70::writeUserRegister()
{
  if (busSend(CMD_REG_RHT_WRITE, _userReg.value)) return getLastResult();
  _userReg.read = false;  // Reread the user register the next time for sure
  return getLastResult();
}


uint8_t gbj_si70::readHeaterRegister()
{
  bool origBusStop = getBusStop();
  setBusRpte();
  if (busSend(CMD_REG_HEATER_READ)) return setLastResult(ERROR_REG_HEATER_READ);
  uint8_t data[1];
  setBusStopFlag(origBusStop);
  if (busReceive(data, sizeof(data)/sizeof(data[0]))) return setLastResult(ERROR_REG_HEATER_READ);
  _heater.regValue = data[0];
  _heater.enabled = true;
  return getLastResult();
}


uint8_t gbj_si70::writeHeaterRegister()
{
  if (busSend(CMD_REG_HEATER_WRITE, _heater.regValue)) return getLastResult();
  _heater.enabled = false;  // Reread the heater register the next time for sure
  return getLastResult();
}


uint8_t gbj_si70::setHeaterStatus(bool status)
{
  // Read user register if needed
  if (!_userReg.read)
  {
    if (readUserRegister()) return getLastResult();
  }
  // Write heater status for HTRE (D2) bit of user register byte if needed
  if (!getHeaterEnabled() && status)
  {
    _userReg.value |= B00000100;  // Set HTRE to 1
    return writeUserRegister();
  }
  if (getHeaterEnabled() && !status)
  {
    _userReg.value &= B11111011;  // Set HTRE to 0
    return writeUserRegister();
  }
  return getLastResult();
}


uint8_t gbj_si70::setBitResolution(bool bitRes1, bool bitRes0)
{
  // Read user register if needed
  if (!_userReg.read)
  {
    if (readUserRegister()) return getLastResult();
  }
  // Determine resolution code
  uint8_t code = ((bitRes1 ? B1 : B0) << 1) | (bitRes0 ? B1 : B0);
  // Write resolution bits RES1 (D7) and RES0 (D0) to user register byte if needed
  if (resolution() != code)
  {
    if (bitRes0)
    {
      _userReg.value |= B00000001;  // Set RES0 to 1
    }
    else
    {
      _userReg.value &= B11111110;  // Set RES0 to 0
    }
    if (bitRes1)
    {
      _userReg.value |= B10000000;  // Set RES1 to 1
    }
    else
    {
      _userReg.value &= B01111111;  // Set RES1 to 0
    }
    return writeUserRegister();
  }
  return getLastResult();
}


float gbj_si70::readTemperature(uint8_t command)
{
  if (busSend(command)) return setLastResult(ERROR_MEASURE_TEMP);
  wait(getUseValuesTyp() ? getConversionTimeTempTyp() : getConversionTimeTempMax());
  uint8_t data[3];
  if (busReceive(data, sizeof(data)/sizeof(data[0]))) return setLastResult(ERROR_MEASURE_TEMP);
  uint16_t wordMeasure;
  wordMeasure = data[0] << 8;   // MSB
  wordMeasure |= data[1];       // LSB
  if (command != CMD_READ_TEMP_FROM_RH \
  && !checkCrc8((uint32_t) wordMeasure, data[2])) return setLastResult(ERROR_MEASURE_TEMP);
  // Convert measured temperature to centigrade
  return calculateTemperature(wordMeasure);
}


float gbj_si70::calculateTemperature(uint16_t wordMeasure)
{
  float temperature = (float) wordMeasure;
  temperature *= 175.72;
  temperature /= 65536.0;
  temperature -= 46.85;
  return temperature;
}


float gbj_si70::calculateHumidity(uint16_t wordMeasure)
{
  float humidity = (float) wordMeasure;
  humidity *= 125.0;
  humidity /= 65536.0;
  humidity -= 6.0;
  return humidity;
}
