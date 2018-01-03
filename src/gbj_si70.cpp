#include "gbj_si70.h"


uint8_t gbj_si70::begin(bool busStop)
{
  initLastResult();
  setBusStop(busStop);
  if (setAddress(GBJ_SI70_ADDRESS)) return getLastResult();
  if (reset()) return getLastResult();
  if (readFwRevision()) return getLastResult();
  if (readSerialNumber()) return getLastResult();
  return getLastResult();
}


uint8_t gbj_si70::reset()
{
  initBus();
  beginTransmission(getAddress());
  writeByte(GBJ_SI70_RESET);
  if (setLastResult(endTransmission(getBusStop()))) return getLastResult();
  wait(50);  // Wait for resetting

  // Send command to read control user register 1
  beginTransmission(getAddress());
  writeByte(GBJ_SI70_REG_RHT_READ);
  if (setLastResult(endTransmission(getBusStop()))) return getLastResult();

  // Read control user register's reset settings value
  uint8_t resetSetting;
  uint8_t byteCount = 1;
  if (requestFrom(getAddress(), byteCount, (uint8_t) getBusStop()) > 0 \
  && available() >= byteCount)
  {
    resetSetting = read();
  }
  if (setLastResult(endTransmission(getBusStop()))) return getLastResult();

  // Validate reset settings
  if (resetSetting != GBJ_SI70_RST_REG_USER) return setLastResult(GBJ_SI70_ERR_RESET);
  return getLastResult();
}


uint8_t gbj_si70::writeLockByte()
{
  initBus();
  beginTransmission(getAddress());
  writeInt(GBJ_SI70_LOCK_BYTE_WRITE);
  writeByte(GBJ_SI70_LOCK_BYTE_VALUE);
  return setLastResult(endTransmission(getBusStop()));
}


float gbj_si70::measureHumidity(bool holdMasterMode)
{
  initBus();
  beginTransmission(getAddress());
  if (holdMasterMode)
  {
    writeByte(GBJ_SI70_MEASURE_RH_HOLD);
  }
  else
  {
    writeByte(GBJ_SI70_MEASURE_RH_NOHOLD);
  }
  if (setLastResult(endTransmission(getBusStop())))
  {
    return setLastResult(GBJ_SI70_ERR_MEASURE_RHUM);
  }
  // Read measuring and checksum bytes
  wait(GBJ_SI70_CONVERSION_TIME);
  uint16_t wordMeasure;
  uint8_t byteCount = 3;
  if (requestFrom(getAddress(), byteCount, (uint8_t) getBusStop()) > 0 \
  && available() >= byteCount)
  {
    wordMeasure = read();   // Read MSB
    wordMeasure <<= 8;
    wordMeasure |= read();  // Read LSB
    if (!checkCrc8((uint32_t) wordMeasure, read()))  // Read checksum
    {
      return setLastResult(GBJ_SI70_ERR_MEASURE_RHUM);
    }
  }
  else
  {
    return setLastResult(GBJ_SI70_ERR_MEASURE_RHUM);
  }
  if (setLastResult(endTransmission(getBusStop())))
  {
    return setLastResult(GBJ_SI70_ERR_MEASURE_RHUM);
  }
  // Convert measured humidity to percentage of relative humidity
  return calculateHumidity(wordMeasure);
}


float gbj_si70::measureHumidity(float *temperature, bool holdMasterMode)
{
    float humidity = measureHumidity(holdMasterMode);
    *temperature = readTemperature(GBJ_SI70_READ_TEMP_FROM_RH); // Retrieve temperature
    return humidity;
}


float gbj_si70::measureTemperature(bool holdMasterMode)
{
  if (holdMasterMode)
  {
    return readTemperature(GBJ_SI70_MEASURE_TEMP_HOLD);
  }
  else
  {
    return readTemperature(GBJ_SI70_MEASURE_TEMP_NOHOLD);
  }
}


//-------------------------------------------------------------------------
// Setters
//-------------------------------------------------------------------------
uint8_t gbj_si70::setHeaterEnabled()    { return setHeaterStatus(true); }
uint8_t gbj_si70::setHeaterDisabled()   { return setHeaterStatus(false); }
//
uint8_t gbj_si70::setResolutionTemp14() { return setBitResolution(false, false); }
uint8_t gbj_si70::setResolutionRhum12() { return setResolutionTemp14(); }
//
uint8_t gbj_si70::setResolutionTemp13() { return setBitResolution(true, false); }
uint8_t gbj_si70::setResolutionRhum10() { return setResolutionTemp13(); }
//
uint8_t gbj_si70::setResolutionTemp12() { return setBitResolution(false, true); }
uint8_t gbj_si70::setResolutionRhum8()  { return setResolutionTemp12(); }
//
uint8_t gbj_si70::setResolutionTemp11() { return setBitResolution(true, true); }
uint8_t gbj_si70::setResolutionRhum11() { return setResolutionTemp11(); }


uint8_t gbj_si70::setResolution(uint8_t resolution)
{
  initLastResult();
  switch (resolution)
  {
    case GBJ_SI70_RES_T11_RH11:
      setResolutionTemp11();
      break;

    case GBJ_SI70_RES_T12_RH8:
      setResolutionTemp12();
      break;

    case GBJ_SI70_RES_T13_RH10:
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
  initLastResult();
  // Read heater control register if needed
  if (!_heaterRegEnabled)
  {
    if (readHeaterRegister()) return getLastResult();
  }
  // Write heater level for Heater [3:0] bits of heater control register byte if needed
  if (getHeaterLevel() != heaterLevel)
  {
    _heaterRegister &= B11110000;     // Reset heater level
    _heaterRegister |= heaterLevel;   // Set heater level
    return writeHeaterRegister();
  }
  return getLastResult();
}


//------------------------------------------------------------------------------
// Getters
//------------------------------------------------------------------------------
uint32_t gbj_si70::getSerialUpper() { return _serialSNA; }
uint32_t gbj_si70::getSerialLower() { return _serialSNB; }
uint8_t  gbj_si70::getFwRevision()  { return _fwRevision; }
uint8_t  gbj_si70::getDeviceType()  { return (_serialSNB >> 24); }


bool gbj_si70::getVddStatus()
{
  // Read user register if needed
  if (!_userRegEnabled)
  {
    if (readUserRegister()) return false;
  }
  // Separate Vdd status from VDDS (D6) bit of user register byte
  uint8_t status = (_userRegister >> 6) & B1;
  return (status == 0);
}


bool gbj_si70::getHeaterEnabled()
{
  // Read user register if needed
  if (!_userRegEnabled)
  {
    if (readUserRegister()) return false;
  }
  // Separate heater status from HTRE (D2) bit of user register byte
  uint8_t status = (_userRegister >> 2) & B1;
  return (status == 1);
}


uint8_t gbj_si70::getHeaterLevel()
{
  if (!_heaterRegEnabled)
  {
    if (readHeaterRegister()) return getLastResult();
  }
  return _heaterRegister & B1111;
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
    * (GBJ_SI70_HEATER_CURRENT_MAX - GBJ_SI70_HEATER_CURRENT_MIN) \
    + GBJ_SI70_HEATER_CURRENT_MIN;
  }
}


uint8_t gbj_si70::getResolutionTemp()
{
  if (!_userRegEnabled)
  {
    if (readUserRegister()) return getLastResult();
  }
  return _tempResolution[resolution()];
}


uint8_t gbj_si70::getResolutionRhum()
{
  if (!_userRegEnabled)
  {
    if (readUserRegister()) return getLastResult();
  }
  return _rhumResolution[resolution()];
}


//------------------------------------------------------------------------------
// Private methods
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
  uint8_t res0 = (_userRegister >> 0) & B1;
  uint8_t res1 = (_userRegister >> 7) & B1;
  return (res1 << 1) | res0;
}


uint8_t gbj_si70::readFwRevision()
{
  initBus();
  // Ask for revision by two commands
  beginTransmission(getAddress());
  writeInt(GBJ_SI70_READ_FW_REVISION);
  if (setLastResult(endTransmission(getBusStop()))) return getLastResult();
  // Read revision byte
  uint8_t byteCount = 1;
  if (requestFrom(getAddress(), byteCount, (uint8_t) getBusStop()) > 0 \
  && available() >= byteCount)
  {
    _fwRevision = read();
  }
  else
  {
    return setLastResult(GBJ_SI70_ERR_FIRMWARE);
  }
  return setLastResult(endTransmission(getBusStop()));
}


uint8_t gbj_si70::readSerialNumber()
{
  // Ask for SNA bytes by two commands
  initBus();
  beginTransmission(getAddress());
  writeInt(GBJ_SI70_READ_SNA);
  if (setLastResult(endTransmission(getBusStop()))) return getLastResult();
  // Read and validate SNA - 4 upper bytes of serial number
  uint8_t byteCount = 8;
  if (requestFrom(getAddress(), byteCount, (uint8_t) getBusStop()) > 0 \
  && available() >= byteCount)
  {
      _serialSNA = 0x00000000;
      for (uint8_t i = 0; i < byteCount / 2; i++) // From SNA_3 to SNA_0
      {
        _serialSNA <<= 8;
        _serialSNA |= read();
        if (!checkCrc8(_serialSNA, read()))
        {
          return setLastResult(GBJ_SI70_ERR_SERIAL_A);
        }
      }
  }
  else
  {
    return setLastResult(GBJ_SI70_ERR_SERIAL_A);
  }
  if (setLastResult(endTransmission(getBusStop()))) return getLastResult();
  // Ask for SNB bytes by two commands
  initBus();
  beginTransmission(getAddress());
  writeInt(GBJ_SI70_READ_SNB);
  if (setLastResult(endTransmission(getBusStop()))) return getLastResult();
  // Read and validate SNB - 4 lower bytes of serial number
  byteCount = 6;
  if (requestFrom(getAddress(), byteCount, (uint8_t) getBusStop()) > 0 \
  && available() >= byteCount)
  {
      _serialSNB = 0x00000000;
      for (uint8_t i = 0; i < byteCount / 3; i++) // From SNB_3 to SNB_0
      {
        _serialSNB <<= 8;
        _serialSNB |= read();
        _serialSNB <<= 8;
        _serialSNB |= read();
        if (!checkCrc8(_serialSNB, read()))
        {
          return setLastResult(GBJ_SI70_ERR_SERIAL_B);
        }
      }
  }
  else
  {
    return setLastResult(GBJ_SI70_ERR_SERIAL_B);
  }
  return setLastResult(endTransmission(getBusStop()));
}


uint8_t gbj_si70::readUserRegister()
{
  initBus();
  // Ask for reading the User Register 1
  beginTransmission(getAddress());
  writeByte(GBJ_SI70_REG_RHT_READ);
  if (setLastResult(endTransmission(getBusStop()))) return getLastResult();
  // Read user register
  uint8_t byteCount = 1;
  if (requestFrom(getAddress(), byteCount, (uint8_t) getBusStop()) > 0 \
  && available() >= byteCount)
  {
    _userRegister = read();
  }
  else
  {
    return setLastResult(GBJ_SI70_ERR_REG_RHT_READ);
  }
  if (setLastResult(endTransmission(getBusStop()))) return getLastResult();
  _userRegEnabled = true;
  return getLastResult();
}


uint8_t gbj_si70::writeUserRegister()
{
  initBus();
  beginTransmission(getAddress());
  writeByte(GBJ_SI70_REG_RHT_WRITE);
  writeByte(_userRegister);
  if (setLastResult(endTransmission(getBusStop()))) return getLastResult();
  _userRegEnabled = false;  // Reread the user register the next time for sure
  return getLastResult();
}


uint8_t gbj_si70::readHeaterRegister()
{
  // Ask for reading the Heater Control Register
  initBus();
  beginTransmission(getAddress());
  writeByte(GBJ_SI70_REG_HEATER_READ);
  if (setLastResult(endTransmission(getBusStop()))) return getLastResult();
  // Read heater register
  uint8_t byteCount = 1;
  if (requestFrom(getAddress(), byteCount, (uint8_t) getBusStop()) > 0 \
  && available() >= byteCount)
  {
    _heaterRegister = read();
  }
  else
  {
    return setLastResult(GBJ_SI70_ERR_REG_HEATER_READ);
  }
  if (setLastResult(endTransmission(getBusStop()))) return getLastResult();
  _heaterRegEnabled = true;
  return getLastResult();
}


uint8_t gbj_si70::writeHeaterRegister()
{
  initBus();
  beginTransmission(getAddress());
  writeByte(GBJ_SI70_REG_HEATER_WRITE);
  writeByte(_heaterRegister);
  _heaterRegEnabled = false;  // Reread the heater register the next time for sure
  return setLastResult(endTransmission(getBusStop()));
}


uint8_t gbj_si70::setHeaterStatus(bool status)
{
  initLastResult();
  // Read user register if needed
  if (!_userRegEnabled)
  {
    if (readUserRegister()) return getLastResult();
  }
  // Write heater status for HTRE (D2) bit of user register byte if needed
  if (!getHeaterEnabled() && status)
  {
    _userRegister |= B00000100;  // Set HTRE to 1
    return writeUserRegister();
  }
  if (getHeaterEnabled() && !status)
  {
    _userRegister &= B11111011;  // Set HTRE to 0
    return writeUserRegister();
  }
  return getLastResult();
}


uint8_t gbj_si70::setBitResolution(bool bitRes1, bool bitRes0)
{
  initLastResult();
  // Read user register if needed
  if (!_userRegEnabled)
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
      _userRegister |= B00000001;  // Set RES0 to 1
    }
    else
    {
      _userRegister &= B11111110;  // Set RES0 to 0
    }
    if (bitRes1)
    {
      _userRegister |= B10000000;  // Set RES1 to 1
    }
    else
    {
      _userRegister &= B01111111;  // Set RES1 to 0
    }
    return writeUserRegister();
  }
  return getLastResult();
}


float gbj_si70::readTemperature(uint8_t command)
{
  initBus();
  beginTransmission(getAddress());
  writeByte(command);
  if (setLastResult(endTransmission(getBusStop())))
  {
    return setLastResult(GBJ_SI70_ERR_MEASURE_TEMP);
  }
  // Read measuring and checksum bytes
  wait(GBJ_SI70_CONVERSION_TIME);
  uint16_t wordMeasure;
  uint8_t byteCount = 3;
  if (requestFrom(getAddress(), byteCount, (uint8_t) getBusStop()) > 0 \
  && available() >= byteCount)
  {
    wordMeasure = read();   // Read MSB
    wordMeasure <<= 8;
    wordMeasure |= read();  // Read LSB
    if (command != GBJ_SI70_READ_TEMP_FROM_RH \
    && !checkCrc8((uint32_t) wordMeasure, read()))  // Read checksum
    {
      return setLastResult(GBJ_SI70_ERR_MEASURE_TEMP);
    }
  }
  else
  {
    return setLastResult(GBJ_SI70_ERR_MEASURE_TEMP);
  }
  if (setLastResult(endTransmission(getBusStop())))
  {
    return setLastResult(GBJ_SI70_ERR_MEASURE_TEMP);
  }
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
