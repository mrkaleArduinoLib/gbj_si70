<a id="library"></a>
# gbjSI70
Library for the humidity and temperature sensors *SI70xx*, especially **SI7021** on board GY-21 communicating on two-wire (I2C) bus.

#### Particle hardware configuration
- Connect microcontroller's pin `D0` to sensor's pin **SDA** (Serial Data).
- Connect microcontroller's pin `D1` to sensor's pin **SCL** (Serial Clock).

#### Arduino UNO hardware configuration
- Connect microcontroller's pin `A4` to sensor's pin **SDA** (Serial Data).
- Connect microcontroller's pin `A5` to sensor's pin **SCL** (Serial Clock).


<a id="credit"></a>
## Credit
Library has been inspired by the library *Adafruit_Si7021*.


<a id="dependency"></a>
## Dependency

#### Particle platform
- **Particle.h**: Includes alternative (C++) data type definitions.

#### Arduino platform
- **Arduino.h**: Main include file for the Arduino SDK version greater or equal to 100.
- **WProgram.h**: Main include file for the Arduino SDK version less than 100.
- **inttypes.h**: Integer type conversions. This header file includes the exact-width integer definitions and extends them with additional facilities provided by the implementation.
- **TwoWire**: I2C system library loaded from the file *Wire.h*.

#### Custom Libraries
- **gbjTwoWire**: I2C custom library loaded from the file *gbj_twowire.h*. The library [gbjSI70](#library) inherits common bus functionality from this library.


<a id="constants"></a>
## Constants
- **GBJ\_SI70\_VERSION**: Name and semantic version of the library.
- **GBJ\_SI70\_TYPE\_SAMPLE1**: Sensor type Engineering samples.
- **GBJ\_SI70\_TYPE\_SAMPLE2**: Sensor type Engineering samples.
- **GBJ\_SI70\_TYPE\_7013**: Sensor type Si7013.
- **GBJ\_SI70\_TYPE\_7020**: Sensor type Si7020.
- **GBJ\_SI70\_TYPE\_7021**: Sensor type Si7021.
- **GBJ\_SI70\_RES\_T14\_RH12**: Resolution of temperature 14 bits, relative humidity 12 bits.
- **GBJ\_SI70\_RES\_T13\_RH10**: Resolution of temperature 13 bits, relative humidity 10 bits.
- **GBJ\_SI70\_RES\_T12\_RH8**: Resolution of temperature 12 bits, relative humidity 8 bits.
- **GBJ\_SI70\_RES\_T11\_RH11**: Resolution of temperature 11 bits, relative humidity 11 bits.
- **GBJ\_SI70\_FW\_VERSION\_10**: Firmware revision byte for sensors of the series *A10*.
- **GBJ\_SI70\_FW\_VERSION\_20**: Firmware revision byte for sensors of the series *A20*.


<a id="errors"></a>
#### Error codes
- **GBJ\_SI70\_ERR\_ADDRESS**: Bad address.
- **GBJ\_SI70\_ERR\_RESET**: Sensor reset failure.
- **GBJ\_SI70\_ERR\_FIRMWARE**: Firmware revision reading failure.
- **GBJ\_SI70\_ERR\_SERIAL\_A**: Serial number upper double word reading failure.
- **GBJ\_SI70\_ERR\_SERIAL\_B**: Serial number upper double word reading failure.
- **GBJ\_SI70\_ERR\_REG\_RHT\_READ**: Reading RH/T User Register 1 failure.
- **GBJ\_SI70\_ERR\_REG\_HEATER\_READ**: Reading Heater Control Register failure.
- **GBJ\_SI70\_ERR\_MEASURE\_RHUM**: Measuring relative humidity failure.
- **GBJ\_SI70\_ERR\_MEASURE\_TEMP**: Measuring temperature failure.

Other error codes as well as result or error codes provides the parent library [gbjTwoWire](#dependency) including other macro constants.
Remaining constants are listed in the library include file. They are used mostly internally as function codes of the sensor.


<a id="interface"></a>
## Interface
The library does not need special constructor and destructor, so that the inherited ones are good enough and there is no need to define them in the library, just use it:
```cpp
  gbj_si70 Sensor = gbj_si70();
```
- [begin()](#begin)
- [reset()](#reset)
- [measureHumidity()](#measureHumidity)
- [measureTemperature()](#measureTemperature)
- [writeLockByte()](#writeLockByte)

#### Setters
- [setResolution()](#setResolution)
- [setResolutionTemp14()](#setResolutionTemp)
- [setResolutionTemp13()](#setResolutionTemp)
- [setResolutionTemp12()](#setResolutionTemp)
- [setResolutionTemp11()](#setResolutionTemp)
- [setResolutionRhum12()](#setResolutionRhum)
- [setResolutionRhum11()](#setResolutionRhum)
- [setResolutionRhum10()](#setResolutionRhum)
- [setResolutionRhum8()](#setResolutionRhum)
- [setHeaterEnabled()](#setHeater)
- [setHeaterDisabled()](#setHeater)
- [setHeaterLevel()](#setHeaterLevel)

#### Getters
- [getResolutionTemp()](#getResolutionTemp)
- [getResolutionRhum()](#getResolutionRhum)
- [getDeviceType()](#getDeviceType)
- [getFwRevision()](#getFwRevision)
- [getHeaterEnabled()](#getHeaterEnabled)
- [getHeaterLevel()](#getHeaterLevel)
- [getHeaterCurrent()](#getHeaterCurrent)
- [getSerialUpper()](#getSerial)
- [getSerialLower()](#getSerial)
- [getVddStatus()](#getVddStatus)


<a id="begin"></a>
## begin()
#### Description
The method takes, sanitizes, and stores parameters of the sensor to a class instance object and initiates two-wire bus.
- You can use [setters](#interface) later in order to change sensor's parameters.

#### Syntax
    uint8_t begin(boolean busStop);

#### Parameters
<a id="prm_busStop"></a>
- **busStop**: Logical flag about releasing bus after end of transmission.
  - *Valid values*: true, false
    - **true**: Releases the bus after data transmission and enables other master devices to control the bus.
    - **false**: Keeps connection to the bus and enables begin further data transmission immediately.
  - *Default value*: true

#### Returns
Some of result or [error codes](#errors) including ones from the parent library.

#### Example
The method has all arguments defaulted and calling without any parameters is equivalent to the calling with all arguments set by corresponding constant with default value:

``` cpp
gbj_si70 Sensor = gbj_si70();
setup()
{
    Sensor.begin();  // It is equivalent to
    Sensor.begin(true);
}
```

[Back to interface](#interface)


<a id="reset"></a>
## reset()
#### Description
The method resets the sensor and sets control registers to their reset settings values.

#### Syntax
    uint8_t reset();

#### Parameters
None

#### Returns
Some of result or [error codes](#errors) including ones from the parent library.

[Back to interface](#interface)


<a id="writeLockByte"></a>
## writeLockByte()
#### Description
The method writes the lock byte to the non-volatile memory of a sensor. It is enough to do it just once for every sensor.

#### Syntax
    uint8_t writeLockByte();

#### Parameters
None

#### Returns
Some of result or [error codes](#errors) including ones from the parent library.

[Back to interface](#interface)


<a id="measureHumidity"></a>
## measureHumidity()
#### Description
The method is overloaded and measures either relative humidity alongside with temperature at once or the humidity alone.
The temperature is returned through referenced input parameter.

#### Syntax
    float measureHumidity(boolean holdMasterMode = false);
    float measureHumidity(float *temperature, bool holdMasterMode = false);

#### Parameters
<a id="temperature"></a>
- **temperature**: Address reference to output variable for measured temperature in centigrade.
  - *Valid values*: address range specific for a platform
  - *Default value*: none

<a id="holdMasterMode"></a>
- **holdMasterMode**: Logical flag about active hold master mode at measuring.
  - *Valid values*: true, false
  - *Default value*: false

#### Returns
- Relative humidity or the error value [GBJ\_SI70\_ERR\_MEASURE\_RHUM](#constants) with corresponding error code in the library object.
- Temperature in centigrade in the referenced variable.

#### Example
The method has the last argument defaulted.

``` cpp
gbj_si70 Sensor = gbj_si70();
float tempValue, rhumValue;
setup()
{
    Sensor.begin();
    rhumValue = Sensor.measureHumidity(&tempValue);
    rhumValue = Sensor.measureHumidity();
}
```

#### See also
[measureTemperature()](#measureTemperature)

[getLastResult()](#getLastResult)

[Back to interface](#interface)


<a id="measureTemperature"></a>
## measureTemperature()
#### Description
The method measures temperature only without measuring the relative humidity.

#### Syntax
    float measureTemperature(boolean holdMasterMode = false);

#### Parameters
<a id="holdMasterMode"></a>
- **holdMasterMode**: Logical flag about active hold master mode at measuring.
  - *Valid values*: true, false
  - *Default value*: false

#### Returns
Temperature in centigrade or the error value [GBJ\_SI70\_ERR\_MEASURE\_TEMP](#constants) with corresponding error code in the library object.

#### Example
The method has the last argument defaulted.

``` cpp
gbj_si70 Sensor = gbj_si70();
float tempValue;
setup()
{
    Sensor.begin();
    tempValue = Sensor.measureTemperature();
}
```

#### See also
[measureHumidity()](#measureHumidity)

[getLastResult()](#getLastResult)

[Back to interface](#interface)


<a id="setResolution"></a>
## setResolution()
#### Description
The method sets the bit resolution by input parameter, which should be appropriate library macro [constant](#constant).
The resolution is determined by that constant but in fact it is the bit resolution for temperature.

#### Syntax
    uint8_t setResolution(uint8_t resolution = GBJ_SI70_RES_T14_RH12);

#### Parameters
<a id="resolution"></a>
- **resolution**: Desired measurement resolution.
  - *Valid values*:  [GBJ\_SI70\_RES\_T14\_RH12](#constants), [GBJ\_SI70\_RES\_T13\_RH10](#constants),  [GBJ\_SI70\_RES\_T12\_RH8](#constants), or [GBJ\_SI70\_RES\_T11\_RH11](#constants)
  - *Default value*: [GBJ\_SI70\_RES\_T14\_RH12](#constants)

#### Returns
Some of result or [error codes](#errors) including ones from the parent library.

#### See also
[setResolutionTempXX()](#setResolutionTemp)

[setResolutionRhumXX()](#setResolutionRhum)

[Back to interface](#interface)


<a id="setResolutionTemp"></a>
## setResolutionTemp11(), setResolutionTemp12(), setResolutionTemp13(), setResolutionTemp14()
#### Description
The particular method sets the bit resolution for temperature measurement to the value in its name.
The method sets the corresponding bit resolution for the relative humidity measurement at the same time by this relation:

Temperature | Relative Humidity
------ | -------
11 | 11
12 | 8
13 | 10
14 | 12

#### Syntax
    uint8_t setResolutionTemp11();
    uint8_t setResolutionTemp12();
    uint8_t setResolutionTemp13();
    uint8_t setResolutionTemp14();

#### Parameters
None

#### Returns
Some of result or [error codes](#errors) including ones from the parent library.

#### See also
[setResolution()](#setResolution)

[getResolutionTemp()](#getResolutionTemp)

[Back to interface](#interface)


<a id="getResolutionTemp"></a>
## getResolutionTemp()
#### Description
The method returns the temperature measurement resolution in bits.

#### Syntax
    uint8_t getResolutionTemp();

#### Parameters
None

#### Returns
Bit resolution (11, 12, 13, or 14) or some of [error codes](#errors) including ones from the parent library.

#### See also
[setResolution()](#setResolution)

[setResolutionTempXX()](#setResolutionTemp)

[Back to interface](#interface)


<a id="setResolutionRhum"></a>
## setResolutionRhum8(), setResolutionRhum10(), setResolutionRhum11(), setResolutionRhum12()
#### Description
The particular method sets the bit resolution for relative humidity measurement to the value in its name.
The method sets the corresponding bit resolution for the temperature measurement at the same time by this relation:

Relative Humidity | Temperature
------ | -------
11 | 11
8 | 12
10 | 13
12 | 14

#### Syntax
    uint8_t setResolutionRhum8();
    uint8_t setResolutionRhum10();
    uint8_t setResolutionRhum11();
    uint8_t setResolutionRhum12();

#### Parameters
None

#### Returns
Some of result or [error codes](#errors) including ones from the parent library.

#### See also
[setResolution()](#setResolution)

[Back to interface](#interface)


<a id="getResolutionRhum"></a>
## getResolutionRhum()
#### Description
The method returns the relative humidity measurement resolution in bits.

#### Syntax
    uint8_t getResolutionRhum();

#### Parameters
None

#### Returns
Bit resolution (8, 10, 11, or 12) or some of [error codes](#errors) including ones from the parent library.

#### See also
[setResolution()](#setResolution)

[setResolutionRhumXX()](#setResolutionRhum)

[Back to interface](#interface)


<a id="setHeater"></a>
## setHeaterEnabled(), setHeaterDisabled()
#### Description
The particular method turns on or off a heater built-in in the sensor.

#### Syntax
    uint8_t setHeaterEnabled();
    uint8_t setHeaterDisabled();

#### Parameters
None

#### Returns
Some of result or [error codes](#errors) including ones from the parent library.

#### See also
[setHeaterLevel()](#setHeaterLevel)

[Back to interface](#interface)


<a id="getHeaterEnabled"></a>
## getHeaterEnabled()
#### Description
The method returns the status of the sensor's heater.

#### Syntax
    boolean getHeaterEnabled();

#### Parameters
None

#### Returns
Flag about the heater switched on or off.
- **true**: The heater is on.
- **false**: The heater is off.

#### See also
[setHeaterEnabled()](#setHeater)

[setHeaterDisabled()](#setHeater)

[Back to interface](#interface)


<a id="setHeaterLevel"></a>
## setHeaterLevel()
#### Description
The method sets the heater level if the heater is enabled and determines by that level the heating current.
The heater level is determined by lower 4 bits in the heater control register.
- The sensors of the class A10 have no current selection. They use predefined current at heater enabled and ignore the input parameter.

#### Syntax
    uint8_t setHeaterLevel(uint8_t heaterLevel = 0x0);

#### Parameters
<a id="heaterLevel"></a>
- **heaterLevel**: Desired heater level.
  - *Valid values*: non-negative integer 0 ~ 15
  - *Default value*: 0

#### Returns
Some of result or [error codes](#errors) including ones from the parent library.

#### See also
[setHeaterEnabled()](#setHeater)

[Back to interface](#interface)


<a id="getHeaterLevel"></a>
## getHeaterLevel()
#### Description
The method returns the heater level as an integer if the heater is enabled
- The sensors of the class A10 have no current selection, so that they return only zero heater level.

#### Syntax
    uint8_t getHeaterLevel();

#### Parameters
None

#### Returns
Current heater level in range 0 ~ 15 or some of [error codes](#errors) including ones from the parent library.

#### See also
[setHeaterLevel()](#setHeaterLevel)

[Back to interface](#interface)


<a id="getHeaterCurrent"></a>
## getHeaterCurrent()
#### Description
The method returns the heater direct current in milliamps if the heater is enabled.
- The sensors of the class A10 have no current selection, so that they return constant heater current.

#### Syntax
    float getHeaterCurrent();

#### Parameters
None

#### Returns
Heater current in milliamps corresponding to the heater level or zero value if some error occurs. The error code can be obtained by the method *getLastResult()*.

#### See also
[setHeaterLevel()](#setHeaterLevel)

[getHeaterLevel()](#getHeaterLevel)

[Back to interface](#interface)


<a id="getDeviceType"></a>
## getDeviceType()
#### Description
The method returns the byte, which determines the type of a sensor.

#### Syntax
    uint8_t getDeviceType();

#### Parameters
None

#### Returns
Device type defined by library macro constant [GBJ\_SI70\_TYPE\_7013](#constants), [GBJ\_SI70\_TYPE\_7020](#constants), [[GBJ\_SI70\_TYPE\_7021](#constants)], [GBJ\_SI70\_TYPE\_SAMPLE1](#constants), or [GBJ\_SI70\_TYPE\_SAMPLE2](#constants), or some of [error codes](#errors) including ones from the parent library.

#### See also
[getFwRevision()](#getFwRevision)

[getSerialUpper()](#getSerialUpper)

[getSerialLower()](#getSerialLower)

[Back to interface](#interface)


<a id="getFwRevision"></a>
## getFwRevision()
#### Description
The method returns the firmware revision byte of the sensor, which determines the series of sensors, either *A10* or *A20*.

#### Syntax
    uint8_t getFwRevision();

#### Parameters
None

#### Returns
Firmware revision either [GBJ\_SI70\_FW\_VERSION\_10](#constants) or [GBJ\_SI70\_FW\_VERSION\_20](#constants) or some of [error codes](#errors) including ones from the parent library.

#### See also
[getDeviceType()](#getDeviceType)

[getSerialUpper()](#getSerialUpper)

[getSerialLower()](#getSerialLower)

[Back to interface](#interface)


<a id="getSerial"></a>
## getSerialUpper(), getSerialLower()
#### Description
The particular method returns the corresponding part of the serial number as a double word (32-bit), so that the entire serial number of the sensor is 64-bit value;

#### Syntax
    uint32_t getSerialUpper();
    uint32_t getSerialLower();

#### Parameters
None

#### Returns
Upper or lower serial double word or some of [error codes](#errors) including ones from the parent library.

#### See also
[getDeviceType()](#getDeviceType)

[getFwRevision()](#getFwRevision)

[Back to interface](#interface)


<a id="getVddStatus"></a>
## getVddStatus()
#### Description
The method returns the status of the supply voltage, which the sensor is powered by.

#### Syntax
    boolean getVddStatus();

#### Parameters
None

#### Returns
Flag about the correctness of the operating voltage.
- **true**: The voltage is correct.
- **false**: The voltage is incorrect.

[Back to interface](#interface)
