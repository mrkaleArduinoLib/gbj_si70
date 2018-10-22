<a id="library"></a>
# gbjSI70
Library for the humidity and temperature sensors *SI70xx*, especially **SI7021** on board GY-21 communicating on two-wire (I2C) bus.
- Sensor address is `0x40` hardcoded and cannot be changed by any library method.

#### Particle hardware configuration
- Connect microcontroller's pin `D0` to sensor's pin **SDA** (Serial Data).
- Connect microcontroller's pin `D1` to sensor's pin **SCL** (Serial Clock).

#### Arduino UNO hardware configuration
- Connect microcontroller's pin `A4` to sensor's pin **SDA** (Serial Data).
- Connect microcontroller's pin `A5` to sensor's pin **SCL** (Serial Clock).

#### Espressif - ESP8266, ESP32 default hardware configuration
- Connect microcontroller's pin `D2` to sensor's pin **SDA** (Serial Data).
- Connect microcontroller's pin `D1` to sensor's pin **SCL** (Serial Clock).


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
- **gbj\_si70::VERSION**: Name and semantic version of the library.

<a id="type"></a>
#### Identification codes
- **gbj\_si70::TYPE\_SAMPLE1**: Sensor type Engineering samples.
- **gbj\_si70::TYPE\_SAMPLE2**: Sensor type Engineering samples.
- **gbj\_si70::TYPE\_7013**: Sensor type Si7013.
- **gbj\_si70::TYPE\_7020**: Sensor type Si7020.
- **gbj\_si70::TYPE\_7021**: Sensor type Si7021.

<a id="resolution"></a>
#### Measurement resolutions
- **gbj\_si70::RESOLUTION\_T14\_RH12**: Resolution of temperature 14 bits, relative humidity 12 bits.
- **gbj\_si70::RESOLUTION\_T13\_RH10**: Resolution of temperature 13 bits, relative humidity 10 bits.
- **gbj\_si70::RESOLUTION\_T12\_RH8**: Resolution of temperature 12 bits, relative humidity 8 bits.
- **gbj\_si70::RESOLUTION\_T11\_RH11**: Resolution of temperature 11 bits, relative humidity 11 bits.

<a id="firmware"></a>
#### Firmware versions
- **gbj\_si70::FW\_VERSION\_10**: Firmware revision byte for sensors of the series *A10*.
- **gbj\_si70::FW\_VERSION\_20**: Firmware revision byte for sensors of the series *A20*.


<a id="errors"></a>
#### Error codes
- **gbj\_si70::ERR\_ADDRESS**: Bad address.
- **gbj\_si70::ERR\_RESET**: Sensor reset failure.
- **gbj\_si70::ERR\_FIRMWARE**: Firmware revision reading failure.
- **gbj\_si70::ERR\_SERIAL\_A**: Serial number upper double word reading failure.
- **gbj\_si70::ERR\_SERIAL\_B**: Serial number upper double word reading failure.
- **gbj\_si70::ERR\_REG\_RHT\_READ**: Reading RH/T User Register 1 failure.
- **gbj\_si70::ERR\_REG\_HEATER\_READ**: Reading Heater Control Register failure.
- **gbj\_si70::ERR\_MEASURE\_RHUM**: Measuring relative humidity failure.
- **gbj\_si70::ERR\_MEASURE\_TEMP**: Measuring temperature failure.

Other error codes as well as result code are inherited from the parent library [gbjTwoWire](#dependency).


<a id="interface"></a>
## Interface

#### Main
- [gbj_si70()](#gbj_si70)
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
- [setHoldMasterMode()](#setHoldMasterMode)

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
- [getHoldMasterMode()](#getHoldMasterMode)

Other possible setters and getters are inherited from the parent library [gbjTwoWire](#dependency) and described there.


<a id="gbj_si70"></a>
## gbj_si70()
#### Description
The library does not need special constructor and destructor, so that the inherited ones are good enough and there is no need to define them in the library, just use it with default or specific parameters as defined at constructor of parent library [gbjTwoWire](#dependency).
- Constructor sets parameters specific to the two-wire bus in general.
- All the constructor parameters can be changed dynamically with corresponding setters later in a sketch.

#### Syntax
    gbj_si70(uint32_t clockSpeed, bool busStop, uint8_t pinSDA, uint8_t pinSCL);

#### Parameters
<a id="prm_busClock"></a>
- **clockSpeed**: Two-wire bus clock frequency in Hertz. If the clock is not from enumeration, it fallbacks to 100 kHz.
  - *Valid values*: gbj\gbj_si70::CLOCK\_100KHZ, gbj\gbj_si70::CLOCK\_400KHZ
  - *Default value*: gbj\gbj_si70::CLOCK\_100KHZ


<a id="prm_busStop"></a>
- **busStop**: Logical flag about releasing bus after end of transmission.
  - *Valid values*: true, false
    - **true**: Releases the bus after data transmission and enables other master devices to control the bus.
    - **false**: Keeps connection to the bus and enables to begin further data transmission immediately.
  - *Default value*: true


<a id="prm_pinSDA"></a>
- **pinSDA**: Microcontroller's pin for serial data. It is not a board pin but GPIO number. For hardware two-wire bus platforms it is irrelevant and none of methods utilizes this parameter for such as platforms for communication on the bus. On the other hand, for those platforms the parameters might be utilized for storing some specific attribute in the class instance object.
  - *Valid values*: positive integer
  - *Default value*: 4 (GPIO4, D2)


<a id="prm_pinSCL"></a>
- **pinSCL**: Microcontroller's pin for serial clock. It is not a board pin but GPIO number. For hardware two-wire bus platforms it is irrelevant and none of methods utilizes this parameter for such as platforms. On the other hand, for those platforms the parameters might be utilized for storing some specific attribute in the class instance object.
  - *Valid values*: positive integer
  - *Default value*: 5 (GPIO5, D1)

#### Returns
Object performing the sensor management.
The constructor cannot return [a result or error code](#constants) directly, however, it stores them in the instance object. The result can be tested in the operational code with the method [getLastResult()](#getLastResult), [isError()](#isError), or [isSuccess()](#isSuccess).

#### Example
The method has all arguments defaulted and calling without any parameters is equivalent to the calling with all arguments set by corresponding constant with default value:

```cpp
  gbj_si70 Sensor = gbj_si70(); // It is equivalent to
  gbj_si70 Sensor = gbj_si70(gbj_si70::CLOCK_100KHZ, true, D2, D1);
```

[Back to interface](#interface)


<a id="begin"></a>
## begin()
#### Description
The method takes, sanitizes, and stores sensor parameters to a class instance object and initiates two-wire bus.
- The method sets parameters specific to the sensor itself.
- All the method parameters can be changed dynamically with corresponding [setters](#interface) later in a sketch.

#### Syntax
    uint8_t begin(boolean holdMasterMode);

#### Parameters
<a id="holdMasterMode"></a>
- **holdMasterMode**: Logical flag about blocking (holding) serial clock line during measurement. At no holding master mode other communication on the bus can be performed.
  - *Valid values*: true, false
  - *Default value*: true

#### Returns
Some of [result or error codes](#constants).

#### See also
[setHoldMasterMode()](#setHoldMasterMode)

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
Some of [result or error codes](#constants).

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
Some of [result or error codes](#constants).

[Back to interface](#interface)


<a id="measureHumidity"></a>
## measureHumidity()
#### Description
The method is overloaded and measures either relative humidity alongside with temperature at once or the humidity alone.
The temperature is returned through referenced input parameter.

#### Syntax
    float measureHumidity();
    float measureHumidity(float *temperature);

#### Parameters
<a id="temperature"></a>
- **temperature**: Address reference to output variable for measured temperature in centigrade.
  - *Valid values*: address range specific for a platform
  - *Default value*: none

#### Returns
- Relative humidity or the error value [gbj\_si70::ERROR\_MEASURE\_RHUM](#errors) with corresponding error code in the library object.
- Temperature in centigrade in the referenced variable.

#### Example
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
    float measureTemperature();

#### Parameters
None

#### Returns
Temperature in centigrade or the error value [gbj\_si70::ERROR\_MEASURE\_TEMP](#errors) with corresponding error code in the library object.

#### Example
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
The method sets the bit resolution by input parameter, which should be appropriate library [constant](#resolution).
The resolution is determined by that constant but in fact it is the bit resolution for temperature.

#### Syntax
    uint8_t setResolution(uint8_t resolution = gbj_si70::RESOLUTION_T14_RH12);

#### Parameters
<a id="resolution"></a>
- **resolution**: Desired measurement resolution in bits.
  - *Valid values*:  [gbj\_si70::RESOLUTION\_T14\_RH12](#resolution), [gbj\_si70::RESOLUTION\_T13\_RH10](#resolution),  [gbj\_si70::RESOLUTION\_T12\_RH8](#resolution), or [gbj\_si70::RESOLUTION\_T11\_RH11](#resolution)
  - *Default value*: [gbj\_si70::RESOLUTION\_T14\_RH12](#resolution)

#### Returns
Some of [result or error codes](#constants).

#### See also
[setResolutionTemp11(), setResolutionTemp12(), setResolutionTemp13(), setResolutionTemp14()](#setResolutionTemp)

[setResolutionRhum8(), setResolutionRhum10(), setResolutionRhum11(), setResolutionRhum12()](#setResolutionRhum)

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
Some of [result or error codes](#constants).

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
Bit resolution (11, 12, 13, or 14) or some of [error codes](#errors).

#### See also
[setResolution()](#setResolution)

[setResolutionTemp11(), setResolutionTemp12(), setResolutionTemp13(), setResolutionTemp14()](#setResolutionTemp)

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
Some of [result or error codes](#constants).

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
Bit resolution (8, 10, 11, or 12) or some of [error codes](#errors).

#### See also
[setResolution()](#setResolution)

[setResolutionRhum8(), setResolutionRhum10(), setResolutionRhum11(), setResolutionRhum12()](#setResolutionRhum)

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
Some of [result or error codes](#constants).

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
Some of [result or error codes](#constants).

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
Current heater level in range 0 ~ 15 or some of [error codes](#errors).

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
Device type defined by library macro constant [gbj\_si70::TYPE\_7013](#type), [gbj\_si70::TYPE\_7020](#type), [[gbj\_si70::TYPE\_7021](#type)], [gbj\_si70::TYPE\_SAMPLE1](#type), or [gbj\_si70::TYPE\_SAMPLE2](#type), or some of [error codes](#errors) including ones from the parent library.

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
Firmware revision either [gbj\_si70::FW\_VERSION\_10](#firmware) or [gbj\_si70::FW\_VERSION\_20](#firmware) or some of [error codes](#errors) including ones from the parent library.

#### See also
[getDeviceType()](#getDeviceType)

[getSerialUpper()](#getSerialUpper)

[getSerialLower()](#getSerialLower)

[Back to interface](#interface)


<a id="getSerial"></a>
## getSerialUpper(), getSerialLower()
#### Description
The particular method returns the corresponding part of the serial number as a double word (32-bit), so that the entire serial number of the sensor is 64-bit value.

#### Syntax
    uint32_t getSerialUpper();
    uint32_t getSerialLower();

#### Parameters
None

#### Returns
Upper or lower serial double word or some of [error codes](#errors).

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
