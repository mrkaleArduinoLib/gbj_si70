/*
  NAME:
  Basic measurement with gbjSI70 library.

  DESCRIPTION:
  The sketch measures humidity and temperature with SI70xx sensor.
  - Connect sensor's pins to microcontroller's I2C bus as described in README.md
    for used platform accordingly.
  - The sketch measures the humidity and temperature at once with one conversion
    at default (highest) resolution.

  LICENSE:
  This program is free software; you can redistribute it and/or modify
  it under the terms of the MIT License (MIT).

  CREDENTIALS:
  Author: Libor Gabaj
*/
#include "gbj_si70.h"

// Time in miliseconds between measurements
const unsigned int PERIOD_MEASURE = 3000;

gbj_si70 sensor = gbj_si70();
// gbj_si70 Sensor = gbj_si70(sensor.CLOCK_400KHZ);
// gbj_si70 Sensor = gbj_si70(sensor.CLOCK_100KHZ, D2, D1);

float tempValue, rhumValue, dewpValue;

void errorHandler(String location)
{
  Serial.println(sensor.getLastErrorTxt(location));
  Serial.println("---");
  return;
}

void setup()
{
  Serial.begin(9600);
  Serial.println("---");

  // Initialize sensor - default holdMasterMode
  if (sensor.isError(sensor.begin()))
  {
    errorHandler("Begin");
    return;
  }
  if (sensor.isError(sensor.setResolutionTemp12()))
  {
    errorHandler("Resolution");
    return;
  }
  Serial.println("Humidity (%) / Temperature (°C)");
}

void loop()
{
  rhumValue = sensor.measureHumidity(tempValue);
  if (sensor.isError())
  {
    errorHandler("Measurement");
  }
  Serial.print(rhumValue);
  Serial.print(" / ");
  Serial.println(tempValue);
  delay(PERIOD_MEASURE);
}
