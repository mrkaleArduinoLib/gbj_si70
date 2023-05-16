/*
  NAME:
  Only temperature measurement with gbjSI70 library.

  DESCRIPTION:
  The sketch measures humidity and temperature with SI70xx sensor.
  - Connect sensor's pins to microcontroller's I2C bus as described in README.md
    for used platform accordingly.
  - The sketch measures the temperature only without humidity measurement
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
// gbj_si70 sensor = gbj_si70(sensor.CLOCK_400KHZ);
// gbj_si70 sensor = gbj_si70(sensor.CLOCK_100KHZ, D2, D1);

float tempValue;

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
  Serial.println("Temperature ('C)");
}

void loop()
{
  tempValue = sensor.measureTemperature();
  if (sensor.isError())
  {
    errorHandler("Measurement");
  }
  Serial.println(tempValue);
  delay(PERIOD_MEASURE);
}
