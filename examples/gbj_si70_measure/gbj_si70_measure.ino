/*
  NAME:
  Basic measurement with gbj_si70 library.

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
#define SKETCH "GBJ_SI70_MEASURE 1.0.0"

const unsigned int PERIOD_MEASURE = 3000;      // Time in miliseconds between measurements

gbj_si70 Sensor = gbj_si70();
float tempValue, rhumValue;


void errorHandler()
{
  if (Sensor.isSuccess()) return;
  Serial.print("Error: ");
  Serial.print(Sensor.getLastResult());
  Serial.println(")");
}


void setup()
{
  Serial.begin(9600);
  Serial.println(SKETCH);
  Serial.println("Libraries:");
  Serial.println(GBJ_TWOWIRE_VERSION);
  Serial.println(GBJ_SI70_VERSION);
  Serial.println("---");

  // Initialize Sensor
  if (Sensor.begin()) // Use default bus stop flag
  {
    errorHandler();
    return;
  }
  if (Sensor.setResolutionTemp12())
  {
    errorHandler();
    return;
  }
  Serial.println("Temperature ('C) / Humidity (%)");
}


void loop()
{
  if (Sensor.isError()) return;
  rhumValue = Sensor.measureHumidity(&tempValue);
  if (Sensor.isSuccess())
  {
    Serial.print(tempValue);
    Serial.print(" / ");
    Serial.println(rhumValue);
  }
  errorHandler();
  delay(PERIOD_MEASURE);
}
