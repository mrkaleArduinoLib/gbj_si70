/*
  NAME:
  Only temperature measurement with gbj_si70 library.

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
#define SKETCH "GBJ_SI70_TEMPERATURE 1.0.0"

const unsigned int PERIOD_MEASURE = 3000;      // Time in miliseconds between measurements

gbj_si70 Sensor = gbj_si70();
float tempValue;


void errorHandler()
{
  if (Sensor.isSuccess()) return;
  Serial.print("Error: ");
  Serial.println(Sensor.getLastResult());
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
  Serial.println("Temperature ('C)");
}


void loop()
{
  if (Sensor.isError()) return;
  tempValue = Sensor.measureTemperature();
  if (Sensor.isSuccess())
  {
    Serial.println(tempValue);
  }
  errorHandler();
  delay(PERIOD_MEASURE);
}
