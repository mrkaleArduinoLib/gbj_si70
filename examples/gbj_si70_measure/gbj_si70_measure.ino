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
#define SKETCH "GBJ_SI70_MEASURE 1.0.0"

#include "gbj_si70.h"

const unsigned int PERIOD_MEASURE = 3000;      // Time in miliseconds between measurements

gbj_si70 Sensor = gbj_si70();
// gbj_si70 Sensor = gbj_si70(gbj_si70::CLOCK_100KHZ, true, D2, D1);
// gbj_si70 Sensor = gbj_si70(gbj_si70::CLOCK_400KHZ);

float tempValue, rhumValue;


void errorHandler()
{
  if (Sensor.isSuccess()) return;
  Serial.print("Error: ");
  Serial.print(Sensor.getLastResult());
  Serial.print(" - ");
  switch (Sensor.getLastResult())
  {
    case gbj_si70::ERROR_ADDRESS:
      Serial.println("Bad address");
      break;

    case gbj_twowire::ERROR_PINS:
      Serial.println("Bad pins");
      break;

    case gbj_si70::ERROR_NACK_OTHER:
      Serial.println("Other error");
      break;

    default:
      Serial.println("Uknown error");
      break;
  }
}


void setup()
{
  Serial.begin(9600);
  Serial.println(SKETCH);
  Serial.println("Libraries:");
  Serial.println(gbj_twowire::VERSION);
  Serial.println(gbj_si70::VERSION);
  Serial.println("---");

  // Initialize Sensor
  if (Sensor.begin()) // Use default holdMasterMode
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
