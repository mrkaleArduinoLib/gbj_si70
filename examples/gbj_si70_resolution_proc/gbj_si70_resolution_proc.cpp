/*
NAME:
List of all possible resolutions of the sensor SI70xx with gbjSI70 library.

DESCRIPTION:
The sketch alters resolution of SI70xx sensor and display it in bits and
respective measurement units.
- Connect sensor's pins to microcontroller's I2C bus as described in README.md
  for used platform accordingly.
- The sketch sets resolution by library methods for demonstration of their
usage.

  LICENSE:
  This program is free software; you can redistribute it and/or modify
  it under the terms of the MIT License (MIT).

  CREDENTIALS:
  Author: Libor Gabaj
*/
#include "gbj_si70.h"

gbj_si70 sensor = gbj_si70();
// gbj_si70 sensor = gbj_si70(sensor.CLOCK_400KHZ);
// gbj_si70 sensor = gbj_si70(sensor.CLOCK_100KHZ, D2, D1);

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
  // List possible resolution
  Serial.println("Temperature and Humidity resolution in bits:");
  for (byte i = 0; i < 4; i++)
  {
    switch (i)
    {
      case 0:
        sensor.setResolutionTemp14();
        // sensor.setResolutionRhum12();
        break;

      case 1:
        sensor.setResolutionTemp13();
        // sensor.setResolutionRhum10();
        break;

      case 2:
        sensor.setResolutionTemp12();
        // sensor.setResolutionRhum8();
        break;

      case 3:
        sensor.setResolutionTemp11();
        // sensor.setResolutionRhum11();
        break;
    }
    if (sensor.isError())
    {
      errorHandler("Resolution");
      continue;
    }
    Serial.print("T: ");
    Serial.print(sensor.getResolutionTemp());
    Serial.print(" - RH: ");
    Serial.println(sensor.getResolutionRhum());
  }
  Serial.println("---");
}

void loop() {}
