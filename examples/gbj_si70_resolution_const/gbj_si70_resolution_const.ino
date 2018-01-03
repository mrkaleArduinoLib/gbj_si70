/*
  NAME:
  List of all possible resolutions of the sensor SI70xx with gbj_si70 library.

  DESCRIPTION:
  The sketch alters resolution of SI70xx sensor and display it in bits and
  respective measurement units.
  - Connect sensor's pins to microcontroller's I2C bus as described in README.md
    for used platform accordingly.
  - The sketch sets resolution by library macro constants for demonstration of
    their usage.

  LICENSE:
  This program is free software; you can redistribute it and/or modify
  it under the terms of the MIT License (MIT).

  CREDENTIALS:
  Author: Libor Gabaj
*/
#include "gbj_si70.h"
#define SKETCH "GBJ_SI70_RESOLUTION_CONST 1.0.0"

gbj_si70 Sensor = gbj_si70();
byte resolutionList[4] = {GBJ_SI70_RES_T14_RH12, GBJ_SI70_RES_T13_RH10, \
  GBJ_SI70_RES_T12_RH8, GBJ_SI70_RES_T11_RH11};


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
  // List possible resolution
  Serial.println("Resolution in bits:");
  for (byte i = 0; i < (sizeof(resolutionList) / sizeof(resolutionList[0])); i++)
  {
    Sensor.setResolution(resolutionList[i]);
    if (Sensor.isSuccess())
    {
      Serial.print("T_");
      Serial.print(Sensor.getResolutionTemp());
      Serial.print("-RH_");
      Serial.println(Sensor.getResolutionRhum());
    }
    errorHandler();
  }
}


void loop() {}
