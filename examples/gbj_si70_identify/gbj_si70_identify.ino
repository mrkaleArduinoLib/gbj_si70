/*
  NAME:
  Using the gbj_si70 library for identifying the sensor SI70xx.

  DESCRIPTION:
  The sketch displays all identification and status data stored in the Sensor.
  - Connect Sensor's pins to microcontroller's I2C bus as described in README.md
    for used platform accordingly.

  LICENSE:
  This program is free software; you can redistribute it and/or modify
  it under the terms of the MIT License (MIT).

  CREDENTIALS:
  Author: Libor Gabaj
*/
#include "gbj_si70.h"
#define SKETCH "GBJ_SI70_IDENTIFY 1.0.0"

gbj_si70 Sensor = gbj_si70();


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
  // Device name
  Serial.print("Sensor: ");
  switch (Sensor.getDeviceType())
  {
    case GBJ_SI70_TYPE_7021:
      Serial.print("Si7021");
      break;
    case GBJ_SI70_TYPE_7020:
      Serial.print("Si7020");
      break;
    case GBJ_SI70_TYPE_7013:
      Serial.print("Si7013");
      break;
    case GBJ_SI70_TYPE_SAMPLE1:
    case GBJ_SI70_TYPE_SAMPLE2:
      Serial.print("Sample");
      break;
    default:
      Serial.print("Unknown");
      break;
  }
  switch (Sensor.getFwRevision())
  {
    case GBJ_SI70_FW_VERSION_10:
      Serial.print("-A10");
      break;
    case GBJ_SI70_FW_VERSION_20:
      Serial.print("-A20");
      break;
    default:
      Serial.print("-???");
      break;
  }
  Serial.println();
  // Address
  Serial.print("Address: 0x");
  Serial.println(Sensor.getAddress(), HEX);
  // Device type
  Serial.print("Device Type: 0x");
  Serial.println(Sensor.getDeviceType(), HEX);
  // Firmware revision
  Serial.print("Firmware Revision: 0x");
  Serial.println(Sensor.getFwRevision(), HEX);
  // Serial number
  char text[30];
  snprintf(text, 30, "0x%08lX - 0x%08lX", Sensor.getSerialUpper(), Sensor.getSerialLower());
  Serial.print("Serial Number: ");
  Serial.println(text);
  // Vdd status
  Serial.print("Vdd Status: ");
  Serial.println((Sensor.getVddStatus() ? "OK" : "LOW"));
  // Heater status
  Serial.print("Heater: ");
  Serial.println((Sensor.getHeaterEnabled() ? "Enabled" : "Disabled"));
  // Temperature resolution
  Serial.print("Temperature Resolution: ");
  Serial.print(Sensor.getResolutionTemp());
  Serial.println(" bits");
  // Humidity resolution
  Serial.print("Humidity Resolution: ");
  Serial.print(Sensor.getResolutionRhum());
  Serial.println(" bits");
  Serial.println(F("---"));
}


void loop() {}
