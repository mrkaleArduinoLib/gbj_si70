/*
  NAME:
  Using the gbjSI70 library for identifying the sensor SI70xx.

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
#define SKETCH "GBJ_SI70_IDENTIFY 1.0.0"

#include "gbj_si70.h"


gbj_si70 Sensor = gbj_si70();
// gbj_si70 Sensor = gbj_si70(gbj_si70::CLOCK_100KHZ, true, D2, D1);
// gbj_si70 Sensor = gbj_si70(gbj_si70::CLOCK_400KHZ);


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
  // Device name
  Serial.print("Sensor: ");
  switch (Sensor.getDeviceType())
  {
    case gbj_si70::TYPE_7021:
      Serial.print("Si7021");
      break;
    case gbj_si70::TYPE_7020:
      Serial.print("Si7020");
      break;
    case gbj_si70::TYPE_7013:
      Serial.print("Si7013");
      break;
    case gbj_si70::TYPE_SAMPLE1:
    case gbj_si70::TYPE_SAMPLE2:
      Serial.print("Sample");
      break;
    default:
      Serial.print("Unknown");
      break;
  }
  switch (Sensor.getFwRevision())
  {
    case gbj_si70::FW_VERSION_10:
      Serial.print("-A10");
      break;
    case gbj_si70::FW_VERSION_20:
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
  snprintf(text, 30, "0x%08lx - 0x%08lx", (long) Sensor.getSerialUpper(), (long) Sensor.getSerialLower());
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
