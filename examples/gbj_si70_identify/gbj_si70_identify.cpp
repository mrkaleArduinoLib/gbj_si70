/*
  NAME:
  Using the gbjSI70 library for identifying the sensor SI70xx.

  DESCRIPTION:
  The sketch displays all identification and status data stored in the sensor.
  - Connect sensor's pins to microcontroller's I2C bus as described in README.md
    for used platform accordingly.

  LICENSE:
  This program is free software; you can redistribute it and/or modify
  it under the terms of the MIT License (MIT).

  CREDENTIALS:
  Author: Libor Gabaj
*/
#include "gbj_si70.h"

gbj_si70 sensor = gbj_si70();
// gbj_si70 sensor = gbj_si70(sensor.CLOCK_100KHZ, D2, D1);
// gbj_si70 sensor = gbj_si70(sensor.CLOCK_400KHZ);

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
  // Device name
  Serial.print("Sensor: ");
  switch (sensor.getDeviceType())
  {
    case sensor.TYPE_7021:
      Serial.print("Si7021");
      break;
    case sensor.TYPE_7020:
      Serial.print("Si7020");
      break;
    case sensor.TYPE_7013:
      Serial.print("Si7013");
      break;
    case sensor.TYPE_SAMPLE1:
    case sensor.TYPE_SAMPLE2:
      Serial.print("Sample");
      break;
    default:
      Serial.print("Unknown");
      break;
  }
  switch (sensor.getFwRevision())
  {
    case sensor.FW_VERSION_10:
      Serial.print("-A10");
      break;
    case sensor.FW_VERSION_20:
      Serial.print("-A20");
      break;
    default:
      Serial.print("-???");
      break;
  }
  Serial.println();
  // Address
  Serial.print("Address: 0x");
  Serial.println(sensor.getAddress(), HEX);
  // Device type
  Serial.print("Device Type: 0x");
  Serial.println(sensor.getDeviceType(), HEX);
  // Firmware revision
  Serial.print("Firmware Revision: 0x");
  Serial.println(sensor.getFwRevision(), HEX);
  // Serial number
  char text[30];
  snprintf(
    text, 30, "0x%08lx-%08lx", (long)sensor.getSNA(), (long)sensor.getSNB());
  Serial.print("Serial Number (SNA-SNB): ");
  Serial.println(text);
  // Vdd status
  Serial.print("Vdd Status: ");
  Serial.println((sensor.getVddStatus() ? "OK" : "LOW"));
  // Heater status
  bool heaterStatusOrig = sensor.getHeaterEnabled();
  if (heaterStatusOrig || sensor.isSuccess(sensor.setHeaterEnabled()))
  {
    Serial.print("Heater Current: ");
    Serial.print(sensor.getHeaterCurrent());
    Serial.println(" mA");
  }
  if (!heaterStatusOrig)
  {
    sensor.setHeaterDisabled();
  }
  if (sensor.isError())
  {
    errorHandler("Heater");
  }
  Serial.print("Heater Status: ");
  Serial.println(sensor.getHeaterEnabled() ? "Enabled" : "Disabled");
  // Temperature resolution
  Serial.print("Temperature Resolution: ");
  Serial.print(sensor.getResolutionTemp());
  Serial.println(" bits");
  // Humidity resolution
  Serial.print("Humidity Resolution: ");
  Serial.print(sensor.getResolutionRhum());
  Serial.println(" bits");
  Serial.println("---");
}

void loop() {}
