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
#define SKETCH "GBJ_SI70_IDENTIFY 1.1.0"

#include "gbj_si70.h"


gbj_si70 Sensor = gbj_si70();
// gbj_si70 Sensor = gbj_si70(gbj_si70::CLOCK_100KHZ, D2, D1);
// gbj_si70 Sensor = gbj_si70(gbj_si70::CLOCK_400KHZ);


void errorHandler(String location)
{
  if (Sensor.isSuccess()) return;
  Serial.print(location);
  Serial.print(" - Error: ");
  Serial.print(Sensor.getLastResult());
  Serial.print(" - ");
  switch (Sensor.getLastResult())
  {
    // General
    case gbj_si70::ERROR_ADDRESS:
      Serial.println("ERROR_ADDRESS");
      break;

    case gbj_si70::ERROR_PINS:
      Serial.println("ERROR_PINS");
      break;

    case gbj_si70::ERROR_RCV_DATA:
      Serial.println("ERROR_RCV_DATA");
      break;

    // Arduino, Esspressif specific
#if defined(__AVR__) || defined(ESP8266) || defined(ESP32)
    case gbj_si70::ERROR_BUFFER:
      Serial.println("ERROR_BUFFER");
      break;

    case gbj_si70::ERROR_NACK_DATA:
      Serial.println("ERROR_NACK_DATA");
      break;

    case gbj_si70::ERROR_NACK_OTHER:
      Serial.println("ERROR_NACK_OTHER");
      break;

    // Particle specific
#elif defined(PARTICLE)
    case gbj_si70::ERROR_BUSY:
      Serial.println("ERROR_BUSY");
      break;

    case gbj_si70::ERROR_END:
      Serial.println("ERROR_END");
      break;

    case gbj_si70::ERROR_TRANSFER:
      Serial.println("ERROR_TRANSFER");
      break;

    case gbj_si70::ERROR_TIMEOUT:
      Serial.println("ERROR_TIMEOUT");
      break;
#endif

    case gbj_si70::ERROR_RESET:
      Serial.println("ERROR_RESET");
      break;

    case gbj_si70::ERROR_FIRMWARE:
      Serial.println("ERROR_FIRMWARE");
      break;

    case gbj_si70::ERROR_SERIAL_A:
      Serial.println("ERROR_SERIAL_A");
      break;

    case gbj_si70::ERROR_SERIAL_B:
      Serial.println("ERROR_SERIAL_B");
      break;

    case gbj_si70::ERROR_REG_RHT_READ:
      Serial.println("ERROR_REG_RHT_READ");
      break;

    case gbj_si70::ERROR_REG_HEATER_READ:
      Serial.println("ERROR_REG_HEATER_READ");
      break;

    case gbj_si70::ERROR_MEASURE_RHUM:
      Serial.println("ERROR_MEASURE_RHUM");
      break;

    case gbj_si70::ERROR_MEASURE_TEMP:
      Serial.println("ERROR_MEASURE_TEMP");
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
    errorHandler("Begin");
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
  snprintf(text, 30, "0x%08lx-%08lx", (long) Sensor.getSNA(), (long) Sensor.getSNB());
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
  Serial.println("---");
  Serial.println("END");
}


void loop() {}
