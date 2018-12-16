/*
NAME:
List of all possible resolutions of the sensor SI70xx with gbjSI70 library.

DESCRIPTION:
The sketch alters resolution of SI70xx sensor and display it in bits and
respective measurement units.
- Connect sensor's pins to microcontroller's I2C bus as described in README.md
  for used platform accordingly.
- The sketch sets resolution by library methods for demonstration of their usage.

  LICENSE:
  This program is free software; you can redistribute it and/or modify
  it under the terms of the MIT License (MIT).

  CREDENTIALS:
  Author: Libor Gabaj
*/
#define SKETCH "GBJ_SI70_RESOLUTION_PROC 1.0.0"

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
  // List possible resolution
  Serial.println("Temperature and Humidity resolution in bits:");
  for (byte i = 0; i < 4; i++)
  {
    switch (i)
    {
      case 0:
        Sensor.setResolutionTemp14();
        // Sensor.setResolutionRhum12();
        break;

      case 1:
        Sensor.setResolutionTemp13();
        // Sensor.setResolutionRhum10();
        break;

      case 2:
        Sensor.setResolutionTemp12();
        // Sensor.setResolutionRhum8();
        break;

      case 3:
        Sensor.setResolutionTemp11();
        // Sensor.setResolutionRhum11();
        break;
    }
    if (Sensor.isSuccess())
    {
      Serial.print("T: ");
      Serial.print(Sensor.getResolutionTemp());
      Serial.print(" - RH: ");
      Serial.println(Sensor.getResolutionRhum());
    }
    errorHandler("Resolution");
  }
  Serial.println("---");
  Serial.println("END");
}


void loop() {}
