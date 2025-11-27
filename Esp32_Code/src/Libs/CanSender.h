#ifndef CANSENDER_H
#define CANSENDER_H

#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include "Constants.h"

class CanSender {
  private:
    MCP_CAN canBus;  
    int csPin;
    int baudRate;
    int frequency;
    int mode;
    int canInt;
    int canMiso;
    int canMosi;
    int canSck;

  public:
    CanSender();
    CanSender(int baudRate,
               int frequency,
               int mode,
               int canInt,
               int canMiso,
               int canMosi,
               int canSck,
               int csPin);
  
   bool begin();

   bool send(unsigned long canId, byte length, byte* data);
};
#endif