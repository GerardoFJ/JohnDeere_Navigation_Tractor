#include "CanSender.h"


CanSender::CanSender() :
          baudRate(CAN_500KBPS),
          frequency(MCP_8MHZ),
          mode(MCP_NORMAL),
          canInt(SystemConfig::CAN0_INT),
          canMiso(SystemConfig::SPI_MISO),
          canMosi(SystemConfig::SPI_MOSI),
          canSck(SystemConfig::SPI_SCK),
          canBus(SystemConfig::CAN0_CS),
          csPin(SystemConfig::CAN0_CS) {}
               


CanSender::CanSender(
              int baudRate,
               int frequency,
               int mode,
               int canInt,
               int canMiso,
               int canMosi,
               int canSck,
               int csPin
              ) :
          baudRate(baudRate),
          frequency(frequency),
          mode(mode),
          canInt(canInt),
          canMiso(canMiso),
          canMosi(canMosi),
          canSck(canSck),
          canBus(csPin),
          csPin(csPin) { }

bool CanSender::begin() {
      SPI.begin(canSck,canMiso , canMosi, csPin);  //SPI initialization
      pinMode(canInt, INPUT);  // Pinout can initialization
      if (canBus.begin(MCP_ANY, baudRate, frequency) == CAN_OK) { //Can actiavte
        canBus.setMode(mode); //Can mode setup
        return true;
      } 
      return false;
    }

bool CanSender::send(unsigned long canId, byte length, byte* data) {
      byte sndStat = canBus.sendMsgBuf(canId, 0, length, data); //can sending data
      if (sndStat == CAN_OK) {
        return true;
      } else {
        return false;
      }
    }
