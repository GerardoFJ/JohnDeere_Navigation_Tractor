#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h> // main arduino lib

//Encoder pin
#define EncoderPin 13
#define EncoderID 0x20
#define BnoID 0x21
#define BluetoothID 0x22

//ble Settings
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define DEBUGGING true

namespace SystemConfig {

  // --- Pin Definitions Can --
  constexpr int CAN0_INT = 4;
  constexpr int CAN0_CS  = 2; 
  constexpr int SPI_SCK  = 18; 
  constexpr int SPI_MISO = 19; 
  constexpr int SPI_MOSI = 23;
  
}

#endif