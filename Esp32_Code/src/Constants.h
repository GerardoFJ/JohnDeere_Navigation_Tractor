#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h> // main arduino lib

//Encoder pin
#define EncoderPin 13

namespace SystemConfig {

  // --- Pin Definitions Can --
  constexpr int CAN0_INT = 4;
  constexpr int CAN0_CS  = 2; 
  constexpr int SPI_SCK  = 18; 
  constexpr int SPI_MISO = 19; 
  constexpr int SPI_MOSI = 23;
  
  
}

#endif