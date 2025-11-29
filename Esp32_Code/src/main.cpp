#include<Arduino.h>
#include "Libs/CanSender.h"
#include "Constants.h"

#define EncoderPin 13

volatile int32_t counter = 0;
volatile int32_t last_counter = 0;
CanSender canSystem;

// CAN Variable
unsigned long prevTX = 0;
const unsigned int invlTX = 50;  // Transmission interval (1 second)
byte txData[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // Data to send
unsigned long messageID = 0x20;

void ARDUINO_ISR_ATTR encoderISR() {
  counter++;
}

void setup() {
  Serial.begin(115200);
  pinMode(EncoderPin, INPUT);
  attachInterrupt(EncoderPin, encoderISR, RISING);
  if (canSystem.begin()) Serial.println("Can Activated!");
  prevTX = millis();
}
void loop() {
       if(counter != last_counter){
       txData[0] = (byte)(counter & 0xFF); 
       txData[1] = (byte)((counter >> 8) & 0xFF);  
       txData[2] = (byte)((counter >> 16) & 0xFF); 
       txData[3] = (byte)((counter >> 24) & 0xFF);
       last_counter == counter;
       }
       if((millis()-prevTX) >= invlTX){
       prevTX = millis();
       Serial.print("Sending CanMessage");
       canSystem.send(messageID, 8, txData);
       }
}
//145
