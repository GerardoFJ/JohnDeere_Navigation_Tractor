#include<Arduino.h>
#include "Libs/CanSender.h"
#include "Constants.h"

//Encoder and sending definitions
volatile int32_t counter = 0;
volatile int32_t last_counter = 0;
unsigned long prevTX = 0;
const unsigned int invlTX = 20;

//CAN DEFINITIONS
CanSender canSystem;
byte txData[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // Data to send
unsigned long messageID = 0x20;

//INTERRUPT FUNCTION FOR ENCODER
void ARDUINO_ISR_ATTR encoderISR() {
  counter++;
}

void setup() {
  Serial.begin(115200); //Serial initialization
  pinMode(EncoderPin, INPUT); //Encoder initialization
  attachInterrupt(EncoderPin, encoderISR, RISING); // Interrupt initialization
  if (canSystem.begin()) Serial.println("Can Activated!"); //Activate can
  prevTX = millis(); // Initialize clock
}
void loop() {
       if(counter != last_counter){ //check counter update
       txData[0] = (byte)(counter & 0xFF); 
       txData[1] = (byte)((counter >> 8) & 0xFF);  
       txData[2] = (byte)((counter >> 16) & 0xFF); 
       txData[3] = (byte)((counter >> 24) & 0xFF);
       last_counter == counter;
       }
       if((millis()-prevTX) >= invlTX){
       prevTX = millis();
       canSystem.send(messageID, 8, txData); // Send every 20 ms (50hz) can frame encoder message
       }
}
