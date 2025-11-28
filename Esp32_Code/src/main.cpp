#include<Arduino.h>
#include "Libs/CanSender.h"
#include "Constants.h"

#define EncoderPin 13

volatile int32_t counter = 0;
int32_t 

CanSender canSystem;

// CAN Variabl es
unsigned long prevTX = 0;
const unsigned int invlTX = 100;  // Transmission interval (1 second)
byte txData[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // Data to send
unsigned long messageID = 0x20;

void ARDUINO_ISR_ATTR encoderISR() {
  counter++;
}

void setup() {
  Serial.begin(115200);
  pinMode(EncoderPin, INPUT);
  attachInterrupt(EncoderPin, encoderISR, RISING);
  while (!Serial) delay(10);
  if (canSystem.begin()) Serial.println("Can Activated!");
  

}
void loop() {
  // Send CAN message at regular intervals
  if (counter >= 138) {
    if((millis() - prevTX) >= invlTX)
    prevTX = millis();
    {
      txData[0] = 0x02;
    }
    canSystem.send(messageID, 8, txData);
    
    
  }
  Serial.println(counter);
  
}
//145
