#include<Arduino.h>
#include "Libs/CanSender.h"
#include "Constants.h"

CanSender canSystem;
// CAN Variabl es
unsigned long prevTX = 0;
const unsigned int invlTX = 2000;  // Transmission interval (1 second)
byte txData[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x12};  // Data to send
unsigned long messageID = 0x20;


void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  if (canSystem.begin()) Serial.println("Can Activated!");
  

}
void loop() {
  // Send CAN message at regular intervals
  if (millis() - prevTX >= invlTX) {
    prevTX = millis();
    canSystem.send(messageID, 8, txData);
  }
}
