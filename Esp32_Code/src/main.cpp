#include<Arduino.h>
#include "Libs/CanSender.h"
#include "Constants.h"
#include "Libs/BnoWrap.h"
#include "Libs/ble.h"

//Encoder and sending definitions
volatile int32_t counter = 0;
volatile int32_t last_counter = 0;
volatile float yaw_val = 0;
volatile float yaw_val_prev = 0;
unsigned long prevTX = 0;
const unsigned int invlTX = 20;

//CAN DEFINITIONS
CanSender canSystem;
byte EncoderData[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // Data to send
byte BnoData[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte CameraData[]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

//BNO DEFINITIONS
BnoWrapper bnoSensor;
BleModule bluetooth(CameraData);


//INTERRUPT FUNCTION FOR ENCODER
void ARDUINO_ISR_ATTR encoderISR() {
  counter++;
}

void setup() {
  Serial.begin(115200); //Serial initialization
  pinMode(EncoderPin, INPUT); //Encoder initialization
  attachInterrupt(EncoderPin, encoderISR, RISING); // Interrupt initialization
  if (canSystem.begin()) Serial.println("Can Activated!"); //Activate can
  if (bnoSensor.begin()) Serial.println("Bno Activated");
  if (bluetooth.begin()) Serial.println("Bluetooth activated");
  prevTX = millis(); // Initialize clock
}
void loop() {
       if(counter != last_counter){ //check counter update
       EncoderData[0] = (byte)(counter & 0xFF); 
       EncoderData[1] = (byte)((counter >> 8) & 0xFF);  
       EncoderData[2] = (byte)((counter >> 16) & 0xFF); 
       EncoderData[3] = (byte)((counter >> 24) & 0xFF);
        last_counter = counter;
       }

       yaw_val = bnoSensor.getYaw();
       if(yaw_val != yaw_val_prev){ // check yaw update
         memcpy(&BnoData[0], (const void *)&yaw_val, sizeof(yaw_val)); //copy value to BnoData
         yaw_val_prev = yaw_val;
       }

       if((millis()-prevTX) >= invlTX){
       prevTX = millis();  
       canSystem.send((unsigned long)EncoderID, 8, EncoderData); // Send every 20 ms (50hz) can frame encoder message
       canSystem.send((unsigned long)BnoID, 8, BnoData);
       if(bluetooth.status()){
        canSystem.send((unsigned long)BluetoothID, 8, CameraData);
       } 
       }
}
