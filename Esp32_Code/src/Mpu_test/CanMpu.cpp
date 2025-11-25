#include<Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include "I2Cdev.h"
#include "MPU6050.h"


// MPU VARIABLES
MPU6050 mpu;
#define SDA_PIN 21
#define SCL_PIN 22

unsigned long prevTX = 0;
const unsigned int invlTX = 2000;

int16_t ax, ay, az;
int16_t gx, gy, gz;

// CAN DEFINITIONS
#define CAN0_INT 4 
#define CAN0_CS  2
#define SPI_SCK  18 
#define SPI_MISO  19 
#define SPI_MOSI  23 
MCP_CAN CAN0(CAN0_CS);

// We'll use an 8-byte buffer and send two CAN frames to transmit all six int16_t values
byte txData[8];
bool canInitialized = false;
unsigned long sentCount = 0;

void initializeCAN() {
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
    canInitialized = true;
    CAN0.setMode(MCP_NORMAL);
  } else {
    Serial.println("Error Initializing MCP2515...");
    Serial.println("Check wiring and try again.");
    canInitialized = false;
  }
}
void sendCANMessage(unsigned long messageId = 0x20) {
  if (canInitialized) {  
    byte sndStat = CAN0.sendMsgBuf(messageId, 8, txData);
    if (sndStat == CAN_OK) {
      Serial.print("ID: 0x");
      Serial.print(messageId, HEX);
      Serial.print(" | Data: ");     
      for (int i = 0; i < 8; i++) {
        if (txData[i] < 0x10) Serial.print("0");
        Serial.print(txData[i], HEX);
        Serial.print(" ");
      }
      Serial.println();     
    } else {
      Serial.println("ERROR: Failed to send message");
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CAN0_CS);
  initializeCAN();
  pinMode(CAN0_INT, INPUT); 
  Wire.begin(SDA_PIN,SCL_PIN);
  mpu.initialize();
   if(mpu.testConnection() ==  false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }
}

void loop() {
  if (millis() - prevTX >= invlTX) {
    prevTX = millis();
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    Serial.print("a[x y z] g[x y z]:\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);

    // Pack first CAN frame: ax, ay, az (3 * int16_t = 6 bytes) + 2 bytes padding
    {
      uint16_t u;
      u = (uint16_t)ax;
      txData[0] = (u >> 8) & 0xFF;
      txData[1] = u & 0xFF;
      u = (uint16_t)ay;
      txData[2] = (u >> 8) & 0xFF;
      txData[3] = u & 0xFF;
      u = (uint16_t)az;
      txData[4] = (u >> 8) & 0xFF;
      txData[5] = u & 0xFF;
      // pad remaining bytes
      txData[6] = 0;
      txData[7] = 0;
    }
    // Send first frame with ID 0x20
    sendCANMessage(0x20);

    // Pack second CAN frame: gx, gy, gz (3 * int16_t = 6 bytes) + 2 bytes padding
    {
      uint16_t u;
      u = (uint16_t)gx;
      txData[0] = (u >> 8) & 0xFF;
      txData[1] = u & 0xFF;
      u = (uint16_t)gy;
      txData[2] = (u >> 8) & 0xFF;
      txData[3] = u & 0xFF;
      u = (uint16_t)gz;
      txData[4] = (u >> 8) & 0xFF;
      txData[5] = u & 0xFF;
      // pad remaining bytes
      txData[6] = 0;
      txData[7] = 0;
    }
    // Send second frame with ID 0x21
    sendCANMessage(0x21);
  }
}
