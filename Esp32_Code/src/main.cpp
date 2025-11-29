#include<Arduino.h>
#include "Libs/CanSender.h"
#include "Constants.h"
#include "Libs/BnoWrap.h"

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// Servicios y Caracteristicas BLE
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define DEBUGGING true

//Variables para BLE
static BLEAddress *pServerAddress;
static boolean doConnect = false;
static boolean connected = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;

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

//INTERRUPT FUNCTION FOR ENCODER
void ARDUINO_ISR_ATTR encoderISR() {
  counter++;
}

// Funcion de callback para notificaciones BLE
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
  // Si el paquete es de 8 bytes (X, Y, Theta, etc)
  if (length == 8) {
      // Copiamos al buffer local
      memcpy(CameraData, pData, 8);
      
      // ¡REENVÍO INMEDIATO AL STM32!
      // No esperamos al loop de 50Hz para evitar lag
      canSystem.send((unsigned long)CameraID, 8, CameraData);
      
      // Debug opcional (quitar si satura el serial)
      if (DEBUGGING){
        Serial.println("Cam Data Forwarded via CAN");
        int16_t x_raw = (int16_t)(pData[0] | (pData[1] << 8));
        int16_t y_raw = (int16_t)(pData[2] | (pData[3] << 8));
        int16_t th_raw = (int16_t)(pData[4] | (pData[5] << 8));
        
        Serial.print("CAM DATA -> X: ");
        Serial.print(x_raw);
        Serial.print(" | Y: ");
        Serial.print(y_raw);
        Serial.print(" | Theta: ");
        Serial.println(th_raw);
      }
  }
}

// Clase para manejar eventos de conexión/desconexión BLE
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pClient) {
    connected = true;
    Serial.println(" - Connected to server");
  }
  void onDisconnect(BLEClient* pClient) {
    connected = false;
    Serial.println(" - Disconnected from server");
  }
};

// Conexión al servidor BLE
bool connectToServer(BLEAddress pAddress) {
    Serial.print("Forming a connection to ");
    Serial.println(pAddress.toString().c_str());

    BLEClient* pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallback());

    if (!pClient->connect(pAddress)) return false;

    BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
    if (pRemoteService == nullptr) {
      pClient->disconnect();
      return false;
    }

    pRemoteCharacteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID);
    if (pRemoteCharacteristic == nullptr) {
      pClient->disconnect();
      return false;
    }

    if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyCallback);

    return true;
}

// Clase para manejar resultados de escaneo BLE
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // Si encontramos el dispositivo con el UUID correcto
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(BLEUUID(SERVICE_UUID))) {
      advertisedDevice.getScan()->stop(); // Detener escaneo
      pServerAddress = new BLEAddress(advertisedDevice.getAddress());
      doConnect = true; // Bandera para conectar en el loop
      Serial.println("Target device found!");
    }
  }
};

void setup() {
  Serial.begin(115200); //Serial initialization
  pinMode(EncoderPin, INPUT); //Encoder initialization
  attachInterrupt(EncoderPin, encoderISR, RISING); // Interrupt initialization
  if (canSystem.begin()) Serial.println("Can Activated!"); //Activate can
  if (bnoSensor.begin()) Serial.println("Bno Activated");
  // Inicializar BLE
  BLEDevice::init("ESP32-Tractor-Client");
  // Configurar escaneo BLE
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  // Iniciar escaneo
  Serial.println("Scanning for Camera Server...");
  pBLEScan->start(5, false);

  prevTX = millis(); // Initialize clock
}
void loop() {
      
  
  if (doConnect == true) {
    if (connectToServer(*pServerAddress)) {
            Serial.println("We are now connected to the Camera!");
        } else {
            Serial.println("Failed to connect.");
        }
        doConnect = false;
    }

    if(!connected && !doConnect) {
      BLEDevice::getScan()->start(0.1, false);
      Serial.println("Scanning for Camera Server...");
    }
  
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
       }
}
