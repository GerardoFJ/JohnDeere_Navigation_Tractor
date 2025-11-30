#include "ble.h"

// Initialize static members
BLEAddress *BleModule::pServerAddress = nullptr;
boolean BleModule::doConnect = false;
boolean BleModule::connected = false;
BLERemoteCharacteristic* BleModule::pRemoteCharacteristic = nullptr;
byte *BleModule::CameraData = nullptr;

bool BleModule::calibrated = false;
uint16_t BleModule::offsetX = 0;
uint16_t BleModule::offsetY = 0;
uint16_t BleModule::offsetAngle = 0;

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pClient) {
    BleModule::connected = true;
  }
  void onDisconnect(BLEClient* pClient) {
    BleModule::connected = false;
  }
};

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // Si encontramos el dispositivo con el UUID correcto
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(BLEUUID(SERVICE_UUID))) {
      advertisedDevice.getScan()->stop(); // Detener escaneo
      BleModule::pServerAddress = new BLEAddress(advertisedDevice.getAddress());
      BleModule::doConnect = true; 
      Serial.println("Target device found!");
    }
  }
};

BleModule::BleModule(byte *CameraData) {
    BleModule::CameraData = CameraData;
}

void BleModule::notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {

  if (length == 8 && CameraData != nullptr) {

      // Big-endian: high byte first
      uint16_t x_raw  = (uint16_t)((pData[2] << 8) | pData[3]);
      uint16_t y_raw  = (uint16_t)((pData[4] << 8) | pData[5]);
      uint16_t angle  = (uint16_t)((pData[6] << 8) | pData[7]);

      // First valid packet: take it as zero reference
      if (!calibrated) {
          offsetX     = x_raw;
          offsetY     = y_raw;
          offsetAngle = angle;
          calibrated  = true;
      }

      // Relative values based on first packet
      int16_t x_rel  = (int16_t)(x_raw  - offsetX);
      int16_t y_rel  = (int16_t)(y_raw  - offsetY);
      int16_t ang_rel= (int16_t)(angle  - offsetAngle);

      CameraData[0] = (uint8_t)(x_rel >> 8);
      CameraData[1] = (uint8_t)(x_rel & 0xFF);
      CameraData[2] = (uint8_t)(y_rel >> 8);
      CameraData[3] = (uint8_t)(y_rel & 0xFF);
      CameraData[4] = (uint8_t)(ang_rel >> 8);
      CameraData[5] = (uint8_t)(ang_rel & 0xFF);
      CameraData[6] = pData[0];
      CameraData[7] = 0;

      if (DEBUGGING) {
        Serial.println("Cam Data Forwarded via CAN");

        Serial.print("RAW  -> X: ");
        Serial.print(x_raw);
        Serial.print(" | Y: ");
        Serial.print(y_raw);
        Serial.print(" | Angle: ");
        Serial.println(angle);

        Serial.print("REL  -> X: ");
        Serial.print(x_rel);
        Serial.print(" | Y: ");
        Serial.print(y_rel);
        Serial.print(" | Angle: ");
        Serial.println(ang_rel);

        // Optional: dump raw bytes
        Serial.print("Raw packet bytes: ");
        for (size_t i = 0; i < length; ++i) {
          Serial.print("0x");
          if (pData[i] < 16) Serial.print('0');
          Serial.print(pData[i], HEX);
          Serial.print(' ');
        }
        Serial.println();
      }
  }
}

bool BleModule::connectToServer(BLEAddress pAddress) {
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

bool BleModule::status(){
    if (doConnect == true) {
        if (connectToServer(*pServerAddress)) {
            Serial.println("We are now connected to the Camera!"); 
        } else {
            Serial.println("Failed to connect.");
        }
        doConnect = false;
    }
    
    if(!connected && !doConnect) {
         BLEDevice::getScan()->start(1, false); 
    }
    return connected;
}

bool BleModule::begin(){
  BLEDevice::init("ESP32-Tractor-Client");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
  return true;
}