#ifndef BLE_H
#define BLE_H

#include <Arduino.h>
#include "Constants.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

class MyClientCallback;
class MyAdvertisedDeviceCallbacks;

class BleModule{ //Bluetooth Class
    friend class MyClientCallback;
    friend class MyAdvertisedDeviceCallbacks;

    private:
        static BLEAddress *pServerAddress;
        static boolean doConnect;
        static boolean connected;
        static BLERemoteCharacteristic* pRemoteCharacteristic;
        static byte *CameraData;

         // Calibration state
        static bool calibrated;
        static uint16_t offsetX;
        static uint16_t offsetY;
        static uint16_t offsetAngle;
        //Internal functions
        static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,uint8_t* pData,size_t length,bool isNotify);
        bool connectToServer(BLEAddress pAddress);

    public:
        BleModule(byte *CameraData); // Object constructor
        bool begin(); // Initialization
        bool status(); //Status update


};



#endif