#ifndef BNOWRAP_H
#define BNOWRAP_H

#include <Arduino.h>
#include "Constants.h"
#include "BNO055_support.h"
#include <Wire.h>

class BnoWrapper{
    private:
        struct bno055_t BNO;
        struct bno055_euler EulerData;
    public:
        BnoWrapper();
        bool begin();
        float getYaw();
};

#endif