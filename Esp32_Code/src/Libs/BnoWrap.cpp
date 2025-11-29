#include "BnoWrap.h"

BnoWrapper::BnoWrapper(){}

bool BnoWrapper::begin(){
  Wire.begin();
  BNO_Init(&BNO);
  bno055_set_reset_sys(1);
  delay(1);
  bno055_set_reset_sys(0);
  bno055_set_euler_unit(0x01);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  return true;

}

float BnoWrapper::getYaw(){
    bno055_read_euler_h(&EulerData.h);
    return float(EulerData.h) / 900.00;
}