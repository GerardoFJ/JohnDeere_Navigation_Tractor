#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;
long tiempo_prev;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;

void setup() {
  Wire.begin(); 
  Serial.begin(115200); //Initializate Serial wo work well at 8MHz/16MHz
  delay(2000);
  /*Initialize device and check connection*/ 
  Serial.println("Initializing MPU...");
  mpu.initialize();
  Serial.println("Testing MPU6050 connection...");
  if(mpu.testConnection() ==  false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }
  else{
    Serial.println("MPU6050 connection successful");
  }
  
}


void loop() {
  // Leer las aceleraciones y velocidades angulares
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  dt = (millis()-tiempo_prev)/1000.0;
  tiempo_prev=millis();
  
  //Calcular los ángulos con acelerometro
  float accel_ang_x=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);
  float accel_ang_y=atan(-ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
  
  //Calcular angulo de rotación con giroscopio y filtro complemento  
  ang_x = 0.98*(ang_x_prev+(gx/131)*dt) + 0.02*accel_ang_x;
  ang_y = 0.98*(ang_y_prev+(gy/131)*dt) + 0.02*accel_ang_y;
  
  
  ang_x_prev=ang_x;
  ang_y_prev=ang_y;

  //Mostrar los angulos separadas por un [tab]
  Serial.print("Rotacion en X Y:\t");
  Serial.print(ang_x); Serial.print("\t");
  Serial.println(ang_y);

  delay(10);
}