#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

#define SDA_PIN 21
#define SCL_PIN 22

int16_t ax, ay, az;
int16_t gx, gy, gz;

//Variables usadas por el filtro pasa bajos
long f_ax,f_ay, f_az;
int p_ax, p_ay, p_az;
long f_gx,f_gy, f_gz;
int p_gx, p_gy, p_gz;
int counter=0;

//Valor de los offsets
int ax_o,ay_o,az_o;
int gx_o,gy_o,gz_o;

void setup() {
  Wire.begin(SDA_PIN,SCL_PIN);
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
  //mpu.setXAccelOffset(0); //Set your accelerometer offset for axis X
  //mpu.setXGyroOffset(0);  //Set your gyro offset for axis X
  /*Print the defined offsets*/
  ax_o=mpu.getXAccelOffset();
  ay_o=mpu.getYAccelOffset();
  az_o=mpu.getZAccelOffset();
  gx_o=mpu.getXGyroOffset();
  gy_o=mpu.getYGyroOffset();
  gz_o=mpu.getZGyroOffset();
  
  Serial.println("Offsets:");
  Serial.print(ax_o); Serial.print("\t"); 
  Serial.print(ay_o); Serial.print("\t"); 
  Serial.print(az_o); Serial.print("\t"); 
  Serial.print(gx_o); Serial.print("\t"); 
  Serial.print(gy_o); Serial.print("\t");
  Serial.print(gz_o); Serial.print("\t");
  Serial.println("nnEnvie cualquier caracter para empezar la calibracionnn");  
  // Espera un caracter para empezar a calibrar
  while (true){if (Serial.available()) break;}  
  Serial.println("Calibrando, no mover IMU");  
}

void loop() {
  /* Read raw accel/gyro data from the module. Other methods commented*/
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // Filtrar las lecturas
  f_ax = f_ax-(f_ax>>5)+ax;
  p_ax = f_ax>>5;

  f_ay = f_ay-(f_ay>>5)+ay;
  p_ay = f_ay>>5;

  f_az = f_az-(f_az>>5)+az;
  p_az = f_az>>5;

  f_gx = f_gx-(f_gx>>3)+gx;
  p_gx = f_gx>>3;

  f_gy = f_gy-(f_gy>>3)+gy;
  p_gy = f_gy>>3;

  f_gz = f_gz-(f_gz>>3)+gz;
  p_gz = f_gz>>3;

  //Cada 100 lecturas corregir el offset
  if (counter==100){
    //Mostrar las lecturas separadas por un [tab]
    Serial.print("promedio:"); Serial.print("t");
    Serial.print(p_ax); Serial.print("\t");
    Serial.print(p_ay); Serial.print("\t");
    Serial.print(p_az); Serial.print("\t");
    Serial.print(p_gx); Serial.print("\t");
    Serial.print(p_gy); Serial.print("\t");
    Serial.println(p_gz);

    //Calibrar el acelerometro a 1g en el eje z (ajustar el offset)
    if (p_ax>0) ax_o--;
    else {ax_o++;}
    if (p_ay>0) ay_o--;
    else {ay_o++;}
    if (p_az-16384>0) az_o--;
    else {az_o++;}
    
    mpu.setXAccelOffset(ax_o);
    mpu.setYAccelOffset(ay_o);
    mpu.setZAccelOffset(az_o);

    //Calibrar el giroscopio a 0º/s en todos los ejes (ajustar el offset)
    if (p_gx>0) gx_o--;
    else {gx_o++;}
    if (p_gy>0) gy_o--;
    else {gy_o++;}
    if (p_gz>0) gz_o--;
    else {gz_o++;}
    
    mpu.setXGyroOffset(gx_o);
    mpu.setYGyroOffset(gy_o);
    mpu.setZGyroOffset(gz_o);    

    counter=0;
  }
  counter++;

}