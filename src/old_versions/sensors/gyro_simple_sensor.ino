#include "BMI088.h"
#include <PrintStream.h>

BMI088 bmi088( BMI088_ACC_ADDRESS, BMI088_GYRO_ADDRESS );

const int interval = 100;

float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float lx = 0, ly = 0, lz = 0;
float x = 0, y = 0, z = 0;
int16_t temp = 0;

void setup(void) {
    Wire.begin();
    Serial.begin(115200);
    bmi088.initialize();
}

void loop(void) {
//    bmi088.getAcceleration(&ax, &ay, &az);
//    bmi088.getGyroscope(&gx, &gy, &gz);
//    temp = bmi088.getTemperature();
    
    //Serial << "Acc: " << ax << ", " << ay << ", " << az << endl;
    lx = x;
    ly = y;
    lz = z;
    
    for(int i = 0; i < interval; i++) {
      bmi088.getGyroscope(&gx, &gy, &gz);
      x += gx;
      y += gy;
      z += gz;
    }
    
    Serial << "Gyro: " << x / interval  << ", " << y / interval << ", " << z / interval << endl;
  }
