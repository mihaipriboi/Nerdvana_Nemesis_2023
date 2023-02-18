#include "Seeed_vl53l0x.h"
#include <Arduino.h>
#include <U8g2lib.h>
#include <string.h>
 
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
#include "BMI088.h"

float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
int16_t temp = 0;
BMI088 bmi088( BMI088_ACC_ADDRESS, BMI088_GYRO_ADDRESS );

 
U8G2_SSD1306_128X64_ALT0_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
Seeed_vl53l0x VL53L0X;


#define SERIAL Serial

String ans;


void setup() {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    Wire.begin();
    u8g2.begin();
    SERIAL.begin(115200);

    u8g2.clearBuffer();                   // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr);   // choose a suitable font

    Status = VL53L0X.VL53L0X_common_init();
    if (VL53L0X_ERROR_NONE != Status) {
        SERIAL.println("start vl53l0x mesurement failed!");
        VL53L0X.print_pal_error(Status);
        while (1);
    }

    VL53L0X.VL53L0X_long_distance_ranging_init();

    if (VL53L0X_ERROR_NONE != Status) {
        SERIAL.println("start vl53l0x mesurement failed!");
        VL53L0X.print_pal_error(Status);
        while (1);
    }

    while (1) {
        if (bmi088.isConnection()) {
            bmi088.initialize();
            Serial.println("BMI088 is connected");
            break;
        } else {
            Serial.println("BMI088 is not connected");
        }

        delay(2000);
    }
}


void loop() {
    VL53L0X_RangingMeasurementData_t RangingMeasurementData;
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    memset(&RangingMeasurementData, 0, sizeof(VL53L0X_RangingMeasurementData_t));
    u8g2.clearBuffer(); 
    
    Status = VL53L0X.PerformSingleRangingMeasurement(&RangingMeasurementData);
    if (VL53L0X_ERROR_NONE == Status) {
        if (RangingMeasurementData.RangeMilliMeter >= 4000) {
            u8g2.drawStr(30, 20, "out of range");

        } else {
             char ans[100];
             itoa(RangingMeasurementData.RangeMilliMeter, ans, 10);
             u8g2.drawStr(30, 20, ans);
        }
    } 

    u8g2.sendBuffer();   

     bmi088.getAcceleration(&ax, &ay, &az);
    bmi088.getGyroscope(&gx, &gy, &gz);
    temp = bmi088.getTemperature();

    Serial.print(ax);
    Serial.print(",");
    Serial.print(ay);
    Serial.print(",");
    Serial.print(az);
    Serial.print(",");

    Serial.print(gx);
    Serial.print(",");
    Serial.print(gy);
    Serial.print(",");
    Serial.print(gz);
    Serial.print(",");

    Serial.print(temp);

    Serial.println();
    
    delay(300);
}







