#include "Seeed_vl53l0x.h"
#include <Arduino.h>
#include <Wire.h>


Seeed_vl53l0x VL53L0X;


void setup() {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    Wire.begin();
    Serial.begin(115200);

    Status = VL53L0X.VL53L0X_common_init();
    if (VL53L0X_ERROR_NONE != Status) {
        Serial.println("start vl53l0x mesurement failed!");
        VL53L0X.print_pal_error(Status);
        while (1);
    }

    VL53L0X.VL53L0X_long_distance_ranging_init();

    if (VL53L0X_ERROR_NONE != Status) {
        Serial.println("start vl53l0x mesurement failed!");
        VL53L0X.print_pal_error(Status);
        while (1);
    }
            Serial.println("start vl53l0x mesurement failed!");

}


void loop()
{
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;


  for (byte i = 1; i < 120; i++)
  {

    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
    } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");

  delay(3000);
}
