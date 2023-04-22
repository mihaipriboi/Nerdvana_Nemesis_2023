#include <Encoder.h>
#include <Pixy2I2C.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "BMI088.h"
#include <PrintStream.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <string.h>


///====================///
///      Defines       ///
///====================///

// Motor driver
#define PWM1 8
#define AIN1 10
#define AIN2 9

// Distance sensors
#define XSHUT_left 33
#define INTERRUPT_left 34
#define XSHUT_front 35
#define INTERRUPT_front 36
#define XSHUT_right 37
#define INTERRUPT_right 38

// Gyro sensors
#define gyro_sample_size 1

///====================///
///      Structs       ///
///====================///

typedef struct {
  Adafruit_VL53L0X *psensor; // pointer to object
  TwoWire *pwire;
  int id;            // IIC id number for the sensor
  int shutdown_pin;  // which pin for shutdown;
  int interrupt_pin; // which pin to use for interrupts.
  Adafruit_VL53L0X::VL53L0X_Sense_config_t sensor_config;     // options for how to use the sensor
  uint16_t range;        // range value used in continuous mode stuff.
  uint8_t sensor_status; // status from last ranging in continuous.
} sensorList_t;

///====================///
///   Initialization   ///
///====================///

// Camera
Pixy2I2C pixy;

// Motor encoder
Encoder myEnc(5, 6);

// Distance sensors
Adafruit_VL53L0X sensor_left;
Adafruit_VL53L0X sensor_front;
Adafruit_VL53L0X sensor_right;

// Gyro sensor

BMI088 bmi088( BMI088_ACC_ADDRESS, BMI088_GYRO_ADDRESS );

// Display

U8G2_SSD1306_128X64_ALT0_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);


///====================///
///      Variables     ///
///====================///

// Motor encoder 
long oldPosition  = -999;

// Motor driver
int out = 255;
int spd = 100;

// Distance sensors
sensorList_t sensors[] = {
  {&sensor_left, &Wire, 0x30, XSHUT_left, INTERRUPT_left, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
  {&sensor_front, &Wire, 0x31, XSHUT_front, INTERRUPT_front, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
  {&sensor_right, &Wire, 0x32, XSHUT_right, INTERRUPT_right, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0}
};

const int COUNT_SENSORS = sizeof(sensors) / sizeof(sensors[0]);

uint16_t ranges_mm[COUNT_SENSORS];

//gyro sensor

float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float lx = 0, ly = 0, lz = 0;
float x = 0, y = 0, z = 0;
int16_t temp = 0;

///====================///
///     Functions      ///
///====================///

void Initialize_sensors() {
  bool found_any_sensors = false;
  // Set all shutdown pins low to shutdown sensors
  for(int i = 0; i < COUNT_SENSORS; i++)
    digitalWrite(sensors[i].shutdown_pin, LOW);
  delay(10);

  for(int i = 0; i < COUNT_SENSORS; i++) {
    // one by one enable sensors and set their ID
    digitalWrite(sensors[i].shutdown_pin, HIGH);
    delay(10); // give time to wake up.
    if(sensors[i].psensor->begin(sensors[i].id, false, sensors[i].pwire, sensors[i].sensor_config)) {
      found_any_sensors = true;
    } 
    else {
      Serial.print(i, DEC);
      Serial.print(F(": c to start\n"));
    }
  }
  if(!found_any_sensors) {
    Serial.println("No valid sensors found");
    while (1);
  }
}

///====================///
///       Setup        ///
///====================///

void setup() {
  Serial.begin(115200);

  /// Camera
  Serial.println(F("Pixycam startnig"));
  pixy.init();
  Serial.println(F("Pixycam ok!"));

  /// Motor driver
  pinMode(PWM1,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);

  /// Distance sensors
  Wire.begin();

  /// Gyro sensor
  bmi088.initialize();

  // Display
  u8g2.begin();
  u8g2.clearBuffer();                   // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr);   // choose a suitable font
  
  Serial.println(F("Distance sensosr starting, initialize IO pins"));
  for (int i = 0; i < COUNT_SENSORS; i++) {
    pinMode(sensors[i].shutdown_pin, OUTPUT);
    digitalWrite(sensors[i].shutdown_pin, LOW);

    if (sensors[i].interrupt_pin >= 0)
      pinMode(sensors[i].interrupt_pin, INPUT_PULLUP);
  }
  Serial.println(F("Starting sensors..."));
  Initialize_sensors();
  Serial.println(F("Distance sensors ok!"));



  // wait for serial port for 2 seconds max
  while (!Serial && (millis() < 2000));
}

///====================///
///        Loop        ///
///====================///

int loop_iteration = 0;
int loop_temp_iteration = 0;
float last_time = 0;
float loop_freq = 0;

void loop() {
  Serial.print("Iteration: #");
  Serial.print(loop_iteration++);
  loop_temp_iteration++;
  Serial.print(" - ");
  Serial.print(millis()/1000);
  Serial.print(".");
  Serial.println(millis()%1000);
  
  /// Camera
  int i; 
  // grab blocks!
  pixy.ccc.getBlocks();
  
  // If there are detect blocks, print them!
  if (pixy.ccc.numBlocks)
  {
    Serial.print("Detected ");
    Serial.println(pixy.ccc.numBlocks);
    for (i=0; i<pixy.ccc.numBlocks; i++)
    {
      Serial.print("  block ");
      Serial.print(i);
      Serial.print(": ");
      pixy.ccc.blocks[i].print();
    }
  }

  /// Motor driver
  if(spd >= 0) {
    digitalWrite(AIN1,HIGH);
    digitalWrite(AIN2,LOW);
  }
  else {
    digitalWrite(AIN1,LOW);
    digitalWrite(AIN2,HIGH);
  }
  
  analogWrite(PWM1,out);

  /// Motor encoder
  long newPosition = myEnc.read();
  if(newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.print("Encoder value: ");
    Serial.print(": ");
    Serial.println(newPosition / 4);
  }

  /// Distance sensors
  for(int i = 0; i < COUNT_SENSORS; i++) {
    ranges_mm[i] = sensors[i].psensor->readRange();
  }
  
  for(int i = 0; i < COUNT_SENSORS; i++) {
    Serial.print("Sensor #");
    Serial.print(i, DEC);
    Serial.print(" at IIC address 0x");
    Serial.print(sensors[i].id, HEX);
    Serial.print(": ");
    Serial.print(ranges_mm[i], DEC);
    Serial.println(" mm");
  }
  Serial.println();

  lx = x; 
  ly = y; 
  lz = z;
  for(int i = 0; i < gyro_sample_size; i++) {
    bmi088.getGyroscope(&gx, &gy, &gz);
    x += gx;
    y += gy;
    z += gz;
  }
  
  Serial << "Gyro: " << (x) / gyro_sample_size  << ", " << (y) / gyro_sample_size << ", " << (z) / gyro_sample_size << endl;
  Serial.println();
  Serial.println();

  if(millis() - last_time > 1000) {
    loop_freq = loop_temp_iteration / (millis() - last_time) * 1000;
    loop_temp_iteration = 0;
    last_time = millis();
  }

  char ans[100];
  u8g2.clearBuffer(); 
  itoa(loop_freq, ans, 10);
  strcat(ans, " Hz");
  u8g2.drawStr(30, 20, ans);
  u8g2.sendBuffer();   

//  delay(1000);
}
