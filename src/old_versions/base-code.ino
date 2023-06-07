#include <math.h>
#include <string.h>
#include <PrintStream.h>

#include <Servo.h> 
#include <Wire.h>
#include <Pixy2I2C.h>
#include <Adafruit_VL53L0X.h>
#include "BMI088.h"
#include <Encoder.h>
#include <U8g2lib.h>


///====================///
///      Defines       ///
///====================///


// Motor driver
#define PWM1 7
#define AIN1 9
#define AIN2 8

// Motor Encoder
#define ENCODER_PIN1 4
#define ENCODER_PIN2 5

// Servo 
#define SERVO_PIN 6
#define SERVO_ANGLE_CORECTION 5

// Camera
#define CAMERA_FPS 60

// Distance sensors
#define XSHUT_LEFT 33
#define INTERRUPT_LEFT 34
#define XSHUT_FRONT 35
#define INTERRUPT_FRONT 36
#define XSHUT_RIGHT 37
#define INTERRUPT_RIGHT 38

// Gyro sensor
#define GYRO_SAMPLE_SIZE 1
#define DRIFT_TEST_TIME 1

// Led
#define LED_PIN LED_BUILTIN

/// Use parts defines
// #define USE_MOTOR_DRIVER
// #define USE_MOTOR_ENCODER
// #define USE_SERVO
#define USE_CAMERA
#define USE_DISTANCE_SENSORS
#define USE_GYRO
#define USE_LED
#define USE_DISPLAY


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


// Motor encoder
Encoder myEnc(ENCODER_PIN1, ENCODER_PIN2);

// Servo
Servo servo;

// Camera
Pixy2I2C pixy;

// Distance sensors
Adafruit_VL53L0X sensor_left;
Adafruit_VL53L0X sensor_front;
Adafruit_VL53L0X sensor_right;

// Gyro sensor
Bmi088 bmi(Wire, 0x19, 0x69);

// Display
U8G2_SSD1306_128X64_ALT0_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);


///====================///
///      Variables     ///
///====================///

// Motor driver
int motor_speed = 100;

// Motor encoder 
long encoder_old_position  = -999;

// Servo
double delay_angle = 165 / 60;
double current_angle = 90;

// Camera
const double camera_fps = (1.0 / CAMERA_FPS) * 1000.0;
double camera_last_fps = 0;

// Distance sensors
sensorList_t distance_sensors[] = {
  {&sensor_left, &Wire, 0x30, XSHUT_LEFT, INTERRUPT_LEFT, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
  {&sensor_front, &Wire, 0x31, XSHUT_FRONT, INTERRUPT_FRONT, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
  {&sensor_right, &Wire, 0x32, XSHUT_RIGHT, INTERRUPT_RIGHT, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0}
};

const int DISTANCE_SENSORS_COUNT = sizeof(distance_sensors) / sizeof(distance_sensors[0]);

const uint16_t ALL_DISTANCE_SENSORS_PENDING = ((1 << DISTANCE_SENSORS_COUNT) - 1);
uint16_t distance_sensors_pending = ALL_DISTANCE_SENSORS_PENDING;
uint32_t distance_sensor_last_cycle_time;

uint16_t distance_sensor_ranges_mm[DISTANCE_SENSORS_COUNT];
bool distance_sensor_timeouts[DISTANCE_SENSORS_COUNT];
uint32_t distace_sensor_stop_times[DISTANCE_SENSORS_COUNT];

// Gyro sensor
double gx, gy, gz;
double gyro_last_read_time = 0;
double drifts_x, drifts_y, drifts_z;

// Led
int led_state = LOW;  // led_state used to set the LED
const long led_interval = 1000;  // led_interval at which to blink (milliseconds)
unsigned long led_previous_time = 0;  // will store last time LED was updated

// Main loop
long loop_iteration = 0;
long loop_temp_iteration = 0;
long loop_last_time = 0;
long loop_freq = 0;


///====================///
///     Functions      ///
///====================///


float clamp(float val, float small, float big) {
  return max(small, min(val, big));
}


/// Initialisation functions

void Initialize_sensors() {
  bool found_any_sensors = false;
  // Set all shutdown pins low to shutdown sensors
  for(int i = 0; i < DISTANCE_SENSORS_COUNT; i++)
    digitalWrite(distance_sensors[i].shutdown_pin, LOW);
  delay(10);

  for(int i = 0; i < DISTANCE_SENSORS_COUNT; i++) {
    // one by one enable sensors and set their ID
    digitalWrite(distance_sensors[i].shutdown_pin, HIGH);
    delay(10); // give time to wake up.
    if(distance_sensors[i].psensor->begin(distance_sensors[i].id, false, distance_sensors[i].pwire, distance_sensors[i].sensor_config)) {
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


/// Motor functions

void motorStart(int speed) {
  int out = abs(speed) * 2.55; // Convert speed to PWM value (0 to 255)
  if(speed >= 0) { // Forward direction
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else { // Reverse direction
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  analogWrite(PWM1, out);
}

void motorStop() {
  motorStart(-100);
  delay(50);
  motorStart(0);    
}


/// Servo functions

void turn(double angle) { 
  if(angle < 90)
    servo.write(0);
  else if(angle > 90)
    servo.write(180);
  else 
    servo.write(90);

  delay(delay_angle * 1.0 * (abs(current_angle - angle)));
  servo.write(90);

  current_angle = angle;
}

void move_servo(double angle) {
  angle = clamp(angle, -1, 1);

  double angle_deg = (angle + 1.0) * 90.0; // Convert angle to degrees (0 to 180)

  angle_deg = clamp(angle_deg, 20, 160); // Clamp angle_deg to (20 to 160)
  angle_deg += SERVO_ANGLE_CORECTION;

  turn(angle_deg);
}


///====================///
///       Setup        ///
///====================///


void setup() {
  Serial.begin(115200);

  /// Led
  #ifdef USE_LED
  pinMode(LED_PIN, OUTPUT);
  led_state = HIGH;
  digitalWrite(LED_PIN, led_state);
  #endif // USE_LED

  /// Display
  #ifdef USE_DISPLAY
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(30, 20, "Starting...");
  u8g2.sendBuffer();
  #endif // USE_DISPLAY

  /// Camera
  #ifdef USE_CAMERA
  Serial.println(F("Camera starting..."));
  pixy.init();
  Serial.println(F("Camera ok!"));
  #endif // USE_CAMERA

  /// Motor driver
  #ifdef USE_MOTOR_DRIVER
  pinMode(PWM1,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  #endif // USE_MOTOR_DRIVER

  /// Servo
  #ifdef USE_SERVO
  servo.attach(SERVO_PIN);
  #endif // USE_SERVO

  /// Wire
  Wire.begin();
  
  /// Distance sensors
  #ifdef USE_DISTANCE_SENSORS
  Serial.println(F("Distance sensors starting, initialize IO pins..."));
  for (int i = 0; i < DISTANCE_SENSORS_COUNT; i++) {
    pinMode(distance_sensors[i].shutdown_pin, OUTPUT);
    digitalWrite(distance_sensors[i].shutdown_pin, LOW);

    if (distance_sensors[i].interrupt_pin >= 0)
      pinMode(distance_sensors[i].interrupt_pin, INPUT_PULLUP);
  }
  Serial.println(F("Starting distance sensors..."));
  Initialize_sensors();
  Serial.println(F("Distance sensors ok!"));

  for (uint8_t i = 0; i < DISTANCE_SENSORS_COUNT; i++) {
    distance_sensors[i].psensor->startRangeContinuous(); // do 100ms cycle
  }
  #endif // USE_DISTANCE_SENSORS


  /// Gyro sensor
  #ifdef USE_GYRO
  int status = bmi.begin();
  if(status < 0) {
    Serial.println("BMI Initialization Error!");
    Serial.println(status);
    while(true);
  } 

  // Gyro drift calculation
  Serial.println("Starting gyro drift calculation...");

  gx = 0;
  gy = 0;
  gz = 0;

  gyro_last_read_time = millis();

  double start_time = millis();
  while(millis() - start_time < DRIFT_TEST_TIME * 1000) {
    bmi.readSensor();
    double read_time = millis();

    gx += (bmi.getGyroX_rads() * (read_time - gyro_last_read_time) * 0.001);
    gy += (bmi.getGyroY_rads() * (read_time - gyro_last_read_time) * 0.001);
    gz += (bmi.getGyroZ_rads() * (read_time - gyro_last_read_time) * 0.001);

    gyro_last_read_time = read_time;
  }

  drifts_x = gx / DRIFT_TEST_TIME;
  drifts_y = gy / DRIFT_TEST_TIME;
  drifts_z = gz / DRIFT_TEST_TIME;

  Serial.print("Drift test done!\nx: ");
  Serial.print(drifts_x, 6);
  Serial.print("   y: ");
  Serial.print(drifts_y, 6);
  Serial.print("   z: ");
  Serial.println(drifts_z, 6);

  // Gyro value reset
  gx = 0;
  gy = 0;
  gz = 0;

  gyro_last_read_time = millis();
  #endif // USE_GYRO

  /// Program start
  #ifdef USE_DISPLAY
  u8g2.clearBuffer();
  u8g2.drawStr(30, 20, "Ready!");
  u8g2.sendBuffer();
  #endif 
}


///====================///
///        Loop        ///
///====================///


void loop() {
  /// Print loop iteration
  Serial << "Iteration: #" << loop_iteration++ << " - " << millis() / 1000 
         << "." << millis() % 1000 <<  "s - " << loop_freq << "hz\n";
  loop_temp_iteration++; 


  /// Motor encoder
  #ifdef USE_MOTOR_ENCODER
  long newPosition = myEnc.read();
  if(newPosition != encoder_old_position) {
    encoder_old_position = newPosition;
    Serial.print("Encoder value: ");
    Serial.print(": ");
    Serial.println(newPosition / 4);
  }
  #endif // USE_MOTOR_ENCODER


  /// Camera
  #ifdef USE_CAMERA
  if(millis() - camera_last_fps >= camera_fps) {
    // grab blocks!
    pixy.ccc.getBlocks();

    // First cube 
    if(pixy.ccc.numBlocks >= 1) {
      pixy.ccc.blocks[0].print();
    }

   // Second cube 
   if(pixy.ccc.numBlocks >= 2) {
      pixy.ccc.blocks[1].print();
   }

    camera_last_fps = millis();
  }
  #endif // USE_CAMERA


  /// Distance sensors
  #ifdef USE_DISTANCE_SENSORS
  for(int i = 0; i < DISTANCE_SENSORS_COUNT; i++) {
    int range_complete = distance_sensors[i].psensor->isRangeComplete();
    if (range_complete) {
      distance_sensor_ranges_mm[i] = distance_sensors[i].psensor->readRange();
    }
  }

  Serial << "Distance:   left: " << distance_sensor_ranges_mm[0] << "mm   front: " 
         << distance_sensor_ranges_mm[1] << "mm   right: " << distance_sensor_ranges_mm[2] << "mm\n";
  #endif // USE_DISTANCE_SENSORS


  /// Gyro
  #ifdef USE_GYRO
  bmi.readSensor();
  double read_time = millis();

  gx += ((bmi.getGyroX_rads() - drifts_x) * (read_time - gyro_last_read_time) * 0.001) * 180.0 / PI;
  gy += ((bmi.getGyroY_rads() - drifts_y) * (read_time - gyro_last_read_time) * 0.001) * 180.0 / PI;
  gz += ((bmi.getGyroZ_rads() - drifts_z) * (read_time - gyro_last_read_time) * 0.001) * 180.0 / PI;

  gyro_last_read_time = read_time;

  Serial << "Gyro: gx: " << gx << "    gy: " << gy << "    gz: " << gz << "\n";
  #endif // USE_GYRO

  
  /// Led blink
  #ifdef USE_LED
  if(millis() - led_previous_time >= led_interval) {
    led_previous_time = millis();

    if (led_state == LOW)
      led_state = HIGH;
    else
      led_state = LOW;

    digitalWrite(LED_PIN, led_state);
  }
  #endif // USE_LED

  /// Display
  #ifdef USE_DISPLAY
  if(millis() - loop_last_time > 1000) {
    loop_freq = loop_temp_iteration * 1000 / (millis() - loop_last_time);
    char ans[100];
    u8g2.clearBuffer(); 
    itoa(loop_freq, ans, 10);
    strcat(ans, " Hz");
    u8g2.drawStr(30, 20, ans);
    u8g2.sendBuffer();   
    loop_temp_iteration = 0;
    loop_last_time = millis();
  }
  #endif // USE_DISPLAY
}


///====================///
///      Comments      ///
///====================///


/// Pixy camera

// pixy.ccc.blocks[i].m_signature The signature number of the detected object (1-7 for normal signatures)
// pixy.ccc.blocks[i].m_x The x location of the center of the detected object (0 to 316)
// pixy.ccc.blocks[i].m_y The y location of the center of the detected object (0 to 208)
// pixy.ccc.blocks[i].m_width The width of the detected object (1 to 316)
// pixy.ccc.blocks[i].m_height The height of the detected object (1 to 208)
// pixy.ccc.blocks[i].m_angle The angle of the object detected object if the detected object is a color code (-180 to 180).
// pixy.ccc.blocks[i].m_index The tracking index of the block
// pixy.ccc.blocks[i].m_age The number of frames the block has been tracked.
// pixy.ccc.blocks[i].print() A member function that prints the detected object information to the serial port