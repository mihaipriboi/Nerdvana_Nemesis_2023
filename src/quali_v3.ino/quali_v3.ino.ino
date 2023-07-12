#include <math.h>
#include <string.h>
#include <PrintStream.h>

#include <Servo.h> 
#include <Wire.h>
#include <Pixy2I2C.h>
#include "BMI088.h"
#include <Encoder.h>
#include <U8g2lib.h>
#include "Ultrasonic.h"

#include <SD.h>
#include <SPI.h>


///====================///
///      Defines       ///
///====================///


// Motor driver
#define PWM1 7
#define AIN1 9
#define AIN2 8

// Motor Encoder
#define ENCODER_PIN1 5
#define ENCODER_PIN2 4

// Servo 
#define SERVO_PIN 6
#define SERVO_ANGLE_CORECTION 5

// Camera
#define CAMERA_FPS 60

// Distance sensors
#define FRONT_SENSOR_PIN 2
#define LEFT_SENSOR_PIN 1
#define RIGHT_SENSOR_PIN 0

// Gyro sensor
#define GYRO_SAMPLE_SIZE 1
#define DRIFT_TEST_TIME 10

// Led
#define LED_PIN LED_BUILTIN

// SD Card
#define chipSelect BUILTIN_SDCARD

//BUTTON
#define BUTTON_PIN 32

/// Use parts defines
#define USE_MOTOR_DRIVER
#define USE_MOTOR_ENCODER
#define USE_SERVO
// #define USE_CAMERA
#define USE_DISTANCE_SENSORS
// #define USE_GYRO
#define USE_LED
#define USE_DISPLAY
#define USE_SD
//#define USE_BUTTON
// #define USE_MOTOR_SAFETY

/// State machine cases
#define MAIN_CASE 0
#define FOLLOW_CUBE 1


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
Ultrasonic ultrasonic_front(FRONT_SENSOR_PIN);
Ultrasonic ultrasonic_left(LEFT_SENSOR_PIN);
Ultrasonic ultrasonic_right(RIGHT_SENSOR_PIN);

// Gyro sensor
Bmi088 bmi(Wire, 0x19, 0x69);

// Display
U8G2_SSD1306_128X64_ALT0_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);


///====================///
///      Variables     ///
///====================///

// Motor driver
int motor_speed = 100;
int medium_motor_speed = 65;
int current_motor_speed = 0;

long safety_timeout = 1000;
long last_safety_time = 0;
int last_motor_enc = -999;

// Camera
const double camera_fps = (1.0 / CAMERA_FPS) * 1000.0;
double camera_last_fps = 0;
int min_area_cube = 160;

const int sig_to_col[] = { 0, 1, -1 };  // 0 - none, -1 - green, 1 - red


// Distance sensors
int left_sensor_cm = 0, right_sensor_cm = 0, front_sensor_cm = 0;
double last_time_front_sensor = 0, last_time_left_sensor = 0, last_time_right_sensor = 0;

// Gyro sensor
double gx, gy, gz;
double gyro_last_read_time = 0;
double drifts_x, drifts_y, drifts_z;

// Led
int led_state = LOW;  // led_state used to set the LED
const long led_interval = 1000;  // led_interval at which to blink (milliseconds)
unsigned long led_previous_time = 0;  // will store last time LED was updated

// Display
const long display_print_interval = 200;
long display_last_print_time = 0;

// SD card
const long sd_interval = 20;
long sd_last_write_time = 0;
char sd_filename[100];
String data_string = "";

// Setup
bool init_error = false;
bool init_sensors_error = false;
bool init_gyro_error = false;
bool init_display_error = false;
bool init_sd_error = false;

// Main loop
long loop_start_time = 0;
long loop_iteration = 0;
long loop_temp_iteration = 0;
long loop_last_time = 0;
long loop_freq = 0;

bool debug = false;

long last_side_read = 0;
long side_read_timeout = 250;
int distance_between_walls = 130;

bool limit_clamp = 0;

struct Cube {
  int color;  // -1 - green, 1 - red
  int x, y;
  int w, h;
  int index;
} cube_array[10], corner_cube, cube;


///====================///
///     Functions      ///
///====================///




///====================///
///       Setup        ///
///====================///


void setup() {
  sensors_setup();
}


///====================///
///        Loop        ///
///====================///

long delta_start = 0;
long delta_dist = 0;
long delta_gyro = 0;
long delta_displ = 0;

void loop() {
  /// Main code
  loop_update();
  loop_cases();
  writeSD();
}


///====================///
///      Comments      ///
///====================///

