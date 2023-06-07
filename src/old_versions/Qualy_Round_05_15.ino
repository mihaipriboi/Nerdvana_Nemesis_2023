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
#define USE_DISTANCE_SENSORS
#define USE_GYRO
#define USE_LED
#define USE_DISPLAY
#define USE_SD
//#define USE_BUTTON
#define USE_MOTOR_SAFETY

/// State machine cases
#define MAIN_CASE 0
#define WAIT_CASE 1


///====================///
///   Initialization   ///
///====================///


// Motor encoder
Encoder myEnc(ENCODER_PIN1, ENCODER_PIN2);

// Servo
Servo servo;

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
int medium_motor_speed = 70;
int current_motor_speed = 0;

long safety_timeout = 1000;
long last_safety_time = 0;
int last_motor_enc = -999;

// Servo
double delay_angle = 165 / 60;
double servo_corrections = 0;

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


///====================///
///     Functions      ///
///====================///


float clamp(float val, float small, float big) {
  return max(small, min(val, big));
}


/// Display functions

void display_print(const char *s) {
  #ifdef USE_DISPLAY
  u8g2.clearBuffer();
  u8g2.drawStr(30, 20, s);
  u8g2.sendBuffer();
  #endif // USE_DISPLAY
}

void display_print(const char *s1, const char *s2) {
  #ifdef USE_DISPLAY
  u8g2.clearBuffer();
  u8g2.drawStr(30, 15, s1);
  u8g2.drawStr(30, 25, s2);
  u8g2.sendBuffer();
  #endif // USE_DISPLAY
}

void display_print(const double n, const char *sufix = "") {
  #ifdef USE_DISPLAY
  char s[100];
  itoa(n, s, 10);
  strcat(s, sufix);

  u8g2.clearBuffer();
  u8g2.drawStr(30, 20, s);
  u8g2.sendBuffer();
  #endif // USE_DISPLAY
}


void display_print(const double a, const double b) {
  #ifdef USE_DISPLAY
  char s1[100], s2[100];
  itoa(a, s1, 10);
  itoa(b, s2, 10);

  u8g2.clearBuffer();
  u8g2.drawStr(30, 15, s1);
  u8g2.drawStr(30, 25, s2);
  u8g2.sendBuffer();
  #endif // USE_DISPLAY
}


/// Motor functions

void motor_start(int speed) {
  current_motor_speed = speed;
  
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

void motor_stop() {
  motor_start(-1); 
  current_motor_speed = 0;
}

long read_motor_encoder() {
  return (double)myEnc.read() / 47.74;
}

/// Servo functions

void move_servo(double angle) {
  angle = -clamp(angle, -1, 1);

  double angle_deg = 90 + angle * 90.0 - servo_corrections; // Convert angle to degrees (0 to 180)
  angle_deg = clamp(angle_deg, 0, 180);
  if(limit_clamp)
    angle_deg = clamp(angle_deg * (150 - 30) / (180 - 0) + 30, 30, 150);
  servo.write(angle_deg);
}


/// SD card
void file_print(String s) {
  File data_file = SD.open(sd_filename, FILE_WRITE);
  if(data_file) {
    data_file.print(s);
    data_file.close();
  }
}

void file_println(String s) {
  File data_file = SD.open(sd_filename, FILE_WRITE);
  if(data_file) {
    data_file.println(s);
    data_file.close();
  }
}


/// Main loop

bool is_in_turn_side_sensors() {
  if(left_sensor_cm + right_sensor_cm > distance_between_walls) {
    last_side_read = millis();
  }

  return millis() - last_side_read <= side_read_timeout;
}


///====================///
///       Setup        ///
///====================///


void setup() {
  Serial.begin(115200);


  // if serial is not available
  while(!Serial && (millis() < 2000));

  if(Serial) debug = true;

  /// Led
  #ifdef USE_LED
  pinMode(LED_PIN, OUTPUT);
  led_state = HIGH;
  digitalWrite(LED_PIN, led_state);
  #endif // USE_LED

  /// Display
  #ifdef USE_DISPLAY
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  display_print("Starting...");
  #endif // USE_DISPLAY

  /// Motor driver
  #ifdef USE_MOTOR_DRIVER
  pinMode(PWM1,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  #endif // USE_MOTOR_DRIVER

  /// Servo
  #ifdef USE_SERVO
  servo.attach(SERVO_PIN, 1230, 1770);
  move_servo(0);
  #endif // USE_SERVO

  /// Wire
  Wire.begin();

  /// Gyro sensor
  #ifdef USE_GYRO
  int status = bmi.begin();
  bmi.setOdr(Bmi088::ODR_400HZ);
  bmi.setRange(Bmi088::ACCEL_RANGE_6G,Bmi088::GYRO_RANGE_500DPS);
  if(status < 0) {
    if(debug) Serial << "BMI Initialization Error!  error: " << status << "\n";
    init_error = init_gyro_error = true;
  }
  else  {
    // Gyro drift calculation
    if(debug) Serial.println("Starting gyro drift calculation...");

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

    if(debug) Serial.print("Drift test done!\nx: ");
    if(debug) Serial.print(drifts_x, 6);
    if(debug) Serial.print("   y: ");
    if(debug) Serial.print(drifts_y, 6);
    if(debug) Serial.print("   z: ");
    if(debug) Serial.println(drifts_z, 6);
  }
  // Gyro value reset
  gx = 0;
  gy = 0;
  gz = 0;

  gyro_last_read_time = millis();
  #endif // USE_GYRO


  #ifdef USE_SD
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present!");
    init_sd_error = true;
  }
  else {
    Serial.println("Card present!");

    // SD.remove("datalog.txt");

    int i = 0;
    do {
      char aux[100];
      itoa(i, aux, 10);

      memset(sd_filename, 0, sizeof(sd_filename));

      strcat(sd_filename, "datalog_quali_");
      strcat(sd_filename, aux);
      strcat(sd_filename, ".csv");

      i++;
    } while(SD.exists(sd_filename));

    if(debug) Serial.print("Using filename: ");
    if(debug) Serial.println(sd_filename);


    String aux;

    if(!init_error) aux = "ok!"; else aux = "error!";
    data_string = "init: ," + aux + "\n";

    if(!init_sensors_error) aux = "ok!"; else aux = "error!";
    data_string += "dist s: ," + aux + "\n";
    
    if(!init_gyro_error) aux = "ok!"; else aux = "error!";
    data_string += "gyro: ," + aux + "\n";

    data_string += "drift: ," + String(drifts_z, 6) + "\n";

    file_println(data_string);
  }
  #endif // USE_SD


  /// Program start
  if(!init_error) {
    digitalWrite(LED_PIN, LOW);
    display_print("Ready!");
  }
  else {
    if(init_sensors_error && init_gyro_error)
      display_print("Dist sensors err!", "Gyro err!");
    else if(init_sensors_error)
      display_print("Dist sensors err!");
    else if(init_gyro_error)
      display_print("Gyro err!");
  }

  #ifdef USE_BUTTON
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  delay(10);

  Serial.println(digitalRead(BUTTON_PIN));
  while(digitalRead(BUTTON_PIN) == 1){
    Serial.println(digitalRead(BUTTON_PIN));
    delay(50);
  }
  #endif


  loop_start_time = millis();

  motor_start(medium_motor_speed);
}


///====================///
///        Loop        ///
///====================///

double kp0 = 0.068;
double ki0 = 0;
double kd0 = 0.1;
double kp = 0.028;
double ki = 0;
double kd = 0.082;
double pid_error, pid_last_error = 0;
double integral = 0;
double turn_delay = 650;
double last_turn_time = 0;
int turns = 0;
double turn_angle = 89.6;
double turn_angle_left = 91;
double current_angle = -1.6;
int turn_direction = 0;
int distance_to_wall_for_turn = 78;
int last_drive_start_cm = 0;
int last_drive_cm = 130;

int loop_case = 0;

int turn_encoder_start_cm = 1000000000;
int flag_wide = 0;

int flag_wait = 0;
long wait_last_time = 0;
long wait_time = 1000;
int wait_speed_direction = 0;

void loop() {
  /// Print loop iteration
  if(millis() - loop_last_time > 1000) {
    loop_freq = loop_temp_iteration * 1000 / (millis() - loop_last_time);
    loop_temp_iteration = 0;
    loop_last_time = millis();
  }

  loop_iteration++;
  loop_temp_iteration++;

  if(debug) Serial << "Iteration: #" << loop_iteration << " - " << millis() / 1000 
         << "." << millis() % 1000 <<  "s - " << loop_freq << "hz\n"; 

  /// Motor encoder
  #ifdef USE_MOTOR_ENCODER
  if(debug) Serial << "Encoder: " << read_motor_encoder() << "\n";


  #ifdef USE_MOTOR_SAFETY
  if(millis() - last_safety_time > safety_timeout) {
    if(abs(current_motor_speed) > 0 && abs(read_motor_encoder() - last_motor_enc) < 1) {
      file_println("Safety stop: ," + String(read_motor_encoder()) + ", " + String(last_motor_enc));
      motor_start(0);
      display_print("Safety stop!");
      delay(999999999);
    }

    last_safety_time = millis();
    last_motor_enc = read_motor_encoder();
  }
  if(current_motor_speed == 0)
    last_safety_time = millis();

  #endif // USE_MOTOR_SAFETY

  #endif // USE_MOTOR_ENCODER



  /// Distance sensors
  #ifdef USE_DISTANCE_SENSORS
  if(millis() - last_time_front_sensor >= 50) {
    front_sensor_cm = ultrasonic_front.MeasureInCentimeters();
    last_time_front_sensor = millis();
  } 
  
  if(millis() - last_time_left_sensor >= 50) {
    left_sensor_cm = ultrasonic_left.MeasureInCentimeters();
    last_time_left_sensor = millis();
  } 
  
  if(millis() - last_time_right_sensor >= 50) {
    right_sensor_cm = ultrasonic_right.MeasureInCentimeters();
    last_time_right_sensor = millis();
  } 

  if(debug) Serial << "Distance:   left: " << left_sensor_cm << "cm   front: " 
         << front_sensor_cm << "cm   right: " << right_sensor_cm << "cm\n";
  #endif // USE_DISTANCE_SENSORS


  /// Gyro
  #ifdef USE_GYRO
  bmi.readSensor();
  double read_time = millis();

  gx += ((bmi.getGyroX_rads() - drifts_x) * (read_time - gyro_last_read_time) * 0.001) * 180.0 / PI;
  gy += ((bmi.getGyroY_rads() - drifts_y) * (read_time - gyro_last_read_time) * 0.001) * 180.0 / PI;
  gz -= ((bmi.getGyroZ_rads() - drifts_z) * (read_time - gyro_last_read_time) * 0.001) * 180.0 / PI;

  gyro_last_read_time = read_time;

  if(debug) Serial << "Gyro: gx: " << gx << "    gy: " << gy << "    gz: " << gz << "\n";
  #endif // USE_GYRO

  
  /// Led blink
  #ifdef USE_LED
  if(millis() - led_previous_time >= led_interval) {
    led_previous_time = millis();

    if (led_state == LOW) {
      led_state = HIGH;
    } else {
      led_state = LOW;
    }

    digitalWrite(LED_PIN, led_state);
  }
  #endif // USE_LED


  /// Display
  #ifdef USE_DISPLAY
  if(millis() - display_last_print_time > display_print_interval) {
    // display_print(current_side, " side");
    display_print((millis() - loop_start_time) / 1000, (millis() - loop_start_time) % 1000);
    // display_print(left_sensor_cm, right_sensor_cm);
    display_last_print_time = millis();
  }
  #endif // USE_DISPLAY



  /// Main code

  switch( loop_case ) {
    case MAIN_CASE: {

      if(turns == 0)
          pid_error = (gz - current_angle) * kp0 + integral * ki0 - (pid_last_error - pid_error) * kd0;
      else
        pid_error = (gz - current_angle) * kp + integral * ki - (pid_last_error - pid_error) * kd;
      integral += pid_error;
      pid_last_error = pid_error;

      move_servo(pid_error);

      if(debug) Serial << "Servo angle: " << pid_error << "\nStraight angle: " << current_angle << "\n";


      if(turn_direction == 0 && front_sensor_cm < 110) {
        if(right_sensor_cm > 100)
          turn_direction = 1;
        if(left_sensor_cm > 100)
          turn_direction = -1;
          
        if(turn_direction) {
          if(debug) Serial << "Turn direction is: " << turn_direction << "\n";
        }
      }

      if(front_sensor_cm > 0 && front_sensor_cm < distance_to_wall_for_turn && turn_direction != 0) {
          if(millis() - last_turn_time > turn_delay) {
            if(turn_direction ==  -1)
              current_angle += turn_angle_left * turn_direction;
            else
                current_angle += turn_angle * turn_direction;

            turns++;
            last_turn_time = millis();
            motor_start(motor_speed);
          }
      }

      if(millis() - last_turn_time > turn_delay && turns == 1) 
        limit_clamp = 1;
      
      if(debug) Serial << "Turns: " << turns << "\n";

      // Last turn
      if(turns == 12) {
        last_drive_start_cm = read_motor_encoder();
        turns++;
      }

      // Stop
      if(turns == 13 && read_motor_encoder() - last_drive_start_cm >= last_drive_cm) {
        motor_stop();
        delay(100000);
      }
      break;
    }

    case WAIT_CASE: {
      if(millis() - wait_last_time > wait_time){
        motor_start(motor_speed * wait_speed_direction);
        
        if(wait_speed_direction == 1)
          loop_case = MAIN_CASE;
          
        last_turn_time = millis();
        integral = 0;
      }

      break;
    }
  }



  /// SD card
  #ifdef USE_SD
  if(millis() - sd_last_write_time > sd_interval) {
    data_string = "itr: #" +  String(loop_iteration) + " , " + String((millis() - loop_start_time) / 1000)
      + "." + String((millis() - loop_start_time) % 1000) + "s , " + String(loop_freq) + "hz  ,  ";
    data_string += "state: " + String(loop_case) + "  ,  ";
    data_string += "turns: " + String(turns) + "  ,  ";
    data_string += "enc: " +  String(read_motor_encoder()) + "cm  ,  ";
    data_string += "left: " + String(left_sensor_cm) + "cm,   front: "
      + String(front_sensor_cm) + "cm,   right: " + String(right_sensor_cm) + "cm  ,  ";
    data_string += "gyro: " + String(gz) + "deg  ,  ";
    data_string += "pd err: " + String(pid_error) + "  ,  ";
    data_string += "str ang: " + String(current_angle) + "deg  ,  ";
    data_string += "turn dir: " + String(turn_direction) + "  ,  ";
    data_string += "spd: " + String(current_motor_speed) + " , ";

    file_println(data_string);

    sd_last_write_time = millis();
  }
  #endif // USE_SD
}


///====================///
///      Comments      ///
///====================///

