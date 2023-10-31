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
#define USE_CAMERA
#define USE_DISTANCE_SENSORS
#define USE_GYRO
#define USE_LED
#define USE_DISPLAY
#define USE_SD
// #define USE_BUTTON
#define USE_MOTOR_SAFETY

/// State machine cases
#define CUBE_PID_CASE 0
#define AVOID_CASE 1
#define TURN_CASE 2
#define GYRO_PID_CASE 3
#define TURN_TO_CENTER_CASE 4
#define GO_STRAIGHT_CASE 5
#define FINISH_CASE 6


///====================///
///   Initialization   ///
///====================///


// Motor encoder
Encoder myEnc(ENCODER_PIN1, ENCODER_PIN2);

// Servo
Servo servo;

// Camera
Pixy2I2C pixy_left;
Pixy2I2C pixy_right;

// Distance sensors
Ultrasonic ultrasonic_front(FRONT_SENSOR_PIN);
Ultrasonic ultrasonic_left(LEFT_SENSOR_PIN);
Ultrasonic ultrasonic_right(RIGHT_SENSOR_PIN);

// Gyro sensor
Bmi088 bmi(Wire, 0x19, 0x69);

// Display
U8G2_SSD1306_128X64_ALT0_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);


///====================///
///      Variables     ///
///====================///

// Motor driver
int motor_speed = 50;
int medium_motor_speed = 50;
int current_motor_speed = 0;
double motor_speed_interval = 150;
double last_time_motor_speed = 0;
double last_motor_encoder = 0;

unsigned long safety_timeout = 1500;
unsigned long last_safety_time = 0;
int last_motor_enc = -999;

// Servo
double delay_angle = 165 / 60;
double servo_correction = -0.15;

// Camera
const double camera_fps = (1.0 / CAMERA_FPS) * 1000.0;
double camera_last_fps = 0;
int min_area_cube = 0;
int max_area_cube = 25000;

const int camera_w = 316;
const int camera_h = 208;
const int camera_joint_width = 72;

const int image_w = camera_w * 2 - camera_joint_width;
const int image_h = camera_h;

const int sig_to_col[] = { 0, 1, -1 };  // 0 - none, -1 - green, 1 - red

int cube_array_size = 0;

struct Cube {
  int color;  // -1 - green, 1 - red
  int x, y;
  int w, h;
  int index;
} cube_array[10], corner_cube, cube;

int ignore_index = -10000000;

// Distance sensors
int left_sensor_cm = 0, right_sensor_cm = 0, front_sensor_cm = 0;
unsigned long last_time_front_sensor = 0, last_time_left_sensor = 0, last_time_right_sensor = 0;

// Gyro sensor
double gx, gy, gz;
unsigned long gyro_last_read_time = 0;
double drifts_x, drifts_y, drifts_z;

// Led
int led_state = LOW;                  // led_state used to set the LED
const long led_interval = 1000;       // led_interval at which to blink (milliseconds)
unsigned long led_previous_time = 0;  // will store last time LED was updated

// Display
const unsigned long display_print_interval = 200;
unsigned long display_last_print_time = 0;

// SD card
const long sd_interval = 5;
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
unsigned long loop_start_time = 0;
unsigned long loop_iteration = 0;
unsigned long loop_temp_iteration = 0;
unsigned long loop_last_time = 0;
long loop_freq = 0;

bool debug = false;

unsigned long last_side_read = 0;
unsigned long side_read_timeout = 250;
int distance_between_walls = 130;

int turn_direction = 0;
int actual_cube_nr;

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
#endif  // USE_DISPLAY
}

void display_print(const char *s1, const char *s2) {
#ifdef USE_DISPLAY
  u8g2.clearBuffer();
  u8g2.drawStr(30, 15, s1);
  u8g2.drawStr(30, 25, s2);
  u8g2.sendBuffer();
#endif  // USE_DISPLAY
}

void display_print(const double n, const char *sufix = "") {
#ifdef USE_DISPLAY
  char s[100];
  itoa(n, s, 10);
  strcat(s, sufix);

  u8g2.clearBuffer();
  u8g2.drawStr(30, 20, s);
  u8g2.sendBuffer();
#endif  // USE_DISPLAY
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
#endif  // USE_DISPLAY
}


/// Motor functions

void motor_start(int speed) {
  current_motor_speed = speed;

  int out = abs(speed) * 2.55;  // Convert speed to PWM value (0 to 255)
  if (speed >= 0) {             // Forward direction
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {  // Reverse direction
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
  angle = clamp(angle + servo_correction, -1, 1);

  double angle_deg = 90 + angle * 90.0;  // Convert angle to degrees (0 to 180)
  angle_deg = clamp(angle_deg, 0, 180);
  servo.write(angle_deg);
}


/// camera functions

int sort(const void *cmp1, const void *cmp2) {
  Cube a = *((Cube *)cmp1);
  Cube b = *((Cube *)cmp2);

  return b.x - a.x;
}


void process_cameras() {
  const int max_cube_nr = 3;

  int n_left = pixy_left.ccc.getBlocks(max_cube_nr);
  int n_right = pixy_right.ccc.getBlocks(max_cube_nr);

  actual_cube_nr = 0;

  Cube c;

  cube.color = 0;
  cube.x = cube.y = 0;
  cube.w = cube.h = 0;
  cube.index = -1;

  corner_cube = {0, 0, 0, 0, 0, 0};

  cube_array_size = 0;

  int min_x = 100000, max_x = 0;
  for (int i = 0; i < n_left; i++) {
    c.color = sig_to_col[pixy_left.ccc.blocks[i].m_signature];
    c.x = camera_w - pixy_left.ccc.blocks[i].m_x;
    c.y = pixy_left.ccc.blocks[i].m_y;
    c.w = pixy_left.ccc.blocks[i].m_width;
    c.h = pixy_left.ccc.blocks[i].m_height;
    c.index = pixy_left.ccc.blocks[i].m_index;

    if (debug) Serial << "Cube left:  col:" << c.color << "  x:" << c.x << "  y:" << c.y
                       << "  w:" << c.w << "  h:" << c.h << "  ind:" << c.index;

    if ((c.w * c.h >= min_area_cube && c.w * c.h < max_area_cube) && c.index != ignore_index) {
      cube_array[cube_array_size++] = c;
      if (cube.w * cube.h < c.w * c.h) {
        // if (corner_cube.w * corner_cube.h < cube.w * cube.h && cube.x < 250) {
        //   corner_cube = {cube.color, cube.x, cube.y, cube.w, cube.h, cube.index};
        // }

        cube.color = c.color;
        cube.x = c.x, cube.y = c.y;
        cube.w = c.w, cube.h = c.h;
        cube.index = c.index;
      }
      // else if (corner_cube.w * corner_cube.h < c.w * c.h && c.x < 250) {
      //   corner_cube = c;
      // }

      if (turn_direction == -1) {
        if (min_x > c.x) {
          min_x = c.x;
          corner_cube = c; 
        }
      }

      if (debug) Serial << "  cube ok";
    }

    if (debug) Serial << "\n";
  }

  for (int i = 0; i < n_right; i++) {
    c.color = sig_to_col[pixy_right.ccc.blocks[i].m_signature];
    c.x = camera_w + (camera_w - pixy_right.ccc.blocks[i].m_x) - camera_joint_width;
    c.y = pixy_right.ccc.blocks[i].m_y;
    c.w = pixy_right.ccc.blocks[i].m_width;
    c.h = pixy_right.ccc.blocks[i].m_height;
    c.index = -pixy_right.ccc.blocks[i].m_index - 1;

    if (debug) Serial << "Cube right:  col:" << c.color << "  x:" << c.x << "  y:" << c.y
                       << "  w:" << c.w << "  h:" << c.h << "  ind:" << c.index;

    if ((c.w * c.h >= min_area_cube && c.w * c.h < max_area_cube) && c.index != ignore_index) {
      cube_array[cube_array_size++] = c;
      if (cube.w * cube.h < c.w * c.h) {
        // if (corner_cube.w * corner_cube.h < cube.w * cube.h && cube.x > image_w - 250) {
        //   corner_cube = {cube.color, cube.x, cube.y, cube.w, cube.h, cube.index};
        // }

        cube.color = c.color;
        cube.x = c.x, cube.y = c.y;
        cube.w = c.w, cube.h = c.h;
        cube.index = c.index;
      }
      // else if (corner_cube.w * corner_cube.h < c.w * c.h && c.x > image_w - 250) {
      //   corner_cube = c;
      // }

      if (turn_direction == 1) {
        if (max_x < c.x) {
          max_x = c.x;
          corner_cube = c; 
        }
      }

      if (debug) Serial << "  cube ok";
    }

    if (debug) Serial << "\n";
  }

  qsort(cube_array, cube_array_size, sizeof(cube_array[0]), sort);

  actual_cube_nr = cube_array_size ? 1 : 0;
  for (int i = 1; i < cube_array_size; i++) {
    actual_cube_nr += abs(cube_array[i].x - cube_array[i - 1].x) > 20;
  }

  if (actual_cube_nr == 1)
    corner_cube = {0, 0, 0, 0, 0, 0};
}

/// SD card
void file_print(String s) {
  File data_file = SD.open(sd_filename, FILE_WRITE);
  if (data_file) {
    data_file.print(s);
    data_file.close();
  }
}

void file_println(String s) {
  File data_file = SD.open(sd_filename, FILE_WRITE);
  if (data_file) {
    data_file.println(s);
    data_file.close();
  }
}


/// Main loop

bool is_in_turn_side_sensors() {
  if (left_sensor_cm + right_sensor_cm > distance_between_walls) {
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
  while (!Serial && (millis() < 2000));

  if (Serial) debug = true;
  debug = false;

  /// Led
  #ifdef USE_LED
  pinMode(LED_PIN, OUTPUT);
  led_state = HIGH;
  digitalWrite(LED_PIN, led_state);
  #endif  // USE_LED

  /// Display
  #ifdef USE_DISPLAY
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  display_print("Starting...");
  #endif  // USE_DISPLAY

  /// Camera
  #ifdef USE_CAMERA
  if (debug) Serial.println(F("Cameras starting..."));
  display_print("Left cam err!");
  pixy_left.init(0x54);
  display_print("Right cam err!");
  pixy_right.init(0x55);
  display_print("Cameras ok!");
  if (debug) Serial.println(F("Cameras ok!"));
  #endif  // USE_CAMERA

  /// Motor driver
  #ifdef USE_MOTOR_DRIVER
  pinMode(PWM1, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  #endif  // USE_MOTOR_DRIVER

  /// Servo
  #ifdef USE_SERVO
  servo.attach(SERVO_PIN, 1200, 1800);
  move_servo(0);
  #endif  // USE_SERVO

  /// Wire
  Wire.begin();

  /// Gyro sensor
  #ifdef USE_GYRO
  int status = bmi.begin();
  bmi.setOdr(Bmi088::ODR_400HZ);
  bmi.setRange(Bmi088::ACCEL_RANGE_6G, Bmi088::GYRO_RANGE_500DPS);
  if (status < 0) {
    if (debug) Serial << "BMI Initialization Error!  error: " << status << "\n";
    init_error = init_gyro_error = true;
  } else {
    // Gyro drift calculation
    if (debug) Serial.println("Starting gyro drift calculation...");
    display_print("Starting gyro", "drift test...");

    gx = 0;
    gy = 0;
    gz = 0;

    gyro_last_read_time = millis();

    double start_time = millis();
    while (millis() - start_time < DRIFT_TEST_TIME * 1000) {
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

    if (debug) Serial.print("Drift test done!\nx: ");
    if (debug) Serial.print(drifts_x, 6);
    if (debug) Serial.print("   y: ");
    if (debug) Serial.print(drifts_y, 6);
    if (debug) Serial.print("   z: ");
    if (debug) Serial.println(drifts_z, 6);
  }
  // Gyro value reset
  gx = 0;
  gy = 0;
  gz = 0;

  gyro_last_read_time = millis();
  #endif  // USE_GYRO


  #ifdef USE_SD
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present!");
    init_sd_error = true;
  } else {
    Serial.println("Card present!");

    // SD.remove("datalog.txt");

    int i = 0;
    do {
      char aux[100];
      itoa(i, aux, 10);

      memset(sd_filename, 0, sizeof(sd_filename));

      strcat(sd_filename, "datalog_Official_");
      strcat(sd_filename, aux);
      strcat(sd_filename, ".csv");

      i++;
    } while (SD.exists(sd_filename));

    if (debug) Serial.print("Using filename: ");
    if (debug) Serial.println(sd_filename);


    String aux;

    if (!init_error) aux = "ok!";
    else aux = "error!";
    data_string = "init: ," + aux + "\n";

    if (!init_sensors_error) aux = "ok!";
    else aux = "error!";
    data_string += "dist s: ," + aux + "\n";

    if (!init_gyro_error) aux = "ok!";
    else aux = "error!";
    data_string += "gyro: ," + aux + "\n";

    data_string += "drift: ," + String(drifts_z, 6) + "\n";

    file_println(data_string);
  }
  #endif  // USE_SD


  /// Program start
  if (!init_error) {
    digitalWrite(LED_PIN, LOW);
    display_print("Ready!");
  } else {
    if (init_sensors_error && init_gyro_error)
      display_print("Dist sensors err!", "Gyro err!");
    else if (init_sensors_error)
      display_print("Dist sensors err!");
    else if (init_gyro_error)
      display_print("Gyro err!");
  }

  #ifdef USE_BUTTON
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  delay(10);

  Serial.println(digitalRead(BUTTON_PIN));
  while (digitalRead(BUTTON_PIN) == 1) {
    Serial.println(digitalRead(BUTTON_PIN));
    delay(50);
  }
  #endif


  loop_start_time = millis();
}


///====================///
///        Loop        ///
///====================///

unsigned long turn_delay = 1000;
unsigned long last_turn_time = 0;
int turns = 0;
double turn_angle = 90;
double current_angle = -2;
int distance_to_wall_for_turn = 70;
int distance_to_wall_for_wide_turn = 40;
int last_drive_start_cm = 0;
int last_drive_cm = 30;

int last_turn_cm = 60;
int turn_cm = 80;

int current_side = 1;  // -1 left 0 center 1 right
int flag = 0;
int flag_straight = 0;

int flag_wide = 0;

int flag_wait = 0;
int flag_turn = 0;
unsigned long wait_last_time = 0;
unsigned long wait_time = 1000;
int wait_speed_direction = 0;
int current_left_cm = 0, current_right_cm = 0;
double speedX = 0;

void all_Sensors() {
  /// Print loop iteration
  if (millis() - loop_last_time > 1000) {
    loop_freq = loop_temp_iteration * 1000 / (millis() - loop_last_time);
    loop_temp_iteration = 0;
    loop_last_time = millis();
  }

  loop_iteration++;
  loop_temp_iteration++;

  if (debug) Serial << "Iteration: #" << loop_iteration << " - " << millis() / 1000
                    << "." << millis() % 1000 << "s - " << loop_freq << "hz\n";

  /// Motor encoder
  #ifdef USE_MOTOR_ENCODER
    if (debug) Serial << "Encoder: " << read_motor_encoder() << "\n";

    if (millis() - last_time_motor_speed >= motor_speed_interval) {
      double current_motor_encoder = (double)myEnc.read() / 47.74;
      speedX = (current_motor_encoder - last_motor_encoder) * (1000.0 / motor_speed_interval); 
      last_time_motor_speed = millis();
      last_motor_encoder = current_motor_encoder;
    } 

    Serial.print("Speed:");
    Serial.println(speedX);


  #ifdef USE_MOTOR_SAFETY
    if (millis() - last_safety_time > safety_timeout) {
      if (abs(current_motor_speed) > 0 && abs(read_motor_encoder() - last_motor_enc) < 1) {
        file_println("Safety stop: ," + String(read_motor_encoder()) + ", " + String(last_motor_enc));
        motor_start(0);
        display_print("Box Box!");
        delay(999999999);
      }

      last_safety_time = millis();
      last_motor_enc = read_motor_encoder();
    }
    if (current_motor_speed == 0)
      last_safety_time = millis();

  #endif  // USE_MOTOR_SAFETY

  #endif  // USE_MOTOR_ENCODER

  /// Camera
  #ifdef USE_CAMERA
    if (millis() - camera_last_fps >= camera_fps) {
      process_cameras();

      camera_last_fps = millis();
    }
  #endif  // USE_CAMERA


  /// Distance sensors
  #ifdef USE_DISTANCE_SENSORS
    if (millis() - last_time_front_sensor >= 50) {
      front_sensor_cm = ultrasonic_front.MeasureInCentimeters();
      last_time_front_sensor = millis();
    }

    if (millis() - last_time_left_sensor >= 50) {
      left_sensor_cm = ultrasonic_left.MeasureInCentimeters();
      last_time_left_sensor = millis();
    }

    if (millis() - last_time_right_sensor >= 50) {
      right_sensor_cm = ultrasonic_right.MeasureInCentimeters();
      last_time_right_sensor = millis();
    }

    if (debug) Serial << "Distance:   left: " << left_sensor_cm << "cm   front: "
                      << front_sensor_cm << "cm   right: " << right_sensor_cm << "cm\n";
  #endif  // USE_DISTANCE_SENSORS


  /// Gyro
  #ifdef USE_GYRO
    bmi.readSensor();
    double read_time = millis();

    // speedX += (bmi.getAccelX_mss() * (read_time - gyro_last_read_time) * 0.001);
    // Serial.println("Speed");
    // Serial.println(speedX);

    gx += ((bmi.getGyroX_rads() - drifts_x) * (read_time - gyro_last_read_time) * 0.001) * 180.0 / PI;
    gy += ((bmi.getGyroY_rads() - drifts_y) * (read_time - gyro_last_read_time) * 0.001) * 180.0 / PI;
    gz -= ((bmi.getGyroZ_rads() - drifts_z) * (read_time - gyro_last_read_time) * 0.001) * 180.0 / PI;

    gyro_last_read_time = read_time;

    if (debug) Serial << "Gyro: gx: " << gx << "    gy: " << gy << "    gz: " << gz << "\n";
  #endif  // USE_GYRO


  /// Led blink
  #ifdef USE_LED
    if (millis() - led_previous_time >= led_interval) {
      led_previous_time = millis();

      if (led_state == LOW) {
        led_state = HIGH;
      } else {
        led_state = LOW;
      }

      digitalWrite(LED_PIN, led_state);
    }
  #endif  // USE_LED


  /// Display
  #ifdef USE_DISPLAY
    if (millis() - display_last_print_time > display_print_interval) {
      // display_print(current_side, " side");
      display_print((millis() - loop_start_time) / 1000, (millis() - loop_start_time) % 1000);
      // display_print(left_sensor_cm, right_sensor_cm);
      display_last_print_time = millis();
    }
  #endif  // USE_DISPLAY
}

int viewSide = 0;
double kp = 0.005;
double ki = 0;
double kd = 0.01;
double pid_error, pid_last_error = 0, integral = 0;

double kp_gyro = 0.032;
double ki_gyro = 0;
double kd_gyro = 0.44;
double pid_error_gyro, pid_last_error_gyro = 0, integral_gyro = 0;

int ext_wall_dist = 30;
int mid_wall_dist = 60;
int int_wall_dist = 95;

int wall_dist = 0;

double cube_delay = 400;
double last_cube_time = 0;

int last_cube_color = 0;
int last_cube_index = -10000000;

long avoid_start_time = 0;
long avoid_interval = 300;

#define NO_CUBE -1000
int into_turn_cube = NO_CUBE;
int out_turn_cube = NO_CUBE;

int turn_start_encoder;
int turn_encoder = 40;

int center_angle_value = 15.5;
int center_angle;

int avoid_angle = 26;
int curr_angle;

int loop_case = GO_STRAIGHT_CASE; //CUBE_PID_CASE;

int start_encoder = 0;

int left_dist_turn = 0;

void loop() {
  all_Sensors();

  // At the start
  if (turn_direction == 0) {
    motor_start(motor_speed);

    if (left_sensor_cm > right_sensor_cm)
      turn_direction = -1;
    else 
      turn_direction = 1;

    if (front_sensor_cm < 100 || actual_cube_nr > 1)
      loop_case = CUBE_PID_CASE;
    else if ((turn_direction == -1 && pixy_left.ccc.numBlocks > 0) || (turn_direction == 1 && pixy_right.ccc.numBlocks > 0))
      loop_case = CUBE_PID_CASE;
    else
      loop_case = GO_STRAIGHT_CASE;
  }

  switch (loop_case) {
    case CUBE_PID_CASE: {
      if (cube.color == 0) {
        move_servo(0);
      } else {
        if ((front_sensor_cm > 35 && front_sensor_cm > 0)) {
          // Serial.println(cube.x);
          pid_error = (cube.x - image_w / 2) * kp + integral * ki + (pid_error - pid_last_error) * kd;
          integral += pid_error;
          pid_last_error = pid_error;

          move_servo(pid_error);

          last_cube_color = cube.color;
          last_cube_index = cube.index;
        } else {
          file_println(corner_cube.color);
          // file_println(corner_cube.x);
          
          out_turn_cube = corner_cube.color;

          if (turn_direction == -1) {  // cube on the left side
            if (out_turn_cube == -1) { wall_dist = int_wall_dist; center_angle = -turn_direction * center_angle_value; } 
            else if (out_turn_cube == 0) { wall_dist = mid_wall_dist; center_angle = 0; }
            else if (out_turn_cube == 1) { wall_dist = ext_wall_dist; center_angle = turn_direction * center_angle_value; }
          } else {
            if (out_turn_cube == -1) { wall_dist = ext_wall_dist; center_angle = turn_direction * center_angle_value; } 
            else if (out_turn_cube == 0) { wall_dist = mid_wall_dist; center_angle = 0; } 
            else if (out_turn_cube == 1) { wall_dist = int_wall_dist; center_angle = -turn_direction * center_angle_value; }
          }

          // if we see only 1 cube, we don't want to use the beaconing system
          if (actual_cube_nr == 1) {
            out_turn_cube = 0;
          }

          flag = 0;
          curr_angle = gz;
          flag_straight = 0;
          loop_case = AVOID_CASE;
          avoid_start_time = millis();
          // motor_stop();
          // delay(100000);
        }
      }

      break;
    }

    case AVOID_CASE: {
      // double angle = curr_angle + last_cube_color * (avoid_angle - abs(current_angle - curr_angle) * 0.5);
      double angle = curr_angle + last_cube_color * (avoid_angle);

      
      if ((-last_cube_color) * (gz - angle) > 0 && flag == 0) {
        move_servo(last_cube_color);
        // ignore_index = last_cube_index;
        into_turn_cube = last_cube_color;
      } else if (abs(gz - current_angle) > 4) {
        flag = 1;
        pid_error_gyro = (gz - current_angle) * kp_gyro + integral * ki_gyro - (pid_last_error_gyro - pid_error_gyro) * kd_gyro;
        integral_gyro += pid_error_gyro;
        pid_last_error_gyro = pid_error_gyro;
        move_servo(-pid_error_gyro);
      } else {
        move_servo(0);

        if (front_sensor_cm > 120)
          flag_straight = true;

        if (flag_straight)
          loop_case = GO_STRAIGHT_CASE;
        else {
          loop_case = TURN_CASE;
        }
      }

      break;
    }

    case TURN_CASE: {
      pid_error_gyro = (gz - current_angle) * kp_gyro + integral * ki_gyro - (pid_last_error_gyro - pid_error_gyro) * kd_gyro;
      integral_gyro += pid_error_gyro;
      pid_last_error_gyro = pid_error_gyro;

      move_servo(-pid_error_gyro);
      if (front_sensor_cm < wall_dist) {
        current_angle += turn_angle * turn_direction;
        turns++;
        turn_encoder = 80 - left_sensor_cm;
        loop_case = GYRO_PID_CASE;
      }

      break;
    }

    case GYRO_PID_CASE: {
      if (abs(gz - current_angle) > 8) {
        pid_error_gyro = (gz - current_angle) * kp_gyro + integral * ki_gyro - (pid_last_error_gyro - pid_error_gyro) * kd_gyro;
        integral_gyro += pid_error_gyro;
        pid_last_error_gyro = pid_error_gyro;

        move_servo(-pid_error_gyro);

        turn_start_encoder = read_motor_encoder();
      } else {
        if (out_turn_cube != 0) {
          if (left_sensor_cm + right_sensor_cm > 80 || read_motor_encoder() - turn_start_encoder < 15) {
            pid_error_gyro = (gz - current_angle) * kp_gyro + integral * ki_gyro - (pid_last_error_gyro - pid_error_gyro) * kd_gyro;
            integral_gyro += pid_error_gyro;
            pid_last_error_gyro = pid_error_gyro;

            move_servo(-pid_error_gyro);
          } else {
            // motor_stop();
            loop_case = CUBE_PID_CASE;

            if (turns == 12) {
              start_encoder = read_motor_encoder();
              loop_case = FINISH_CASE;
            }
          }
        } else {
          if (read_motor_encoder() - turn_start_encoder < turn_encoder) {
            pid_error_gyro = (gz - current_angle) * kp_gyro + integral * ki_gyro - (pid_last_error_gyro - pid_error_gyro) * kd_gyro;
            integral_gyro += pid_error_gyro;
            pid_last_error_gyro = pid_error_gyro;

            move_servo(-pid_error_gyro);
          } else {
            loop_case = CUBE_PID_CASE;
            motor_stop();
            delay(100000);

            if (turns == 12) {
              start_encoder = read_motor_encoder();
              loop_case = FINISH_CASE;
            }
          }
        }
      }

      break;
    }

    case GO_STRAIGHT_CASE: {
      if (front_sensor_cm > 110) {
        pid_error_gyro = (gz - current_angle) * kp_gyro + integral * ki_gyro - (pid_last_error_gyro - pid_error_gyro) * kd_gyro;
        integral_gyro += pid_error_gyro;
        pid_last_error_gyro = pid_error_gyro;

        move_servo(-pid_error_gyro);
      } else {
        file_println(cube.color);
        out_turn_cube = cube.color;

        if (turn_direction == -1) {  // cube on the left side
          if (out_turn_cube == -1) { wall_dist = int_wall_dist; center_angle = -turn_direction * center_angle_value; } 
          else if (out_turn_cube == 0) { wall_dist = mid_wall_dist; center_angle = 0; }
          else if (out_turn_cube == 1) { wall_dist = ext_wall_dist; center_angle = turn_direction * center_angle_value; }
        } else {
          if (out_turn_cube == -1) { wall_dist = ext_wall_dist; center_angle = turn_direction * center_angle_value; } 
          else if (out_turn_cube == 0) { wall_dist = mid_wall_dist; center_angle = 0; } 
          else if (out_turn_cube == 1) { wall_dist = int_wall_dist; center_angle = -turn_direction * center_angle_value; }
        }

        // if we see only 1 cube, we don't want to use the beaconing system
        if (actual_cube_nr == 1) {
          out_turn_cube = 0;
        }

        flag = 0;
        curr_angle = gz;
        loop_case = TURN_CASE;
        avoid_start_time = millis();
      }
      break;
    }

    case FINISH_CASE: {
      if( read_motor_encoder() - start_encoder < 10 ){
        pid_error_gyro = (gz - current_angle) * kp_gyro + integral * ki_gyro - (pid_last_error_gyro - pid_error_gyro) * kd_gyro;
        integral_gyro += pid_error_gyro;
        pid_last_error_gyro = pid_error_gyro;

        move_servo(-pid_error_gyro);
      } else{
        motor_stop();
        delay(1000);
      }
      break;
    }
  }



/// SD card
#ifdef USE_SD
  if (millis() - sd_last_write_time > sd_interval) {
    data_string = "itr: #" + String(loop_iteration) + " , " + String((millis() - loop_start_time) / 1000)
                  + "." + String((millis() - loop_start_time) % 1000) + "s , " + String(loop_freq) + "hz  ,  ";
    data_string += "state: " + String(loop_case) + "  ,  ";
    data_string += "turns: " + String(turns) + "  ,  ";
    data_string += "enc: " + String(read_motor_encoder()) + "cm  ,  ";
    data_string += "speed: " + String(speedX) + "cm/s  ,  ";
    data_string += "left: " + String(left_sensor_cm) + "cm,   front: "
                   + String(front_sensor_cm) + "cm,   right: " + String(right_sensor_cm) + "cm  ,  ";
    data_string += "gyro: " + String(gz) + "deg  ,  ";
    data_string += "pd err: " + String(pid_error) + "  ,  ";
    data_string += "str ang: " + String(current_angle) + "deg  ,  ";
    data_string += "turn dir: " + String(turn_direction) + "  ,  ";
    data_string += "cube col: " + String(cube.color) + " ,   ";
    // data_string += "cube col: " + String(cube.color) + "  ,  ";
    // data_string += "corn col: " + String(cube_corner_color) + "  ,  ";
    // data_string += "curret side: " + String(current_side) + "  ,  ";
    // data_string += "last enc: " + String(last_turn_cm) + " , ";
    // data_string += "last time: " + String(millis() - last_turn_time) + " , ";
    // data_string += ", last cube color " + String(last_cube_color);

    if (pixy_left.ccc.numBlocks >= 1) {
      data_string += ", cam 1:,  sig: " + String(pixy_left.ccc.blocks[0].m_signature) + ",  loc x: "
                     + String(camera_w - pixy_left.ccc.blocks[0].m_x) + ",  loc y: " + String(pixy_left.ccc.blocks[0].m_y) + ",  width: "
                     + String(pixy_left.ccc.blocks[0].m_width) + ",  height: " + String(pixy_left.ccc.blocks[0].m_height) + ",  index: "
                     + String(pixy_left.ccc.blocks[0].m_index) + ",  age: " + String(pixy_left.ccc.blocks[0].m_age);
    }

    if (pixy_left.ccc.numBlocks >= 2) {
      data_string += ", cam 1:,  sig: " + String(pixy_left.ccc.blocks[1].m_signature) + ",  loc x: "
                     + String(camera_w - pixy_left.ccc.blocks[1].m_x) + ",  loc y: " + String(pixy_left.ccc.blocks[1].m_y) + ",  width: "
                     + String(pixy_left.ccc.blocks[1].m_width) + ",  height: " + String(pixy_left.ccc.blocks[1].m_height) + ",  index: "
                     + String(pixy_left.ccc.blocks[1].m_index) + ",  age: " + String(pixy_left.ccc.blocks[1].m_age);
    }

    // Second cube
    if (pixy_right.ccc.numBlocks >= 1) {
      data_string += ", cam 2:,  sig: " + String(pixy_right.ccc.blocks[0].m_signature) + ",  loc x: "
                     + String(camera_w + (camera_w - pixy_right.ccc.blocks[0].m_x) - camera_joint_width) + ",  loc y: " + String(pixy_right.ccc.blocks[0].m_y) + ",  width: "
                     + String(pixy_right.ccc.blocks[0].m_width) + ",  height: " + String(pixy_right.ccc.blocks[0].m_height) + ",  angle: "
                     + ",  index: " + String(pixy_right.ccc.blocks[0].m_index) + ",  age: " + String(pixy_right.ccc.blocks[0].m_age);
    }

    if (pixy_right.ccc.numBlocks >= 2) {
      data_string += ", cam 2:,  sig: " + String(pixy_right.ccc.blocks[1].m_signature) + ",  loc x: "
                     + String(camera_w + (camera_w - pixy_right.ccc.blocks[1].m_x) - camera_joint_width) + ",  loc y: " + String(pixy_right.ccc.blocks[1].m_y) + ",  width: "
                     + String(pixy_right.ccc.blocks[1].m_width) + ",  height: " + String(pixy_right.ccc.blocks[1].m_height) + ",  angle: "
                     + ",  index: " + String(pixy_right.ccc.blocks[1].m_index) + ",  age: " + String(pixy_right.ccc.blocks[1].m_age);
    }

    file_println(data_string);

    sd_last_write_time = millis();
  }
#endif  // USE_SD
}


///====================///
///      Comments      ///
///====================///


/// Pixy camera

// pixy_right.ccc.blocks[i].m_signature The signature number of the detected object (1-7 for normal signatures)
// pixy_right.ccc.blocks[i].m_x The x location of the center of the detected object (0 to 316)
// pixy_right.ccc.blocks[i].m_y The y location of the center of the detected object (0 to 208)
// pixy_right.ccc.blocks[i].m_width The width of the detected object (1 to 316)
// pixy_right.ccc.blocks[i].m_height The height of the detected object (1 to 208)
// pixy_right.ccc.blocks[i].m_angle The angle of the object detected object if the detected object is a color code (-180 to 180).
// pixy_right.ccc.blocks[i].m_index The tracking index of the block
// pixy_right.ccc.blocks[i].m_age The number of frames the block has been tracked.
// pixy_right.ccc.blocks[i].print() A member function that prints the detected object information to the serial port