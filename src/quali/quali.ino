#include <math.h>
#include <string.h>
#include <PrintStream.h>

#include <Servo.h> 
#include <Wire.h>
#include "BMI088.h"
#include <Encoder.h>
#include <Servo.h> 

#include <SD.h>
#include <SPI.h>

// SD Card
#define chipSelect BUILTIN_SDCARD
#define BUTTON_PIN 24




// SD card
const long sd_interval = 20;
double sd_last_write_time = 0;
char sd_filename[100];
String data_string = "";
bool debug = true;


#define Vector Point


// ============================= GEOMETRIC OBJECTS =============================

struct Point {
    int16_t x; 
    int16_t y;

    Point() : x(0), y(0) {}
    Point(int16_t x, int16_t y) : x(x), y(y) {}
};

struct Angle {
  double dist; 
  double ang;

  // Default constructor
  Angle() : dist(0), ang(0) {}

  // Parameterized constructor
  Angle(double dist, double ang) : dist(dist), ang(ang) {}
};


const int MAX_CNT_POINTS = 2000;

Point points[MAX_CNT_POINTS];
int cntPoints= 0;

Angle angles[MAX_CNT_POINTS];

double gx, gy, gz;

int current_motor_speed = 100;

#define FRONT 0
#define RIGHT 1
#define BACK 2
#define LEFT 3

#define WALL_NR 4
int wall_dist[WALL_NR];
int wall_inv[WALL_NR] = {BACK, LEFT, FRONT, RIGHT};

// ============================= SETUP =============================

double start_lidar_time = 0;

void setup() {
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  servo_setup();
  move_servo(0);

  gyro_setup(true);

  SD_setup();
  lidarSetup();
  Serial.println("Finished Lidar");

  Serial.println("Finised seting up the lidar");

  motor_driver_setup();

  int button_flag = 0;
  
  while(digitalRead(BUTTON_PIN) == 1) {
    lidarRead();
    read_gyro(false);
    Serial << digitalRead(BUTTON_PIN) << '\n';
  }

  motor_start(100);
}

// ============================= LOOP =============================

double d1, d2, d3, d4; //left front right back

long last_gyro_read;
long gyro_read_interval = 1;

double kp = 0.0013;
double ki = 0;
double kd = 0.0040;
double pid_error = 1, pid_last_error = 0;

double kp_gyro = 0.025;
double ki_gyro = 0;
double kd_gyro = 0.035;
double pid_error_gyro, pid_last_error_gyro = 0;

long long numberPoints = 0;

double last_time_pid = millis();

double flag_time = 0;
int flagg = 0;

#define SECTION 0 
#define ROTATE 1
#define GO_SECTION 2
#define STOP 10

int CASE = SECTION;
double current_angle = -4.3;
int turns = 0;
int pid_case = 0;

int last_time1 = millis();
int cnt1 = 0;
int loop_cnt = 0;

bool gyro_flag, accel_flag;

int direction = 0;

double last_rotate = 0;

void loop() {
  lidarRead();
  read_gyro(false);

  if(millis() - last_gyro_read > 10) {
    Serial << "ind: " << loop_cnt << " C: " << CASE << " f: " << wall_dist[FRONT] << " r: " << wall_dist[RIGHT]
      << " b: " << wall_dist[BACK] << " l: " << wall_dist[LEFT] <<  " " << gx << " " << pid_error << " " << direction << "\n";
    last_gyro_read = millis();
    writeSD(0, (wall_dist[FRONT] > 0 && wall_dist[FRONT] < 500), 
    (wall_dist[LEFT] && wall_dist[RIGHT] && wall_dist[FRONT] && wall_dist[LEFT] + wall_dist[RIGHT] > 1500 && wall_dist[FRONT] < 800),
    abs(gx - current_angle) < 10);
  }

  if((wall_dist[LEFT] + wall_dist[RIGHT]) > 1200 && !direction) { 
    if(wall_dist[LEFT] > wall_dist[RIGHT])
      direction = -1;
    else
      direction = 1;
  } 

  
  switch(CASE) {
    case SECTION: {
      if(wall_dist[FRONT] > 0 && wall_dist[FRONT] < 500) {
        writeSD(1, -1, -1, -1);
        pid_case = 0;
        current_angle += direction * 90;
        turns++;
        last_rotate = millis();
        CASE = ROTATE;
      }
      else if(abs(pid_error) <= 0.15 || abs(wall_dist[RIGHT] - wall_dist[LEFT]) > 1200) {
        pid_case = 1;
        pid_error_gyro = (current_angle - gx) * kp_gyro + (pid_error_gyro - pid_last_error_gyro) * kd_gyro;
        pid_last_error_gyro = pid_error_gyro;

        move_servo(pid_error_gyro); 
      }
      else {
        if(millis() - last_time_pid > 10 && wall_dist[LEFT] && wall_dist[RIGHT] && abs(wall_dist[RIGHT] - wall_dist[LEFT]) < 1200) {
          pid_case = 2;
          move_servo(pid_error); 
          pid_error = (wall_dist[RIGHT] - wall_dist[LEFT]) * kp + (pid_error - pid_last_error) * kd;
          pid_last_error = pid_error;
          last_time_pid = millis();
        }
      }
      break;
    }
    case ROTATE: {
      if((wall_dist[RIGHT] - wall_dist[LEFT]) < 1200 && abs(current_angle - gx) < 5 && millis() - last_rotate >= 1000) {
        if(turns >= 12) 
          CASE = STOP;
        else
          CASE = SECTION;
      } else {
        pid_error_gyro = (current_angle - gx) * kp_gyro + (pid_error_gyro - pid_last_error_gyro) * kd_gyro;
        pid_last_error_gyro = pid_error_gyro;

        move_servo(pid_error_gyro); 
      } 
      break;
    }
    case STOP: {
      delay(500);
      move_servo(-1);
      motor_start(-5);
      Serial.println("Stop case");
      writeSD(-1, -1, -1, -1);
      delay(100000);
      break;
    }
    default: {
      break;
    }
  }

}