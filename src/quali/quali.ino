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

void setup() {
  Serial.begin(115200);
  
  servo_setup();

  SD_setup();
  lidarSetup();

  Serial.println("Finised seting up the lidar");

  motor_driver_setup();
  delay(1000);

  gyro_setup(true);
  Serial.println("Finished Lidar");
  motor_start(100);
}

// ============================= LOOP =============================

double d1, d2, d3, d4; //left front right back

long last_gyro_read;
long gyro_read_interval = 1;

double kp = 0.001;
double ki = 0;
double kd = 0.004;
double pid_error = 1, pid_last_error = 0;

double kp_gyro = 0.02;
double ki_gyro = 0;
double kd_gyro = 0.03;
double pid_error_gyro, pid_last_error_gyro = 0;

long long numberPoints = 0;

double last_time_pid = millis();

double flag_time = 0;
int flagg= 0;

#define SECTION 0 
#define ROTATE 1
#define GO_SECTION 2
#define STOP 10

int CASE = SECTION;
double current_angle = 0;
int turns = 0;
int pid_case = 0;

int last_time1 = millis();
int cnt1 = 0;
int loop_cnt = 0;

bool gyro_flag, accel_flag;

void loop() {
  lidarRead();
  read_gyro(false);

  // if(millis() - last_time1 >= 1000) {
  //   Serial.println("Loop cnt: ");
  //   Serial.println(cnt1);
  //   Serial.println(loop_cnt);
  //   cnt1 = loop_cnt = 0;
  //   last_time1 = millis();
  // } 
  // loop_cnt++;

  if(millis() - last_gyro_read > 10) {
    Serial << "ind: " << loop_cnt << " C: " << CASE << " f: " << wall_dist[FRONT] << " r: " << wall_dist[RIGHT]
      << " b: " << wall_dist[BACK] << " l: " << wall_dist[LEFT] <<  " " << gx << " " << pid_error << " " << dist_to_cube <<  "\n";
    last_gyro_read = millis();
    writeSD();
  }

  
  switch(CASE) {
    case SECTION: {

      if(wall_dist[FRONT] > 0 && wall_dist[FRONT] < 800 && abs(gx - current_angle) < 10) {
        pid_case = 0;
        CASE = ROTATE;
      }
      else if(abs(pid_error) <= 0.15 || abs(gx - current_angle) > 25 || (wall_dist[RIGHT] - wall_dist[LEFT]) > 1200) {
        pid_case = 1;
        pid_error_gyro = (current_angle - gx) * kp_gyro + (pid_error_gyro - pid_last_error_gyro) * kd_gyro;
        pid_last_error_gyro = pid_error_gyro;

        move_servo(pid_error_gyro); 
      }
      else {
        if(millis() - last_time_pid > 10 && wall_dist[LEFT] && wall_dist[RIGHT] && (wall_dist[RIGHT] - wall_dist[LEFT]) < 1200) {
          pid_case = 2;
          pid_error = (wall_dist[RIGHT] - wall_dist[LEFT]) * kp + (pid_error - pid_last_error) * kd;
          pid_last_error = pid_error;
          // Serial << pid_error << '\n';
          move_servo(pid_error); 
          last_time_pid = millis();
        }
      }
      break;
    }
    case ROTATE: {
      if(wall_dist[BACK] > 0 && wall_dist[BACK] < 1200) {
        current_angle += 90;
        turns++;
        CASE = GO_SECTION;
      } else {
        move_servo(0.8);
      }
      break;
    }
    case GO_SECTION: {
      if(wall_dist[BACK] > 1000)
        if(turns >= 12) 
          CASE = STOP;
        else
          CASE = SECTION;
      else {
        pid_error_gyro = (current_angle - gx) * kp_gyro + (pid_error_gyro - pid_last_error_gyro) * kd_gyro;
        pid_last_error_gyro = pid_error_gyro;

        move_servo(pid_error_gyro); 
      }
      break;
    }
    case STOP: {
      move_servo(-1);
      motor_start(-10);
      Serial.println("Stop case");
      break;
    }
    default: {
      break;
    }
  }

}
