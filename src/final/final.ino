#include <math.h>
#include <string.h>
#include <PrintStream.h>

#include <Servo.h>
#include <Wire.h>
#include "BMI088.h"
#include <Encoder.h>
#include <Servo.h>
#include <Pixy2I2C.h>

#include <SD.h>
#include <SPI.h>

// SD Card
#define chipSelect BUILTIN_SDCARD
// Camera
#define CAMERA_FPS 30


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

  Point()
    : x(0), y(0) {}
  Point(int16_t x, int16_t y)
    : x(x), y(y) {}
};

struct Angle {
  double dist;
  double ang;

  // Default constructor
  Angle()
    : dist(0), ang(0) {}

  // Parameterized constructor
  Angle(double dist, double ang)
    : dist(dist), ang(ang) {}
};

struct Cube {
  int color;  // -1 - green, 1 - red
  int x, y;
  int w, h;
  int index;
} corner_cube, cube;


const int MAX_CNT_POINTS = 2000;

Point points[MAX_CNT_POINTS];
int cntPoints = 0;

Angle angles[MAX_CNT_POINTS];

double gx, gy, gz;

int current_motor_speed = 100;
int motor_speed = 80;

#define FRONT 0
#define RIGHT 1
#define BACK 2
#define LEFT 3

#define GREEN -1
#define RED 1

#define WALL_NR 4
int wall_dist[WALL_NR];
double slopes[WALL_NR];
int old_wall_dist[WALL_NR];
int wall_inv[WALL_NR] = { BACK, LEFT, FRONT, RIGHT };

// ============================= SETUP =============================


int direction = 1;
int side_wall = LEFT;
double current_angle = 0;

void setup() {
  Serial.begin(115200);

  servo_setup();

  SD_setup();
  camera_setup(true);

  gyro_setup(true);
  Serial.println("Finished Gyro");

  lidarSetup();

  Serial.println("Finised seting up the lidar");

  motor_driver_setup();

  double start_lidar_time = millis();

  while (millis() - start_lidar_time < 1000) {
    lidarRead();
    read_gyro(false);
  }

  while(digitalRead(BUTTON_PIN) == 1) {
    lidarRead();
    read_gyro(false);
  }

  motor_start(motor_speed);
}

// ============================= LOOP =============================

double d1, d2, d3, d4;  //left front right back

long last_gyro_read;
long gyro_read_interval = 1;

long long numberPoints = 0;

double last_time_pid = 0;

double flag_time = 0;
int flagg = 0;

const double camera_interval = (1000 / CAMERA_FPS);
double last_camera_read = 0;
int cube_color = 0;  // -1 green, 0 none, 1 red

#define SECTION 0
#define ROTATE 1
#define GO_SECTION 2
#define AVOID_CUBE 3
#define PASS_CUBE 4
#define PASS_LAST_CUBE 5



#define STOP 10

int CASE = SECTION;
int turns = 0;
int pid_case = 0;

int last_time1 = millis();
int cnt1 = 0;
int loop_cnt = 0;

bool gyro_flag, accel_flag;

double dist_to_cube = 0;
double last_dist_to_cube = 0;
double dist_cube_wall = 0;
double last_cube_y = 0;

double last_read_cube = millis();

#define SECTION_NR 4
#define POSITION_NR 2

// -1 green, 0 none, 1 red
int cube_colors[SECTION_NR][POSITION_NR] = { { -1, 1 }, { -1, -1 }, { 1, 0 }, { -1, 0 } };

int section = 0;
// int current_cube_color = get_cube(section, 0);

int goal_distance = 0;

int last_cube_color = 0;

double last_avoid_cube_time = 0;

double last_rotate = 0;
double rotate_timeout = 2500;
double last_turn_ok = 0;
double xa = 0;
double ya = 0;

double kp = 0.0018;  //0.0013;
double ki = 0;
double kd = 0.003;
double pid_error = 1, pid_last_error = 0;

double kp_gyro = 0.04;
double ki_gyro = 0;
double kd_gyro = 0.25;
double pid_error_gyro, pid_last_error_gyro = 0;

double ang = 0;

bool flag = 0;

int turn_ok = 1;

int final_cube_color = 0;
int final_cube_pos = 0;
int final_cube_turn = 0;

int cube_section_cnt = 0;

void loop() {
  lidarRead();
  read_gyro(false);
  read_camera(true);

  if (millis() - last_gyro_read > 10) {
    Serial << " C: " << CASE << " f: " << wall_dist[FRONT] << " r: " << wall_dist[RIGHT]
           << " b: " << wall_dist[BACK] << " l: " << wall_dist[LEFT] << " " << gx << " " << pid_error << " " 
           << pid_error_gyro << " " << dist_to_cube << " " << last_dist_to_cube << " " << goal_distance << " " 
           << xa << " " << ya << " " << last_cube_y << " " << cube_color << "\n";
    last_gyro_read = millis();
    writeSD();
  }

  if(turns == 12 && millis() - last_rotate > rotate_timeout && wall_dist[BACK] > 1100 && wall_dist[BACK] < 2000 && (wall_dist[LEFT] + wall_dist[RIGHT]) < 1200) {
    CASE = STOP;
  }

  if(direction == 0 && (wall_dist[LEFT] + wall_dist[RIGHT]) > 1200) {
    if(wall_dist[LEFT] > wall_dist[RIGHT]) {
      direction = 1;
      side_wall = LEFT;
    } else {
      direction = -1;
      side_wall = RIGHT;
      current_angle = -5;
    }
  }

  switch (CASE) {

    case SECTION:
      {
        if(turn_ok && wall_dist[FRONT] && wall_dist[FRONT] < 700  && millis() - last_rotate > rotate_timeout) {
          turns++;
          last_rotate = millis();
          last_cube_color = 0;
          last_dist_to_cube = 0;
          flag = 0;
          cube_section_cnt = 0;
          if (wall_dist[side_wall] > 550)
            CASE = ROTATE;
          else
            current_angle += direction * 90;
          
          // CASE = STOP;
        } else if (last_dist_to_cube && cube_color != last_cube_color && cube_color != 0 && wall_dist[BACK] < 1800) {

          // data_string += "a intrat sloboz, cube_color: " + String(cube_color) + ", last_cube_color " + String(last_cube_color) + '\n';

          move_servo(cube_color * 1);

          if (cube_color == GREEN) {
            if(direction == 1)
              goal_distance = 250 + (wall_dist[side_wall] - 230) / 6;
            else
              goal_distance = 850 - (1000 - wall_dist[side_wall]) / 6;
          } else {
            if(direction == 1)
              goal_distance = 770 - (1000 - wall_dist[side_wall]) / 5;
            else
              goal_distance = 270 + (wall_dist[side_wall] - 200) / 5;
          }

          last_cube_color = cube_color;
          // save last cube 

          cube_section_cnt++;

          if(turns == 3) {
            final_cube_color = last_cube_color;
            final_cube_pos = cube_section_cnt;
            final_cube_turn = turns;
          } 
          if(turns == 4) {
            if(last_cube_y < 1700) {
              final_cube_color = last_cube_color;
              final_cube_pos = cube_section_cnt;
              final_cube_turn = turns;
            }
          }
          
          CASE = AVOID_CUBE;
        } else {

          if(millis() - last_turn_ok > rotate_timeout && (wall_dist[BACK] > 1850 || (wall_dist[LEFT] + wall_dist[RIGHT]) > 1200) && !turn_ok) {
            turn_ok = 1;
            last_turn_ok = millis();
          }
          pid_error_gyro = ((current_angle) - gx) * kp_gyro + (pid_error_gyro - pid_last_error_gyro) * kd_gyro;
          pid_last_error_gyro = pid_error_gyro;

          move_servo(pid_error_gyro);
        }
        break;
      }

    case AVOID_CUBE:
      {
        if (last_cube_color == GREEN && ((wall_dist[side_wall] < goal_distance && direction == 1) || (wall_dist[side_wall] > goal_distance && direction == -1))  || 
              last_cube_color == RED && ((wall_dist[side_wall] > goal_distance && direction == 1) || (wall_dist[side_wall] < goal_distance && direction == -1)) && 
                (wall_dist[LEFT] + wall_dist[RIGHT]) > 800) {
          CASE = PASS_CUBE;
        } else {
          pid_error_gyro = ((current_angle + last_cube_color * 55) - gx) * kp_gyro + (pid_error_gyro - pid_last_error_gyro) * kd_gyro;
          pid_last_error_gyro = pid_error_gyro;

          move_servo(pid_error_gyro);
        }
        break;
      }

    case PASS_CUBE:
      {
        if (wall_dist[BACK] > last_cube_y + 200 && (wall_dist[FRONT] + wall_dist[BACK]) < 3400) {
          last_dist_to_cube = 0;
          if(final_cube_color == 1 && final_cube_turn % 4 == turns % 4 && turns > 5 && final_cube_pos == cube_section_cnt)
            CASE = PASS_LAST_CUBE;
          else
            CASE = SECTION;
        } else {
          pid_error_gyro = (current_angle - gx) * kp_gyro + (pid_error_gyro - pid_last_error_gyro) * kd_gyro;
          pid_last_error_gyro = pid_error_gyro;

          move_servo(pid_error_gyro);
        }
        break;
      }

    case ROTATE:
      {
        if (abs((current_angle - direction * 270) - gx) < 5) {
          current_angle -= direction * 270;
          last_rotate = millis();
          CASE = SECTION;
        } else if ((wall_dist[side_wall] > 500 || wall_dist[FRONT] > 450) && (wall_dist[side_wall] > 450 || wall_dist[FRONT] > 250) && flag == 0) {
          ang = myacos((wall_dist[FRONT] - 450) / sqrt((wall_dist[FRONT] - 450) * (wall_dist[FRONT] - 450) + (wall_dist[side_wall] - 500) * (wall_dist[side_wall] - 500)));

          pid_error_gyro = ((current_angle - direction * ang) - gx) * kp_gyro + (pid_error_gyro - pid_last_error_gyro) * kd_gyro;
          pid_last_error_gyro = pid_error_gyro;

          move_servo(pid_error_gyro);
        } else {
          flag = 1;
          pid_error_gyro = ((current_angle - direction * 270) - gx) * kp_gyro + (pid_error_gyro - pid_last_error_gyro) * kd_gyro;
          pid_last_error_gyro = pid_error_gyro;

          move_servo(pid_error_gyro);
        }
        break;
      }

    case PASS_LAST_CUBE: {
      if(wall_dist[FRONT] && wall_dist[FRONT] < 500) {
        current_angle -= 180;
        direction *= -1;
        if(side_wall == LEFT)
          side_wall = RIGHT;
        else
          side_wall = LEFT;
        last_rotate = millis();
        last_cube_color = 0;
        last_dist_to_cube = 0;
        flag = 0;
        cube_section_cnt = 0;
        turns++;
        CASE = SECTION;
      } else {
        pid_error_gyro = (current_angle - gx) * kp_gyro + (pid_error_gyro - pid_last_error_gyro) * kd_gyro;
        pid_last_error_gyro = pid_error_gyro;

        move_servo(pid_error_gyro);
      }
      break;
    }
    
    case STOP:
      {
        lidarMotorWrite(0);
        move_servo(0);
        motor_start(-10);
        Serial.println("Stop case");
        break;
      }

    default:
      {
        break;
      }
  }
}