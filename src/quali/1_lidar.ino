#define lidarSendBuff(buff) Serial1.write(buff, sizeof(buff))

const byte port_lidar_motor_pwm = 2;

const byte buff_start_express_scan[] = {0xA5, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22};
const byte buff_reset[] = {0xA5, 0x40};

// Restarting lidar
void lidarReset() {
    lidarSendBuff(buff_reset);
}

// Starting express scanning
void lidarStartExpressScan() {
    lidarSendBuff(buff_start_express_scan);
}

// Reading the lidar response
void lidarReadDescription() {
    for (int i = 0; i < 7; ++i) {
        long lastTime = millis();
        Serial.println(i);
        while (!Serial1.available()) {};
        Serial1.read();
    }
    
    Serial.println("done");
}

// Setting the lidar motor speed
void lidarMotorWrite(int speed_motor) {
    analogWrite(port_lidar_motor_pwm, speed_motor); 
}

// Initializing lidar
void lidarSetup() {
    lidarMotorWrite(255);
    delay(100);
    Serial1.begin(115200);
    lidarReset();
    delay(1000);
    while (Serial1.available()) {
        Serial1.read();
    }
    lidarStartExpressScan();
    lidarReadDescription();
}

// Stopping lidar
void lidarStop() {
    lidarMotorWrite(0);
}

byte lidar_buff[84];
byte lidar_old_buff[84];

int lidar_buff_ptr = 0;
int cnt_points = 0;

double old_angle = 0;

// Getting angle compensation
double getDiffAngle(byte data_angle) {
    bool sign = data_angle >> 7;
    double diff_angle = (double)(data_angle & 0b1111111) / (1 << 3);
    if (sign) {
        diff_angle = -diff_angle;
    }
    return diff_angle;
}


double angleDiff(double old_angle, double now_angle) {
    return old_angle <= now_angle ? now_angle - old_angle : now_angle - old_angle + 360;
}


double d0[2];
uint16_t distance[2];
double last_time = 0;
int flag = 0;


#define MAX_POINTS 1000
#define INF 10000

bool wall_updated[WALL_NR] = {1, 1, 1, 1};

double old_ang = 10000;
double old_dist = 10000;

Point current_line[MAX_POINTS];
int current_line_size = 0;

double calculate_threshold(double distance) {
  double a = 0.0010947040728267199;
  double b = -0.46574727812407213;
  double c = 56.010728111461475;
  double points = a * distance * distance + b * distance + c;
  double threshold = max(7.0, points);  // Ensure threshold is at least 7
  return threshold;
}

void calculate_linear_regression_segment(Point& start, Point& end, double& dist, int& dir) {  
  if(current_line_size < 2) return; // Not enough points for regression
  
  double x_start = INFINITY, x_end = -INFINITY;
  double x_sum = 0, y_sum = 0, x2_sum = 0, xy_sum = 0;
  for(int i = 0; i < current_line_size; ++i) {
    x_sum += current_line[i].x;
    y_sum += current_line[i].y;
    x2_sum += current_line[i].x * current_line[i].x;
    xy_sum += current_line[i].x * current_line[i].y;

    if(current_line[i].x < x_start) x_start = current_line[i].x;
    if(current_line[i].x > x_end) x_end = current_line[i].x;
  }

  double m, c;

  if(current_line_size * x2_sum - x_sum * x_sum == 0) m = INF;
  else m = (current_line_size * xy_sum - x_sum * y_sum) / (current_line_size * x2_sum - x_sum * x_sum);
  
  c = (y_sum - m * x_sum) / current_line_size;

  start.x = x_start;
  start.y = m * x_start + c;
  
  end.x = x_end;
  end.y = m * x_end + c;

  dist = abs(c) / sqrt(1 + m * m);

  if(abs(m) < 1) {
    if((start.y + end.y) / 2 < 0)
      dir = BACK;
    else
      dir = FRONT;
  } else {
    if((start.x + end.x) / 2 < 0)  {
      dir = LEFT; 
    }
    else
      dir = RIGHT;
  }
}

void lidarProcessPoint(Angle angl, Point p) {
  double ang, dist;
  ang = angl.ang;
  dist = angl.dist;

  if(abs(dist - old_dist) < 10 && min(abs(ang - old_ang), 360 - abs(ang - old_ang)) < 1.5) {
    current_line[current_line_size++] = p;
  } else {
      if(current_line_size > 10) {
        Point start, end;
        double wall_d;
        int wall_dir;

        calculate_linear_regression_segment(start, end, wall_d, wall_dir);

        wall_dist[wall_dir] = wall_d;
        wall_updated[wall_dir] = true;
      }

      current_line_size = 0;
      current_line[current_line_size++] = p;
  }

  old_dist = dist;
  old_ang = ang;
}

// Processing data from the lidar
void lidarProcessingData() {

    double start_angle = (((uint16_t)(lidar_buff[3] & 0b01111111) << 8) ^ lidar_buff[2]) / 64.;
    double old_start_angle = (((uint16_t)(lidar_old_buff[3] & 0b01111111) << 8) ^ lidar_old_buff[2]) / 64.;

    for (int i = 0; i < 16; ++i) {
        byte* cabin = lidar_old_buff + 5 * i + 4;
        
        d0[0] = getDiffAngle(((cabin[0] & 0b11) << 4) ^ (cabin[4] & 0b1111));
        d0[1] = getDiffAngle(((cabin[2] & 0b11) << 4) ^ (cabin[4] >> 4));

        for (int i = 0; i < 2; ++i) {
            distance[i] = ((uint16_t)cabin[2 * i + 1] << 6) ^ (cabin[2 * i] >> 2);
        }

        for (int j = 0; j < 2; ++j) {
            int k = 2 * i + 1 + j;
            double angle = old_start_angle + angleDiff(old_start_angle, start_angle) / 32 * k;
            if (angle >= 360) {
                angle -= 360;
            } else if(angle < 0) {
              angle = angle + 360;
            }

            double x, y;

            y = -distance[j] * cos(radians(angle));
            x = -distance[j] * sin(radians(angle));

            if(distance[j] > 5 && distance[j] < 4000) {
              // Serial << x << " " << y << '\n';
              // Serial << distance[j] << " " << angle << '\n';
              lidarProcessPoint(Angle(distance[j], angle), Point(x, y));
            }

        }
    }
}

void lidarRead() {
    if (!Serial1.available()) {
        return;
    }
    byte current_byte = Serial1.read();
    if (lidar_buff_ptr == 0) {
        if ((current_byte >> 4) != 0xA) {
            return;
        }
    } else if (lidar_buff_ptr == 1) {
        if ((current_byte >> 4) == 0xA) {
            lidar_buff_ptr = 0;
        } else if ((current_byte >> 4) != 0x5) {
            lidar_buff_ptr = 0;
            return;
        }
    }

    lidar_buff[lidar_buff_ptr++] = current_byte;

    if (lidar_buff_ptr == 4) {
        lidarProcessingData();
    }

    if (lidar_buff_ptr == 84) {
        memcpy(lidar_old_buff, lidar_buff, 84);
        lidar_buff_ptr = 0;
    }
}
