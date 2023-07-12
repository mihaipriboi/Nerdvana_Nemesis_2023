int loop_case = 2;

int turn_direction = 0;

int current_angle = 2;

int turns = 0;
double turn_delay = 1550;
double last_turn_time = 0;

double kp = 0.019;
double kd = 0.13;
double pid_error, pid_last_error = 0;

int last_drive_start_cm = 0;
int last_drive_cm = 130;

double kp1 = 0.03;
double kd1 = 0.06;
double pid_error1, pid_last_error1 = 0;

void loop_cases() {
  switch( loop_case ) {
    case MAIN_CASE: {

      pid_error = (gx - current_angle) * kp - (pid_last_error - pid_error) * kd;
      pid_last_error = pid_error;

      move_servo(-pid_error);

      if(turn_direction == 0 && front_sensor_cm < 110) {
        if(right_sensor_cm > 100)
          turn_direction = 1;
        if(left_sensor_cm > 100)
          turn_direction = -1;
          
        if(turn_direction) {
          if(debug) Serial << "Turn direction is: " << turn_direction << "\n";
        }
      } 

      // if(millis() - loop_start_time > 1500) {
      //   display_print(read_motor_encoder());
      //   motor_stop();
      //   delay(100000);
      // } 

      // if(millis() - loop_start_time > 1500) {
      //   display_print(read_motor_encoder());
      //   motor_stop();
      //   delay(100000);
      // } 

      // if(right_sensor_cm < 20) {
      //   motor_stop();
      //   delay(100000);
      //   read_motor_encoder();
      // }

      if(front_sensor_cm > 0 && front_sensor_cm < 55 && turn_direction != 0) {
          if(millis() - last_turn_time > turn_delay) {
              current_angle += 89.6 * turn_direction;

            turns++;
            motor_start(motor_speed);
            last_turn_time = millis();
            motor_start(motor_speed);
          }
      }

      // Last turn
      if(turns == 12) {
        motor_stop();
        delay(100000);
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
    case 1: {
      if(left_sensor_cm < 10 || abs(gx - current_angle) > 5) {
        pid_error1 = (20 - left_sensor_cm) * kp1 - (pid_last_error1 - pid_error1) * kd1;
        pid_last_error1 = pid_error1;

        move_servo(pid_error1);
      } else if(right_sensor_cm < 10 || abs(gx - current_angle) > 5) {
        pid_error1 = (right_sensor_cm - 20) * kp1 - (pid_last_error1 - pid_error1) * kd1;
        pid_last_error1 = pid_error1;

        move_servo(pid_error1);
      } else {
        loop_case = 0;
      }
      break;
    }
    
  }
}