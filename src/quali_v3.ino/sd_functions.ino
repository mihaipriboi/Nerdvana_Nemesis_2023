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

void writeSD() {
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
    data_string += "gyro: " + String(gx) + "deg  ,  ";
    data_string += "pd err: " + String(pid_error) + "  ,  ";
    data_string += "str ang: " + String(current_angle) + "deg  ,  ";
    data_string += "turn dir: " + String(turn_direction) + "  ,  ";
    data_string += "spd: " + String(current_motor_speed) + " , ";
    data_string += "d dist: " + String(delta_dist) + " , ";
    data_string += "d gyro: " + String(delta_gyro) + ", ";
    data_string += "d displ: " + String(delta_displ) + ", ";

    file_println(data_string);

    sd_last_write_time = millis();
  }
  #endif // USE_SD
}