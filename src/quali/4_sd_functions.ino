/// SD card functions

void SD_setup() {
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present!");
    //init_sd_error = true;
  }
  else {
    Serial.println("Card present!");

    // SD.remove("datalog.txt");

    int i = 0;
    do {
      char aux[100];
      itoa(i, aux, 10);

      memset(sd_filename, 0, sizeof(sd_filename));

      strcat(sd_filename, "datalog_");
      strcat(sd_filename, aux);
      strcat(sd_filename, ".csv");

      i++;
    } while(SD.exists(sd_filename));

    if(debug) Serial.print("Using filename: ");
    if(debug) Serial.println(sd_filename);


    // String aux;

    // if(!init_error) aux = "ok!"; else aux = "error!";
    // data_string = "init: ," + aux + "\n";

    // if(!init_sensors_error) aux = "ok!"; else aux = "error!";
    // data_string += "dist s: ," + aux + "\n";
    
    // if(!init_gyro_error) aux = "ok!"; else aux = "error!";
    // data_string += "gyro: ," + aux + "\n";

    // data_string += "drift: ," + String(drifts_x, 6) + "\n";

    // file_println(data_string);
  }
}

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

// String cube_log_at_index(int i) {
//   return ",s: " + String(pixy.ccc.blocks[i].m_signature) + ",x: "
//         + String(pixy.ccc.blocks[i].m_x) + ",y: " + String(pixy.ccc.blocks[i].m_y) + ",w: "
//         + String(pixy.ccc.blocks[i].m_width) + ",h: " + String(pixy.ccc.blocks[i].m_height) + ",ind: "
//         + String(pixy.ccc.blocks[i].m_index) + ",age: " + String(pixy.ccc.blocks[i].m_age) + ", ";
// }

void writeSD() {

  data_string += "time " + String(millis()) + " , "; 
  data_string += "left " + String(wall_dist[LEFT]) +  " , right " + String(wall_dist[RIGHT]) + " , FRONT " + String(wall_dist[FRONT]) + " , ";
  data_string += "pd_case: " + String(pid_case) + " , ";
  data_string += "pd err: " + String(pid_error) + " ,gx " + String(gx)+ '\n';

  if(millis() - sd_last_write_time > 1000) {
    file_println(data_string);
    data_string = "";
    sd_last_write_time = millis();  
  }

  /// SD card
  #ifdef USE_SD
  if(millis() - sd_last_write_time > sd_interval) {
    data_string = "itr: #" +  String(loop_iteration) + " , " + String((millis() - loop_start_time) / 1000)
      + "." + String((millis() - loop_start_time) % 1000) + "s , " + String(loop_freq) + "hz  ,  ";
    data_string += "state: " + String(loop_case) + "  ,  ";
    data_string += "turns: " + String(turns) + "  ,  ";
    data_string += "enc: " +  String(read_motor_encoder()) + "cm  ,  ";
    // data_string += "left: " + String(left_sensor_cm) + "cm,   front: "
    //   + String(front_sensor_cm) + "cm,   right: " + String(right_sensor_cm) + "cm  ,  ";
    data_string += "gyro: " + String(gx) + "deg  ,  ";
    data_string += "pd err: " + String(pid_error) + "  ,  ";
    data_string += "str ang: " + String(current_angle) + "deg  ,  ";
    data_string += "turn dir: " + String(turn_direction) + "  ,  ";
    data_string += "spd: " + String(current_motor_speed) + " , ";
    data_string += "d dist: " + String(delta_dist) + " , ";
    data_string += "d gyro: " + String(delta_gyro) + ", ";
    data_string += "d displ: " + String(delta_displ) + ", ";
    data_string += "line: " + String(sees_line) + ", ";

    if (pixy.ccc.numBlocks >= 1)
      data_string += cube_log_at_index(0);
    if (pixy.ccc.numBlocks >= 2)
      data_string += cube_log_at_index(1);
    if (pixy.ccc.numBlocks >= 3)
      data_string += cube_log_at_index(2);
    if (pixy.ccc.numBlocks >= 4)
      data_string += cube_log_at_index(3);

    file_println(data_string);

    sd_last_write_time = millis();
  }
  #endif // USE_SD
}