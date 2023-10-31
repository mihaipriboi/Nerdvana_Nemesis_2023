void loop_update() {
  /// Print loop iteration
  if(millis() - loop_last_time > 100) {
    loop_freq = loop_temp_iteration * 1000 / (millis() - loop_last_time);
    loop_temp_iteration = 0;
    loop_last_time = millis();
  }

  loop_iteration++;
  loop_temp_iteration++;

  if(debug) Serial << "Iteration: #" << loop_iteration << " - " << millis() / 1000 
         << "." << millis() % 1000 <<  "s - " << loop_freq << "hz\n"; 


  /// Camera
  #ifdef USE_CAMERA
    if (millis() - camera_last_fps >= camera_fps) {
      const int max_cube_nr = 3;
      int n_cubes = pixy.ccc.getBlocks(max_cube_nr);
      pixy.ccc.blocks[0].print();

      cube.color = sig_to_col[pixy.ccc.blocks[i].m_signature];
      cube.x = 316 - pixy.ccc.blocks[i].m_x;
      cube.y = pixy.ccc.blocks[i].m_y;
      cube.w = pixy.ccc.blocks[i].m_width;
      cube.h = pixy.ccc.blocks[i].m_height;
      cube.index = pixy.ccc.blocks[i].m_index;

      camera_last_fps = millis();
    }
  #endif  // USE_CAMERA

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
  delta_start = millis();
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
  delta_dist = millis() - delta_start;
  if(debug) Serial << "Distance:   left: " << left_sensor_cm << "cm   front: " 
         << front_sensor_cm << "cm   right: " << right_sensor_cm << "cm\n";
  #endif // USE_DISTANCE_SENSORS


  /// Gyro
  #ifdef USE_GYRO
  delta_start = millis();
  bmi.readSensor();
  double read_time = millis();

  gx += ((bmi.getGyroX_rads() - drifts_x) * (read_time - gyro_last_read_time) * 0.001) * 180.0 / PI;
  // gy -= ((bmi.getGyroY_rads() - drifts_y) * (read_time - gyro_last_read_time) * 0.001) * 180.0 / PI;
  // gz -= ((bmi.getGyroZ_rads() - drifts_z) * (read_time - gyro_last_read_time) * 0.001) * 180.0 / PI;

  gyro_last_read_time = read_time;

  delta_gyro = millis() - delta_start;
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
  delta_start = millis();
  if(millis() - display_last_print_time > display_print_interval) {
    // display_print(current_side, " side");
    display_print((millis() - loop_start_time) / 1000, (millis() - loop_start_time) % 1000);
    // display_print(left_sensor_cm, right_sensor_cm);
    display_last_print_time = millis();
  }
  delta_displ = millis() - delta_start;
  #endif // USE_DISPLAY
}