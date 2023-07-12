void sensors_setup() {
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

   /// Camera
  #ifdef USE_CAMERA
  if (debug) Serial.println(F("Cameras starting..."));
  display_print("Cam err!");
  pixy.init(0x54);
  display_print("Camera ok!");
  if (debug) Serial.println(F("Cameras ok!"));
  #endif  // USE_CAMERA

  /// Servo
  #ifdef USE_SERVO
  servo.attach(SERVO_PIN, 1230, 1770);
  move_servo(0);
  delay(150);
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
      // gy += (bmi.getGyroY_rads() * (read_time - gyro_last_read_time) * 0.001);
      // gz += (bmi.getGyroZ_rads() * (read_time - gyro_last_read_time) * 0.001);

      gyro_last_read_time = read_time;
    }

    drifts_x = gx / DRIFT_TEST_TIME;
    // drifts_y = gy / DRIFT_TEST_TIME;
    // drifts_z = gz / DRIFT_TEST_TIME;

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

    data_string += "drift: ," + String(drifts_x, 6) + "\n";

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