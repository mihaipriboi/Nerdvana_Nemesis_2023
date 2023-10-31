/// Motor functions

void motor_start(int speed) {
  speed = -speed;
  current_motor_speed = speed;
  
  int out = abs(speed) * 2.55; // Convert speed to PWM value (0 to 255)
  if(speed >= 0) { // Forward direction
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else { // Reverse direction
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
  return (0.01285) * (double)myEnc.read();
}