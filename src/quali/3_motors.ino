// Servo 
#define SERVO_PIN 10
#define SERVO_ANGLE_CORECTION 0

// Servo
Servo servo;

/// Motor pins
#define PWM1 7
#define AIN1 9
#define AIN2 8

// Motor Encoder
#define ENCODER_PIN1 5
#define ENCODER_PIN2 4

Encoder myEnc(ENCODER_PIN1, ENCODER_PIN2);


void motor_driver_setup() {
  pinMode(PWM1, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
}

void motor_start(int speed) {
  speed = -speed;  
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
  motor_start(-10); 
}

long read_motor_encoder() {
  return (0.01285) * (double)myEnc.read();
}


double servo_correction = 0;

float clamp(float val, float small, float big) {
  return max(small, min(val, big));
}

/// Servo functions

void move_servo(double angle) {
  angle = clamp(angle + servo_correction, -1, 1);

  double angle_deg = 90 + angle * 90.0;  // Convert angle to degrees (0 to 180)
  angle_deg = clamp(angle_deg, 0, 180);
  servo.write(angle_deg);
}

void servo_setup() {
  // #ifdef QUALI
  // servo.attach(SERVO_PIN, 1320, 1550);
  // #else
  // servo.attach(SERVO_PIN, 1320, 1550);
  // #endif // QUALI

  #ifdef QUALI
  servo.attach(SERVO_PIN, 1400, 1611);
  #else
  servo.attach(SERVO_PIN, 1400, 1611);
  #endif // QUALI

  move_servo(0);
  delay(50);
}