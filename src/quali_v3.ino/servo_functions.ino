// Servo
double servo_correction = 0.05;

float clamp(float val, float small, float big) {
  return max(small, min(val, big));
}

/// Servo functions

void move_servo(double angle) {
  angle = clamp(angle + servo_correction, -1, 1);

  double angle_deg = 90 + angle * 90.0;  // Convert angle to degrees (0 to 180)
  angle_deg = clamp(angle_deg, 25, 165);
  servo.write(angle_deg);
}