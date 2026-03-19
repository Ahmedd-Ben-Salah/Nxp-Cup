#include <Pixy2.h>
#include <Servo.h>

Pixy2 pixy;
Servo steeringServo;

// --- Pin Configuration ---
const int SERVO_PIN = 8;
const int RIGHT_RPWM = 2;
const int RIGHT_LPWM = 3;
const int LEFT_RPWM = 4;
const int LEFT_LPWM = 5;

// --- Steering & Tuning Parameters ---
const int CENTER_ANGLE = 90;
const int MAX_STEER = 35; // Allow a bit more steering range for sharp corners

// PID Tuning Variables (Tune these on the track)
// P: Proportional - Reacts to current error
// I: Integral - Reacts to accumulated error (fixes drift on long curves)
// D: Derivative - Reacts to rate of change (dampens oscillation/wobble)
float Kp = 1.1;  
float Ki = 0.01; 
float Kd = 0.5;  

int last_error = 0;
int integral = 0;

// --- Speed Parameters ---
// Adaptive speed: go fast on straights, slow down in corners
const int MAX_SPEED = 120;   // Speed on perfect straight
const int MIN_SPEED = 80;    // Speed on sharpest corner
const int SCREEN_CENTER_X = 39; // X-axis center of the Pixy2 camera
const int SCREEN_WIDTH = 78;

// Keep track of the last known good error to extrapolate if lines are lost briefly
int last_known_error = 0;
unsigned long lost_line_time = 0;
const unsigned long MAX_LOST_TIME = 500; // Stop if lost for more than 500ms

void setup() {
  Serial.begin(115200);

  // 1. Initialize Steering
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(CENTER_ANGLE);

  // 2. Initialize Motors
  pinMode(RIGHT_RPWM, OUTPUT); pinMode(RIGHT_LPWM, OUTPUT);
  pinMode(LEFT_RPWM, OUTPUT); pinMode(LEFT_LPWM, OUTPUT);

  // Set PWM frequency for smoother motor operation
  analogWriteFrequency(RIGHT_RPWM, 20000); analogWriteFrequency(RIGHT_LPWM, 20000);
  analogWriteFrequency(LEFT_RPWM, 20000); analogWriteFrequency(LEFT_LPWM, 20000);
  stopMotors();

  // 3. Initialize Camera
  pixy.init();
  pixy.changeProg("line");

  // 4. Launch Sequence
  Serial.println("System Ready. 3 seconds until launch...");
  delay(1000);
  Serial.println("2...");
  delay(1000);
  Serial.println("1...");
  delay(1000);
  Serial.println("GO!");
}

void loop() {
  int8_t res = pixy.line.getMainFeatures();
  
  // FAILSAFE: No lines detected
  if (res <= 0 || pixy.line.numVectors == 0) {
    if (lost_line_time == 0) {
      lost_line_time = millis(); // Start timer
    }
    
    // If lost for too long, stop the car
    if (millis() - lost_line_time > MAX_LOST_TIME) {
      stopMotors();
      steeringServo.write(CENTER_ANGLE);
      return; 
    } else {
      // Blind recovery: continue steering based on last known error, but reduce speed drastically
      applySteeringAndDrive(last_known_error, true);
      delay(10);
      return;
    }
  } 
  
  // Line found, reset lost timer
  lost_line_time = 0;

  int target_x = SCREEN_CENTER_X; 

  // --- SMART NOISE FILTERING ---
  int left_line_x = -1;
  int right_line_x = -1;
  
  int closest_left_y0 = -1;  
  int closest_right_y0 = -1;

  for (int i = 0; i < pixy.line.numVectors; i++) {
    int x0 = pixy.line.vectors[i].m_x0; // Tail (closest to car)
    int y0 = pixy.line.vectors[i].m_y0; 
    int x1 = pixy.line.vectors[i].m_x1; // Head (furthest from car)
    int y1 = pixy.line.vectors[i].m_y1;

    // RULE 1: Horizon / Depth Filter -> Ignore distant noise
    if (y0 < 20) continue; 

    // RULE 2: Intersection / Crosswalk Filter (Horizontal Lines)
    // If dy is very small compared to dx, it's a cross line, ignore it.
    if (abs(x1 - x0) > abs(y1 - y0) * 2.0) continue;

    // Sort valid vectors into Left and Right
    if (x0 < SCREEN_CENTER_X) {
      if (y0 > closest_left_y0) {
        closest_left_y0 = y0;
        left_line_x = x1; 
      }
    } 
    else {
      if (y0 > closest_right_y0) {
        closest_right_y0 = y0;
        right_line_x = x1;
      }
    }
  }

  // --- DETERMINE TARGET (PATH CENTER) ---
  if (left_line_x != -1 && right_line_x != -1) {
    // Both lines seen: ideal state
    target_x = (left_line_x + right_line_x) / 2;
  } 
  else if (left_line_x != -1 && right_line_x == -1) {
    // Only left line seen: estimate center by shifting right
    target_x = left_line_x + 36; 
  }
  else if (right_line_x != -1 && left_line_x == -1) {
    // Only right line seen: estimate center by shifting left
    target_x = right_line_x - 36; 
  }
  else {
    // All vectors filtered out as noise
    target_x = SCREEN_CENTER_X; 
  }

  // Calculate generic error
  int error = target_x - SCREEN_CENTER_X; 
  last_known_error = error;

  // Execute Control Loop
  applySteeringAndDrive(error, false);
  
  delay(10); // Run loop slightly faster for quicker PID response
}

// --- CORE CONTROL FUNCTION ---
void applySteeringAndDrive(int error, bool is_recovery) {
  // 1. PID Calculation
  int P = error * Kp;
  
  // Anti-windup for integral part to prevent massive overshoot
  integral += error;
  integral = constrain(integral, -100, 100); 
  int I = integral * Ki;
  
  int D = (error - last_error) * Kd;
  
  int steering_adjustment = P + I + D;
  last_error = error;
  
  // 2. Servo Actuation
  // Depending on hardware setup, + might need to be -
  int final_angle = CENTER_ANGLE + steering_adjustment;
  final_angle = constrain(final_angle, CENTER_ANGLE - MAX_STEER, CENTER_ANGLE + MAX_STEER);
  steeringServo.write(final_angle);
  
  // 3. Adaptive Speed Control
  // If we are steering hard, reduce speed
  int abs_steer = abs(final_angle - CENTER_ANGLE); 
  
  // Map steering angle to a speed penalty
  // When steer is 0 -> speed is MAX_SPEED
  // When steer is MAX_STEER -> speed is MIN_SPEED
  int straightness_speed = map(abs_steer, 0, MAX_STEER, MAX_SPEED, MIN_SPEED);
  
  // Constrain just in case mapping goes out of bounds
  int target_speed = constrain(straightness_speed, MIN_SPEED, MAX_SPEED);

  if (is_recovery) {
    target_speed = MIN_SPEED; // Slower during blind recovery
  }

  // 4. Differential Drive (Electronic Differential)
  // When turning, outside wheel must spin faster than inside wheel
  int left_speed = target_speed;
  int right_speed = target_speed;

  // Adjusting differential based on steering ratio
  // If turning right (angle > 90), right wheel slows down, left wheel speeds up
  float diff_factor = 1.0; 
  if (final_angle > CENTER_ANGLE) {
    // Turning Right
    diff_factor = 1.0 - (float(abs_steer) / MAX_STEER) * 0.4; // up to 40% reduction on inner wheel
    right_speed = target_speed * diff_factor;
  } else if (final_angle < CENTER_ANGLE) {
    // Turning Left
    diff_factor = 1.0 - (float(abs_steer) / MAX_STEER) * 0.4; 
    left_speed = target_speed * diff_factor;
  }

  drive(left_speed, right_speed);
}

// --- Motor Helpers ---
// Independent wheel speed control
void drive(int left_speed, int right_speed) {
  // Ensure we don't send negative values or exceeding values
  left_speed = constrain(left_speed, 0, 255);
  right_speed = constrain(right_speed, 0, 255);

  analogWrite(RIGHT_RPWM, right_speed); analogWrite(RIGHT_LPWM, 0);
  analogWrite(LEFT_RPWM, left_speed); analogWrite(LEFT_LPWM, 0);
}

void driveForward(int speed) {
  drive(speed, speed);
}

void stopMotors() {
  analogWrite(RIGHT_RPWM, 0); analogWrite(RIGHT_LPWM, 0);
  analogWrite(LEFT_RPWM, 0); analogWrite(LEFT_LPWM, 0);
}
