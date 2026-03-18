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
const int MAX_STEER = 30; // Max steering throw (60 to 120 degrees)

// PD Tuning Variables (Adjust these to fix wobbling!)
float Kp = 0.8; // Proportional: How hard it turns toward the center
float Kd = 0.3; // Derivative: Dampens the steering to prevent overshooting
int last_error = 0;

const int BASE_SPEED = 90; // Keep it slow for the first test
const int SCREEN_CENTER_X = 39; // The exact middle of the camera feed

void setup() {
  Serial.begin(115200);

  // 1. Lock Steering Straight
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(CENTER_ANGLE);

  // 2. Initialize Motors (Power off)
  pinMode(RIGHT_RPWM, OUTPUT); pinMode(RIGHT_LPWM, OUTPUT);
  pinMode(LEFT_RPWM, OUTPUT); pinMode(LEFT_LPWM, OUTPUT);

  analogWriteFrequency(RIGHT_RPWM, 20000); analogWriteFrequency(RIGHT_LPWM, 20000);
  analogWriteFrequency(LEFT_RPWM, 20000); analogWriteFrequency(LEFT_LPWM, 20000);
  stopMotors();

  // 3. Initialize Camera
  pixy.init();
  pixy.changeProg("line");

  // 4. The 3-Second Launch Sequence
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
  
  // FAILSAFE: Stop if no lines are seen
  if (res <= 0 || pixy.line.numVectors == 0) {
    stopMotors();
    steeringServo.write(CENTER_ANGLE);
    return; 
  } 
  
  int target_x = SCREEN_CENTER_X; 

  // --- THE STRICT NOISE FILTER ---
  int left_line_x = -1;
  int right_line_x = -1;
  
  // We want to find the lines that start CLOSEST to the car (highest Y value)
  int closest_left_y0 = -1;  
  int closest_right_y0 = -1;

  for (int i = 0; i < pixy.line.numVectors; i++) {
    int x0 = pixy.line.vectors[i].m_x0; // Bottom of the line (tail)
    int y0 = pixy.line.vectors[i].m_y0; // Y ranges from 0 (top) to 51 (bottom)
    int x1 = pixy.line.vectors[i].m_x1; // Top of the line (head)
    int y1 = pixy.line.vectors[i].m_y1;

    // FILTER 1: The Horizon Rule. 
    // If the line starts higher up on the screen (e.g., Y < 25), it's background noise. Ignore it.
    if (y0 < 25) continue; 

    // FILTER 2: The Vertical Rule.
    // Track lines point forward. If the line is mostly horizontal, ignore it.
    if (abs(x1 - x0) > abs(y1 - y0) * 1.5) continue;

    // Now, sort the surviving valid lines into Left and Right
    if (x0 < SCREEN_CENTER_X) {
      // Is this line closer to the car than the last one we checked?
      if (y0 > closest_left_y0) {
        closest_left_y0 = y0;
        left_line_x = x1; // We steer towards the "head" of the line
      }
    } 
    else {
      if (y0 > closest_right_y0) {
        closest_right_y0 = y0;
        right_line_x = x1;
      }
    }
  }

  // --- FIND THE CENTER OF THE TRACK ---
  if (left_line_x != -1 && right_line_x != -1) {
    target_x = (left_line_x + right_line_x) / 2;
  } 
  // EMERGENCY FALLBACK: We only see a valid left line
  else if (left_line_x != -1 && right_line_x == -1) {
    target_x = left_line_x + 35; // Push the car right to stay centered
  }
  // EMERGENCY FALLBACK: We only see a valid right line
  else if (right_line_x != -1 && left_line_x == -1) {
    target_x = right_line_x - 35; // Push the car left to stay centered
  }
  else {
    // If all vectors were filtered out as noise, keep driving straight
    target_x = SCREEN_CENTER_X;
  }

  // --- SMOOTH STEERING MATH (PD CONTROL) ---
  int error = target_x - SCREEN_CENTER_X; 
  
  int P = error * Kp;
  int D = (error - last_error) * Kd;
  int steering_adjustment = P + D;
  last_error = error;
  
  // Apply adjustment (Change the + to a - if your steering is backwards)
  int final_angle = CENTER_ANGLE + steering_adjustment;
  
  // Constrain to protect the servo
  final_angle = constrain(final_angle, CENTER_ANGLE - MAX_STEER, CENTER_ANGLE + MAX_STEER);
  
  // --- ACTUATE ---
  steeringServo.write(final_angle);
  driveForward(BASE_SPEED);
  
  delay(20); 
}

// --- Motor Helpers ---
void driveForward(int speed) {
  analogWrite(RIGHT_RPWM, speed); analogWrite(RIGHT_LPWM, 0);
  analogWrite(LEFT_RPWM, speed); analogWrite(LEFT_LPWM, 0);
}

void stopMotors() {
  analogWrite(RIGHT_RPWM, 0); analogWrite(RIGHT_LPWM, 0);
  analogWrite(LEFT_RPWM, 0); analogWrite(LEFT_LPWM, 0);
}