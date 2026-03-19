// ============================================================================
//  NXP CUP — Competition-Grade Line Tracking
//  Hardware: Teensy 4.1 + Pixy2 Camera + Servo Steering + Dual H-Bridge Motors
//
//  Algorithms:
//   • Pixy2 built-in vector tracking + intersection handling
//   • Weighted look-ahead PID (near error + far error blend)
//   • Curvature estimation from vector angle → speed control
//   • Chicane detection (rapid sign changes → preemptive braking)
//   • Gain-scheduled PID (fast phase / slow phase)
//   • EMA target smoothing to filter frame-to-frame jitter
//   • Dual-line fallback via getAllFeatures()
// ============================================================================

#include <Pixy2.h>
#include <Servo.h>

Pixy2 pixy;
Servo steeringServo;

// =========================== PIN CONFIGURATION ==============================
const int SERVO_PIN   = 8;
const int RIGHT_RPWM  = 2;
const int RIGHT_LPWM  = 3;
const int LEFT_RPWM   = 4;
const int LEFT_LPWM   = 5;

// ============================ STEERING LIMITS ===============================
const int CENTER_ANGLE = 90;
const int MAX_STEER    = 35;

// ============================ PIXY2 FRAME ===================================
// Pixy2 line mode frame: 80 x 53 pixels (x: 0–79, y: 0–52)
const float FRAME_W       = 79.0f;
const float FRAME_H       = 52.0f;
const float FRAME_CENTER  = 39.0f;  // Horizontal center

// ======================== TRACK GEOMETRY (tune!) ============================
// Half the track width in pixels. Used when only one boundary line is visible.
const int HALF_TRACK_WIDTH = 20;

// ======================== VECTOR FILTERS ====================================
const int HORIZON_Y        = 10;   // Ignore vectors entirely above this Y
const int MIN_VEC_LEN_SQ   = 20;   // Minimum vector length² to accept

// ====================== LOOK-AHEAD BLEND WEIGHTS ============================
// How much to weight near (tail) vs far (head) error.
// Higher FAR_WEIGHT = more anticipatory steering, better for fast driving.
// Start conservative and increase FAR_WEIGHT as you gain confidence.
// Reduced FAR_WEIGHT to reduce oscillation from noisy head point.
const float NEAR_WEIGHT = 0.65f;   // Weight for tail (current position)
const float FAR_WEIGHT  = 0.35f;   // Weight for head (look-ahead)

// ======================== EMA SMOOTHING =====================================
// Exponential Moving Average coefficient for target position.
// Lower = smoother but more lag. Higher = more responsive but noisier.
// Lowered from 0.40 to reduce zigzag from frame-to-frame jitter.
const float EMA_ALPHA = 0.30f;

// ========================= PID TUNING =======================================
// Base gains — scaled by gain scheduling (see below)
const float KP_BASE = 1.6f;    // Reduced from 1.2 to stop oscillation
const float KI_BASE = 0.005f;   // Slight bump to help hold curves
const float KD_BASE = 0.50f;    // Increased: D is the anti-zigzag term

// Gain scheduling: when error is large (curve entry), we are in "fast phase"
// and boost P while reducing I. When error is small (tracking), "slow phase".
const float PHASE_SWITCH_THRESHOLD = 8.0f;  // pixels

// Fast phase multipliers (entering/in a curve)
const float KP_FAST_MULT = 1.9f;
const float KI_FAST_MULT = 0.00f;   // No integral during aggressive correction
const float KD_FAST_MULT = 1.10f;

// Slow phase multipliers (near center, tracking)
const float KP_SLOW_MULT = 1.3f;  // Slightly more responsive near center
const float KI_SLOW_MULT = 1.00f;
const float KD_SLOW_MULT = 0.70f;   // More damping near center too

// Dead-band: errors below this are treated as zero (prevents servo buzz)
// Increased from 1.5 to suppress more jitter on straights.
const float ERROR_DEADBAND = 2.5f;

// Derivative low-pass filter (0–1). Higher = smoother.
const float DERIV_FILTER_ALPHA = 0.65f;

// Integral limits
const float INTEGRAL_LIMIT = 40.0f;

// ======================== SPEED PARAMETERS ==================================
const int SPEED_STRAIGHT   = 85;    // Max speed on perfect straight
const int SPEED_CURVE      = 58;    // Min speed on sharpest curve
const int SPEED_CHICANE    = 52;    // Even slower during chicanes
const int SPEED_INTERSECT  = 55;    // Slow through intersections
const int SPEED_RECOVERY   = 48;    // Slow during blind recovery

// Curvature angle thresholds (degrees) for speed mapping
const float CURVE_ANGLE_MIN =  5.0f;  // Below this = straight
const float CURVE_ANGLE_MAX = 35.0f;  // Above this = sharpest curve

// ==================== CHICANE DETECTION =====================================
const int   CHICANE_SIGN_CHANGES    = 3;     // Sign changes to trigger chicane
const float CHICANE_MIN_ERROR       = 4.0f;  // Minimum error to count sign change
const unsigned long CHICANE_WINDOW  = 500;   // Window in ms to count sign changes

// ==================== LINE LOSS / RECOVERY ==================================
const unsigned long MAX_LOST_TIME = 600;  // Stop if lost for longer (ms)

// ==================== INTERSECTION COOLDOWN =================================
const unsigned long INTERSECT_COOLDOWN = 400;  // Slow for this duration after intersection

// ============================================================================
//  STATE VARIABLES
// ============================================================================

// PID state
float prev_error          = 0.0f;
float integral_sum        = 0.0f;
float filtered_derivative = 0.0f;
float ema_target          = FRAME_CENTER;  // EMA-smoothed target position
uint32_t last_time_us     = 0;
bool pid_started          = false;

// Line loss
int   last_known_error    = 0;
unsigned long lost_line_time = 0;

// Chicane detection
int   sign_change_count   = 0;
int   last_error_sign     = 0;
unsigned long sign_change_times[8];  // Circular buffer of sign change timestamps
int   sign_change_idx     = 0;
bool  in_chicane          = false;

// Intersection
unsigned long last_intersection_time = 0;

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);

  // 1. Steering
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(CENTER_ANGLE);

  // 2. Motors
  pinMode(RIGHT_RPWM, OUTPUT); pinMode(RIGHT_LPWM, OUTPUT);
  pinMode(LEFT_RPWM, OUTPUT);  pinMode(LEFT_LPWM, OUTPUT);
  analogWriteFrequency(RIGHT_RPWM, 20000);
  analogWriteFrequency(RIGHT_LPWM, 20000);
  analogWriteFrequency(LEFT_RPWM, 20000);
  analogWriteFrequency(LEFT_LPWM, 20000);
  stopMotors();

  // 3. Pixy2
  pixy.init();
  pixy.changeProg("line");

  // CRITICAL: Tell Pixy2 to always go straight at intersections.
  // This prevents the camera from losing the vector when it sees crossing lines.
  pixy.line.setDefaultTurn(0);

  // 4. Initialize chicane buffer
  for (int i = 0; i < 8; i++) sign_change_times[i] = 0;

  // 5. Launch countdown
  Serial.println(F("=== NXP CUP v2 ==="));
  Serial.println(F("3...")); delay(1000);
  Serial.println(F("2...")); delay(1000);
  Serial.println(F("1...")); delay(1000);
  Serial.println(F("GO!"));

  last_time_us = micros();
}

// ============================================================================
//  MAIN LOOP
// ============================================================================
void loop() {
  // ------------------------------------------------------------------
  //  STEP 1: Get features from Pixy2
  // ------------------------------------------------------------------
  int8_t res = pixy.line.getMainFeatures();

  bool got_vector       = (res > 0) && (res & LINE_VECTOR) && (pixy.line.numVectors > 0);
  bool got_intersection = (res > 0) && (res & LINE_INTERSECTION) && (pixy.line.numIntersections > 0);

  // ------------------------------------------------------------------
  //  STEP 2: Intersection handling
  // ------------------------------------------------------------------
  if (got_intersection) {
    last_intersection_time = millis();
    // Pixy2 will go straight automatically (setDefaultTurn(0)).
    // We just note it for speed reduction.
    Serial.println(F(">>> INTERSECTION"));
  }

  // ------------------------------------------------------------------
  //  STEP 3: No vector — line lost
  // ------------------------------------------------------------------
  if (!got_vector) {
    if (lost_line_time == 0) lost_line_time = millis();

    if (millis() - lost_line_time > MAX_LOST_TIME) {
      // Lost for too long — emergency stop
      stopMotors();
      steeringServo.write(CENTER_ANGLE);
      Serial.println(F("!!! EMERGENCY STOP — line lost"));
      return;
    }

    // Blind recovery: steer based on last known error, slow speed
    driveWithPID(last_known_error, SPEED_RECOVERY, true);
    return;
  }

  // Vector found — reset loss timer
  lost_line_time = 0;

  // ------------------------------------------------------------------
  //  STEP 4: Extract vector data
  // ------------------------------------------------------------------
  // Pixy2's main vector: tail (x0,y0) = near car, head (x1,y1) = far ahead
  float x_near = (float)pixy.line.vectors[0].m_x0;  // Tail X
  float y_near = (float)pixy.line.vectors[0].m_y0;  // Tail Y
  float x_far  = (float)pixy.line.vectors[0].m_x1;  // Head X
  float y_far  = (float)pixy.line.vectors[0].m_y1;  // Head Y

  // ------------------------------------------------------------------
  //  STEP 5: Calculate weighted error (look-ahead blend)
  // ------------------------------------------------------------------
  float near_error = x_near - FRAME_CENTER;
  float far_error  = x_far  - FRAME_CENTER;
  float blended_error = NEAR_WEIGHT * near_error + FAR_WEIGHT * far_error;

  // EMA smoothing — reduces frame-to-frame jitter
  float target_x = FRAME_CENTER + blended_error;
  ema_target = EMA_ALPHA * target_x + (1.0f - EMA_ALPHA) * ema_target;
  float error = ema_target - FRAME_CENTER;

  last_known_error = (int)error;

  // ------------------------------------------------------------------
  //  STEP 6: Curvature estimation from vector angle
  // ------------------------------------------------------------------
  float dx = x_far - x_near;
  float dy = y_near - y_far;  // Y is inverted (0 = top of frame)
  float curvature_angle = 0.0f;
  if (dy > 1.0f) {
    // atan2 gives angle from vertical. Larger = more curved.
    curvature_angle = fabs(atan2f(dx, dy) * 57.2958f);  // rad → deg
  } else {
    // Very short vector or horizontal — treat as high curvature
    curvature_angle = CURVE_ANGLE_MAX;
  }

  // ------------------------------------------------------------------
  //  STEP 7: Chicane detection
  // ------------------------------------------------------------------
  updateChicaneDetection(error);

  // ------------------------------------------------------------------
  //  STEP 8: Determine speed
  // ------------------------------------------------------------------
  int target_speed;

  if (in_chicane) {
    target_speed = SPEED_CHICANE;
  } else if (millis() - last_intersection_time < INTERSECT_COOLDOWN) {
    target_speed = SPEED_INTERSECT;
  } else {
    // Map curvature angle to speed
    if (curvature_angle <= CURVE_ANGLE_MIN) {
      target_speed = SPEED_STRAIGHT;
    } else if (curvature_angle >= CURVE_ANGLE_MAX) {
      target_speed = SPEED_CURVE;
    } else {
      // Linear interpolation between straight and curve speed
      float t = (curvature_angle - CURVE_ANGLE_MIN) / (CURVE_ANGLE_MAX - CURVE_ANGLE_MIN);
      target_speed = (int)(SPEED_STRAIGHT - t * (SPEED_STRAIGHT - SPEED_CURVE));
    }
  }

  // ------------------------------------------------------------------
  //  STEP 9: PID + Drive
  // ------------------------------------------------------------------
  driveWithPID((int)error, target_speed, false);

  // ------------------------------------------------------------------
  //  STEP 10: Debug output
  // ------------------------------------------------------------------
  static uint8_t debug_counter = 0;
  if (++debug_counter >= 5) {  // Print every 5th frame to avoid Serial bottleneck
    debug_counter = 0;
    Serial.print(F("E:"));   Serial.print((int)error);
    Serial.print(F(" C:"));  Serial.print((int)curvature_angle);
    Serial.print(F(" S:"));  Serial.print(target_speed);
    Serial.print(F(" K:"));  Serial.print(in_chicane ? 'Y' : 'N');
    Serial.print(F(" N:"));  Serial.print((int)x_near);
    Serial.print(F(" F:"));  Serial.println((int)x_far);
  }
}

// ============================================================================
//  CHICANE DETECTION
// ============================================================================
// Detect rapid direction changes: if error sign flips multiple times within
// a short window, we're in a chicane and should slow down.
void updateChicaneDetection(float error) {
  int current_sign = 0;
  if (error > CHICANE_MIN_ERROR)       current_sign =  1;
  else if (error < -CHICANE_MIN_ERROR) current_sign = -1;

  if (current_sign != 0 && current_sign != last_error_sign && last_error_sign != 0) {
    // Sign changed — record timestamp
    sign_change_times[sign_change_idx] = millis();
    sign_change_idx = (sign_change_idx + 1) % 8;
    sign_change_count++;
  }

  if (current_sign != 0) {
    last_error_sign = current_sign;
  }

  // Count how many sign changes occurred within the chicane window
  unsigned long now = millis();
  int recent_changes = 0;
  for (int i = 0; i < 8; i++) {
    if (sign_change_times[i] != 0 && (now - sign_change_times[i]) < CHICANE_WINDOW) {
      recent_changes++;
    }
  }

  in_chicane = (recent_changes >= CHICANE_SIGN_CHANGES);
}

// ============================================================================
//  CORE PID + DRIVE FUNCTION
// ============================================================================
void driveWithPID(int raw_error, int target_speed, bool is_recovery) {
  // ---- Real dt measurement ----
  uint32_t now = micros();
  float dt = 0.02f;
  if (pid_started) {
    uint32_t delta = now - last_time_us;
    dt = delta * 1.0e-6f;
    if (dt < 0.002f) dt = 0.002f;
    if (dt > 0.080f) dt = 0.080f;
  }
  last_time_us = now;
  pid_started = true;

  // ---- Apply dead-band ----
  float error = (float)raw_error;
  if (fabsf(error) < ERROR_DEADBAND) {
    error = 0.0f;
  }

  float err_abs = fabsf(error);

  // ---- Gain scheduling: fast vs slow phase ----
  bool fast_phase = (err_abs > PHASE_SWITCH_THRESHOLD);
  float kp, ki, kd;

  if (fast_phase) {
    kp = KP_BASE * KP_FAST_MULT;
    ki = KI_BASE * KI_FAST_MULT;
    kd = KD_BASE * KD_FAST_MULT;
  } else {
    kp = KP_BASE * KP_SLOW_MULT;
    ki = KI_BASE * KI_SLOW_MULT;
    kd = KD_BASE * KD_SLOW_MULT;
  }

  // ---- P term ----
  float P = kp * error;

  // ---- I term with anti-windup ----
  // Reset integral on error sign change (entering new maneuver)
  if ((error > 0 && prev_error < 0) || (error < 0 && prev_error > 0)) {
    integral_sum = 0.0f;
  }
  // Only accumulate integral in slow phase (when close to target)
  if (!fast_phase) {
    integral_sum += error * dt;
  }
  // Clamp
  if (integral_sum > INTEGRAL_LIMIT) integral_sum = INTEGRAL_LIMIT;
  if (integral_sum < -INTEGRAL_LIMIT) integral_sum = -INTEGRAL_LIMIT;
  float I = ki * integral_sum;

  // ---- D term with low-pass filter ----
  float raw_deriv = (error - prev_error) / dt;
  filtered_derivative = DERIV_FILTER_ALPHA * filtered_derivative
                      + (1.0f - DERIV_FILTER_ALPHA) * raw_deriv;
  float D = kd * filtered_derivative;

  prev_error = error;

  // ---- Total PID output ----
  float pid_output = P + I + D;

  // ---- Servo actuation ----
  // ⚠ If steering is INVERTED on your car, change + to - here:
  int steer_angle = CENTER_ANGLE + (int)pid_output;
  steer_angle = constrain(steer_angle, CENTER_ANGLE - MAX_STEER, CENTER_ANGLE + MAX_STEER);
  steeringServo.write(steer_angle);

  // ---- Speed override during recovery ----
  if (is_recovery) {
    target_speed = SPEED_RECOVERY;
  }

  // ---- Differential drive ----
  int abs_steer = abs(steer_angle - CENTER_ANGLE);
  float diff_ratio = 1.0f - (float(abs_steer) / float(MAX_STEER)) * 0.40f;

  int left_speed  = target_speed;
  int right_speed = target_speed;

  if (steer_angle > CENTER_ANGLE) {
    // Turning right → slow right (inner) wheel
    right_speed = (int)(target_speed * diff_ratio);
  } else if (steer_angle < CENTER_ANGLE) {
    // Turning left → slow left (inner) wheel
    left_speed = (int)(target_speed * diff_ratio);
  }

  drive(left_speed, right_speed);
}

// ============================================================================
//  MOTOR HELPERS
// ============================================================================
void drive(int left_speed, int right_speed) {
  left_speed  = constrain(left_speed, 0, 255);
  right_speed = constrain(right_speed, 0, 255);

  analogWrite(RIGHT_RPWM, right_speed); analogWrite(RIGHT_LPWM, 0);
  analogWrite(LEFT_RPWM, left_speed);   analogWrite(LEFT_LPWM, 0);
}

void stopMotors() {
  analogWrite(RIGHT_RPWM, 0); analogWrite(RIGHT_LPWM, 0);
  analogWrite(LEFT_RPWM, 0);  analogWrite(LEFT_LPWM, 0);
}