// ===== MOTOR PINS =====
const uint8_t ENA = 5;
const uint8_t IN1 = 8;
const uint8_t IN2 = 9;
const uint8_t IN3 = 7;
const uint8_t IN4 = 6;
const uint8_t ENB = 10;

// ===== SENSOR PINS (LEFT → RIGHT) =====
const uint8_t S0 = A4;   // FAR LEFT
const uint8_t S1 = A3;   // LEFT
const uint8_t S2 = A2;   // CENTER
const uint8_t S3 = A1;   // RIGHT
const uint8_t S4 = A0;   // FAR RIGHT

// ===== SPEED =====
int baseSpeed     = 65;
int rotateMedium  = 70;
int rotateHard    = 95;
int rotate90      = 130;
int recoverySpeed = 90;

// ===== PID =====
float Kp = 26.0;
float Kd = 45.0;
int maxCorrection = 70;

// ===== CENTER OFFSET =====
float centerOffset = -0.10;

// ===== MEMORY =====
float lastError = 0;
int lastSeenSide = 0;   // -1 = left, +1 = right

// ===== RECOVERY TIMER =====
unsigned long lostStartTime = 0;
const unsigned long RECOVERY_DELAY_MS = 80;
bool waitingRecovery = false;

// ===== TURN ESCALATION =====
bool turning = false;
bool hard90 = false;
int turnDir = 0;   // -1 = left, +1 = right
unsigned long turnStartTime = 0;
const unsigned long TURN_90_TIMEOUT_MS = 180;

// =================================================
// MOTOR CONTROL (SIGNED SPEEDS)
// =================================================
void setMotor(int left, int right) {

  // LEFT MOTOR
  if (left >= 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, left);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, -left);
  }

  // RIGHT MOTOR
  if (right >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, right);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -right);
  }
}

void setup() {
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(S0, INPUT); pinMode(S1, INPUT);
  pinMode(S2, INPUT); pinMode(S3, INPUT);
  pinMode(S4, INPUT);
}

void loop() {

  // BLACK = LOW
  int s0 = digitalRead(S0) == LOW;
  int s1 = digitalRead(S1) == LOW;
  int s2 = digitalRead(S2) == LOW;
  int s3 = digitalRead(S3) == LOW;
  int s4 = digitalRead(S4) == LOW;

  int sum = s0 + s1 + s2 + s3 + s4;

  // ===== THIN LINE CENTER BRIDGE =====
  // S2 lost, but line still between S1 and S3
  bool thinCenterBridge = (!s2 && s1 && s3);

  // ===== TRACK LAST SEEN SIDE =====
  if (s0 || s1) lastSeenSide = -1;
  else if (s3 || s4) lastSeenSide = 1;

  // =================================================
  // LINE LOST
  // =================================================
  if (sum == 0) {
    if (!waitingRecovery) {
      lostStartTime = millis();
      waitingRecovery = true;
      setMotor(0, 0);
      return;
    }

    if (millis() - lostStartTime < RECOVERY_DELAY_MS) {
      setMotor(0, 0);
      return;
    }

    if (lastSeenSide < 0)
      setMotor(-recoverySpeed, recoverySpeed);
    else
      setMotor(recoverySpeed, -recoverySpeed);

    return;
  }

  waitingRecovery = false;

  // =================================================
  // THIN LINE → GO STRAIGHT
  // =================================================
  if (thinCenterBridge) {
    setMotor(baseSpeed, baseSpeed);
    lastError = 0;   // prevent PID jump
    return;
  }

  // =================================================
  // LOCKED TURNING / 90 DEG MODE
  // =================================================
  if (turning) {

    if (s2) {
      turning = false;
      hard90 = false;
    } else {
      if (!hard90 && millis() - turnStartTime > TURN_90_TIMEOUT_MS) {
        hard90 = true;
      }

      int speed = hard90 ? rotate90 : rotateHard;

      if (turnDir < 0)
        setMotor(-speed, speed);
      else
        setMotor(speed, -speed);

      return;
    }
  }

  // =================================================
  // START TURN CONDITIONS (ONLY IF NOT THIN LINE)
  // =================================================
  if (s0 && !thinCenterBridge) {
    turning = true;
    hard90 = false;
    turnDir = -1;
    turnStartTime = millis();
    return;
  }

  if (s4 && !thinCenterBridge) {
    turning = true;
    hard90 = false;
    turnDir = 1;
    turnStartTime = millis();
    return;
  }

  // =================================================
  // MEDIUM TURN (INNER SENSORS)
  // =================================================
  if (s1 && !s2 && !s3) {
    setMotor(-rotateMedium, rotateMedium);
    return;
  }

  if (s3 && !s2 && !s1) {
    setMotor(rotateMedium, -rotateMedium);
    return;
  }

  // =================================================
  // NORMAL PID
  // =================================================
  int weights[5] = {-2, -1, 0, 1, 2};

  float position =
      (s0 * weights[0] +
       s1 * weights[1] +
       s2 * weights[2] +
       s3 * weights[3] +
       s4 * weights[4]) / (float)sum;

  float error = position + centerOffset;

  float derivative = error - lastError;
  float correction = (Kp * error) + (Kd * derivative);
  correction = constrain(correction, -maxCorrection, maxCorrection);

  lastError = error;

  setMotor(baseSpeed - correction, baseSpeed + correction);
}
