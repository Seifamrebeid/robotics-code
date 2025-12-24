// ===== MOTOR PINS =====
const uint8_t ENA = 5;
const uint8_t IN1 = 8;
const uint8_t IN2 = 9;

const uint8_t IN3 = 7;
const uint8_t IN4 = 6;
const uint8_t ENB = 10;

// ===== ENCODER =====
const uint8_t ENCODER_PIN = 2;
volatile long encoderCount = 0;

// ===== PARAMETERS =====
const float WHEEL_DIAMETER = 0.065;   // meters
const int   PULSES_PER_REV = 20;
const float TARGET_DISTANCE = 1.5;    // meters

int leftSpeed  = 80;   // slower
int rightSpeed = 70;

// ===== TARGET =====
long targetPulses;

// ===== ENCODER ISR =====
void encoderISR() {
  encoderCount++;
}

// ===== MOTOR CONTROL =====
void moveForward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(ENCODER_PIN, INPUT);   // IMPORTANT FIX
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, CHANGE);

  float circumference = WHEEL_DIAMETER * 3.1416;
  targetPulses = (TARGET_DISTANCE / circumference) * PULSES_PER_REV;

  encoderCount = 0;
}

void loop() {
  moveForward();

  if (encoderCount >= targetPulses) {
    stopMotors();
    while (1);
  }
}
