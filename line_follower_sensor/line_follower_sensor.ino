// ===============================
// 5-SENSOR WHITE LINE FOLLOWER
// ESP32-S3 DevKit + DRV8835 + Sharp GP2Y0A41SK0F
// Stops if distance <= 5cm, moves again if > 5cm
// ===============================

// -------- Line sensor pins (LEFT → RIGHT) --------
// Must be ADC-capable pins on ESP32-S3 (GPIO 1..20)
int sensorPins[5] = {4, 5, 6, 7, 15};
int sensorValues[5];

// -------- Motor driver pins --------
int leftPWM    = 37;
int leftPhase  = 38;
int rightPWM   = 39;
int rightPhase = 20;

// -------- Sharp distance sensor --------
const int sharpPin = 16;   // ADC pin (GPIO 1..20 on ESP32-S3) - change if needed
const int STOP_CM  = 5;   // stop when distance <= 5cm

// -------- Tuning values --------
int threshold = 250;  // white < threshold
int baseSpeed = 190;  // forward speed
int Kp = 80;          // steering strength

// -------- PWM settings --------
const uint32_t PWM_FREQ_HZ  = 20000;
const uint8_t  PWM_RES_BITS = 8;   // duty 0..255

// ===============================
void setup() {
  Serial.begin(9600);
  delay(200);

  analogReadResolution(12); // 0..4095

  pinMode(leftPhase, OUTPUT);
  pinMode(rightPhase, OUTPUT);

  // ESP32 core (new LEDC API): attach PWM to pins
  ledcAttach(leftPWM,  PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttach(rightPWM, PWM_FREQ_HZ, PWM_RES_BITS);

  pinMode(sharpPin, INPUT);
}

// ===============================
void loop() {
  readSensors();

  int d = readSharpDistanceCM();
  if (d <= STOP_CM) {
    setMotors(0, 0);
    return; // stays stopped until distance becomes > 5cm
  }

  followLine();
}

// ===============================
// Read all 5 line sensors
void readSensors() {
  for (int i = 0; i < 5; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }
}

// ===============================
// Sharp sensor helpers
int readSharpRawAvg() {
  const int N = 10;
  long sum = 0;
  for (int i = 0; i < N; i++) {
    sum += analogRead(sharpPin);
    delayMicroseconds(200);
  }
  return (int)(sum / N);
}

int readSharpDistanceCM() {
  int raw = readSharpRawAvg();
  float v = (raw / 4095.0f) * 3.3f;

  // Approx inverse model for GP2Y0A41 (best ~4–30cm)
  float cm = 30.0f;
  if (v > 0.12f) cm = 12.08f / (v - 0.11f);

  if (cm < 4.0f)  cm = 4.0f;
  if (cm > 30.0f) cm = 30.0f;

  return (int)(cm + 0.5f);
}

// ===============================
// Calculate line position error
int getLineError() {
  int weights[5] = {-2, -1, 0, 1, 2};
  int sum = 0, count = 0;

  for (int i = 0; i < 5; i++) {
    if (sensorValues[i] < threshold) { // white detected
      sum += weights[i];
      count++;
    }
  }

  if (count == 0) return 100; // line lost
  return sum / count;
}

// ===============================
// Control motors (new LEDC API)
void setMotors(int leftSpeed, int rightSpeed) {
  digitalWrite(leftPhase,  leftSpeed >= 0 ? LOW  : HIGH);
  digitalWrite(rightPhase, rightSpeed >= 0 ? HIGH : LOW);

  int l = constrain(abs(leftSpeed),  0, 255);
  int r = constrain(abs(rightSpeed), 0, 255);

  ledcWrite(leftPWM,  (uint32_t)l);
  ledcWrite(rightPWM, (uint32_t)r);
}

// ===============================
// Line following logic
void followLine() {
  int error = getLineError();

  if (error == 100) {
    // safer to stop if line lost
    setMotors(190, 190);
    return;
  }

  int correction = Kp * error;

  int leftSpeed  = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  leftSpeed  = constrain(leftSpeed,  0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  setMotors(leftSpeed, rightSpeed);
}
