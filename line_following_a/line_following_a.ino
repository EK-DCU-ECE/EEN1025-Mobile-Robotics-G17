// ===============================
// 5-SENSOR WHITE LINE FOLLOWER
// ESP32 + DRV8835
// ===============================

// -------- Line sensor pins (LEFT → RIGHT) --------
int sensorPins[5] = {4, 5, 6, 7, 15};
int sensorValues[5];

// -------- Motor driver pins --------
int leftPWM    = 37;
int leftPhase  = 38;
int rightPWM   = 39;
int rightPhase = 40;

// -------- Tuning values --------
int threshold = 250;     // white < 250, black > 3800
int baseSpeed = 190;     // forward speed
int Kp = 80;             // steering strength

// ===============================
void setup() {
  Serial.begin(9600);

  pinMode(leftPWM, OUTPUT);
  pinMode(leftPhase, OUTPUT);
  pinMode(rightPWM, OUTPUT);
  pinMode(rightPhase, OUTPUT);
}

// ===============================
void loop() {
  readSensors();
  followLine();
}

// ===============================
// Read all 5 sensors
void readSensors() {
  for (int i = 0; i < 5; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }
}

// ===============================
// Calculate line position error
int getLineError() {
  int weights[5] = {-2, -1, 0, 1, 2};
  int sum = 0;
  int count = 0;

  for (int i = 0; i < 5; i++) {
    if (sensorValues[i] < threshold) { // white detected
      sum += weights[i];
      count++;
    }
  }

  if (count == 0) {
    return 100; // line lost
  }

  return sum / count;
}

// ===============================
// Control motors
void setMotors(int leftSpeed, int rightSpeed) {
  digitalWrite(leftPhase, leftSpeed >= 0 ? LOW : HIGH);
  digitalWrite(rightPhase, rightSpeed >= 0 ? HIGH : LOW);

  analogWrite(leftPWM, abs(leftSpeed));
  analogWrite(rightPWM, abs(rightSpeed));
}

// ===============================
// Line following logic
void followLine() {
  int error = getLineError();

  if (error == 100) {
    // Line lost → stop
    setMotors(190, 190);
    return;
  }

  int correction = Kp * error;

  int leftSpeed  = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  setMotors(leftSpeed, rightSpeed);
}







