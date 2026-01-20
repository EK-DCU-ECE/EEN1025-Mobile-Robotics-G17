#include <WiFi.h>

// ===============================
// ===== 5-SENSOR WHITE LINE FOLLOWER =====
// ===============================

// -------- Line sensor pins (LEFT → RIGHT) --------
int sensorPins[5] = {4, 5, 6, 7, 15};
int sensorValues[5];

// -------- Motor driver pins --------
int leftPWM    = 37;
int leftPhase  = 38;
int rightPWM   = 39;
int rightPhase = 20;

// -------- Tuning values --------
int threshold = 250;     // white < 250, black > 3800
int baseSpeed = 190;     // forward speed
int Kp = 80;             // steering strength

// ===============================
// ===== Wi-Fi & Server =====
const char* ssid = "iPhone";
const char* password = "12345678";

const char* server = "3.250.38.184";
const int port = 8000;
const char* teamID = "dtbv6902";

WiFiClient client;

// ===============================
// ===== Robot State =====
int currentPosition = 0;          // position to notify server
static unsigned long lastUpdate = 0;  // for 5-second updates
bool finished = false;             // final node reached

// ===============================
// ===== Buffer for HTTP =====
#define BUFSIZE 512

// ===============================
// ===== Setup =====
void setup() {
  Serial.begin(9600);
  delay(1000);

  Serial.println("=== ESP32 Line Follower Robot Demo Started ===");

  // Motor pins
  pinMode(leftPWM, OUTPUT);
  pinMode(leftPhase, OUTPUT);
  pinMode(rightPWM, OUTPUT);

  // Connect to Wi-Fi
  connectToWiFi();

  // Notify arrival at starting position 0
  int dest = notifyArrival(currentPosition);
  if (dest == -1) {
    Serial.println("Failed to notify server at position 0");
  } else if (dest == -2) {
    Serial.println("Already at final destination!");
    finished = true;
    setMotors(0, 0);
  } else {
    Serial.println("Server next destination: " + String(dest));
  }

  lastUpdate = millis();
}

// ===============================
// ===== Main Loop =====
void loop() {
  // Stop robot if finished
  if (finished) {
    setMotors(0, 0);
    return;
  }

  // 1. Follow line continuously
  readSensors();
  followLine();

  // 2. Send next position every 5 seconds
  if (millis() - lastUpdate >= 5000) {
    lastUpdate = millis();

    int dest = notifyArrival(currentPosition);
    if (dest == -1) {
      Serial.println("Server error at position " + String(currentPosition));
    } else if (dest == -2) {
      Serial.println("Reached final destination!");
      finished = true;
      setMotors(0, 0); // stop
    } else {
      Serial.println("Server next destination: " + String(dest));
      currentPosition++;
    }
  }
}

// ===============================
// ===== Line Following Functions =====
void readSensors() {
  for (int i = 0; i < 5; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }
}

int getLineError() {
  int weights[5] = {-2, -1, 0, 1, 2};
  int sum = 0;
  int count = 0;

  for (int i = 0; i < 5; i++) {
    if (sensorValues[i] < threshold) { // black line detected
      sum += weights[i];
      count++;
    }
  }

  if (count == 0) return 100; // line lost
  return sum / count;
}

void setMotors(int leftSpeed, int rightSpeed) {
  digitalWrite(leftPhase, leftSpeed >= 0 ? LOW : HIGH);
  digitalWrite(rightPhase, rightSpeed >= 0 ? HIGH : LOW);

  analogWrite(leftPWM, abs(leftSpeed));
  analogWrite(rightPWM, abs(rightSpeed));
}

void followLine() {
  int error = getLineError();

  if (error == 100) {
    // Line lost → move forward cautiously
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

// ===============================
// ===== Wi-Fi / Server Functions =====
void connectToWiFi() {
  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    Serial.print(".");
    delay(500);
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to Wi-Fi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to Wi-Fi.");
  }
}

bool connectToServer() {
  Serial.print("Connecting to server: ");
  Serial.print(server);
  Serial.print(":");
  Serial.println(port);

  if (!client.connect(server, port)) {
    Serial.println("Error connecting to server");
    return false;
  }
  return true;
}

String readResponse() {
  char buffer[BUFSIZE];
  memset(buffer, 0, BUFSIZE);
  client.readBytes(buffer, BUFSIZE);
  return String(buffer);
}

int getStatusCode(String &response) {
  if (response.length() < 12) return 0;
  return response.substring(9, 12).toInt();
}

String getResponseBody(String &response) {
  int split = response.indexOf("\r\n\r\n");
  if (split == -1) return "";
  String body = response.substring(split + 4);
  body.trim();
  return body;
}

// Returns:
// -1 → server error
// -2 → final destination reached
// >=0 → next destination
int notifyArrival(int position) {
  if (!connectToServer()) return -1;

  String postBody = "position=" + String(position);

  client.println("POST /api/arrived/" + String(teamID) + " HTTP/1.1");
  client.println("Host: " + String(server));
  client.println("Content-Type: application/x-www-form-urlencoded");
  client.print("Content-Length: ");
  client.println(postBody.length());
  client.println(); // end headers
  client.println(postBody); // send body

  String response = readResponse();
  int statusCode = getStatusCode(response);
  client.stop();

  if (statusCode != 200) return -1;

  String body = getResponseBody(response);

  if (body.equals("Finished")) return -2;

  return body.toInt();
}

