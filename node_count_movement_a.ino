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
// ===============================
const char* ssid = "iPhone";
const char* password = "12345678";

const char* server = "3.250.38.184";
const int port = 8000;
const char* teamID = "dtbv6902";

WiFiClient client;

// ===============================
// ===== Robot State =====
String fullRoute = "";    // e.g., "0,1,2,3"
int currentPosition = 0;
static bool nodeTriggered = false; // avoid multiple triggers per node

// ===============================
// ===== Setup =====
void setup() {
  Serial.begin(9600);
  delay(1000);

  Serial.println("=== ESP32 Line Follower Robot Started ===");

  // Motor pins
  pinMode(leftPWM, OUTPUT);
  pinMode(leftPhase, OUTPUT);
  pinMode(rightPWM, OUTPUT);

  // Connect to Wi-Fi
  connectToWiFi();

  // Fetch full route
  fullRoute = getFullRoute();
  if (fullRoute != "") {
    Serial.println("Route fetched: " + fullRoute);
  } else {
    Serial.println("Failed to fetch route.");
  }

  // Notify arrival at starting position
  notifyArrival(currentPosition);
}

// ===============================
// ===== Main Loop =====
void loop() {
  readSensors();
  followLine();

  // Check for node
  if (isNode()) {
    if (!nodeTriggered) {
      nodeTriggered = true;
      Serial.println("Node reached!");
      notifyArrival(currentPosition);
      currentPosition++;
      setMotors(0, 0); // pause briefly at node
      delay(500);
    }
  } else {
    nodeTriggered = false; // reset for next node
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
// ===== Node Detection =====
bool isNode() {
  for (int i = 0; i < 5; i++) {
    if (sensorValues[i] < threshold) { // not high → not a node
      return false;
    }
  }
  return true; // all 5 sensors HIGH → node detected
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
  String response = "";
  unsigned long timeout = millis() + 3000;
  while (client.connected() && millis() < timeout) {
    while (client.available()) {
      char c = client.read();
      response += c;
    }
  }
  return response;
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

bool notifyArrival(int position) {
  if (!connectToServer()) return false;

  String postBody = "position=" + String(position);

  client.println("POST /api/arrived/" + String(teamID) + " HTTP/1.1");
  client.println("Host: " + String(server));
  client.println("Content-Type: application/x-www-form-urlencoded");
  client.print("Content-Length: ");
  client.println(postBody.length());
  client.println();
  client.println(postBody);

  String response = readResponse();
  int statusCode = getStatusCode(response);
  client.stop();

  Serial.print("Arrival POST status code: ");
  Serial.println(statusCode);
  return statusCode == 200;
}

String getFullRoute() {
  if (!connectToServer()) return "";

  client.println("GET /api/getRoute/" + String(teamID) + " HTTP/1.1");
  client.println("Host: " + String(server));
  client.println("Connection: close");
  client.println();

  String response = readResponse();
  int statusCode = getStatusCode(response);
  client.stop();

  if (statusCode == 200) {
    return getResponseBody(response);
  }
  return "";
}
