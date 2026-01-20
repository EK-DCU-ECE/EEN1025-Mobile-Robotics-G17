#include <WiFi.h>

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
int rightPhase = 20;

// -------- Tuning values --------
int threshold = 250;     // white < 250, black > 3800
int baseSpeed = 190;     // forward speed
int Kp = 80;             // steering strength

// ===============================
// Wi-Fi & Server
const char* ssid = "iPhone";
const char* password = "12345678";

const char* server = "3.250.38.184";
const int port = 8000;
const char* teamID = "dtbv6902";

WiFiClient client;
#define BUFSIZE 512

// ===============================
// Robot state
int currentPosition = 0;
bool finished = false;
bool nodeTriggered = false; // prevents multiple triggers on same node

// ===============================
// Setup
void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("=== ESP32 Node-Based White Line Follower Started ===");

  pinMode(leftPWM, OUTPUT);
  pinMode(leftPhase, OUTPUT);
  pinMode(rightPWM, OUTPUT);
  pinMode(rightPhase, OUTPUT);

  connectToWiFi();
}

// ===============================
// Main loop
void loop() {
  if (finished) {
    setMotors(0,0); // stop at final node
    return;
  }

  readSensors();
  followLine();

  // Node detection
  if (isNode()) {
    if (!nodeTriggered) {
      nodeTriggered = true; // prevent multiple triggers
      Serial.println("Node reached at position " + String(currentPosition));
      setMotors(0,0); // pause at node
      delay(500);     // optional pause

      int dest = notifyArrival(currentPosition);

      if (dest == -1) {
        Serial.println("Server error at position " + String(currentPosition));
      } else if (dest == -2) {
        Serial.println("Reached final destination!");
        finished = true;
        setMotors(0,0);
      } else {
        Serial.println("Server next destination: " + String(dest));
        currentPosition++;
      }
    }
  } else {
    nodeTriggered = false; // reset for next node
  }
}

// ===============================
// Read all 5 sensors
void readSensors() {
  for (int i = 0; i < 5; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }
}

// ===============================
// Calculate line error
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

  if (count == 0) return 100; // line lost

  return sum / count;
}

// ===============================
// Motor control
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
    // Line lost → move straight
    setMotors(baseSpeed, baseSpeed);
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
// Node detection: all 5 sensors detect white
bool isNode() {
  for (int i = 0; i < 5; i++) {
    if (sensorValues[i] >= threshold) return false; // not all white
  }
  return true;
}

// ===============================
// Wi-Fi / Server Functions
void connectToWiFi() {
  Serial.print("Connecting to Wi-Fi: "); Serial.println(ssid);
  WiFi.begin(ssid,password);
  int attempts=0;
  while(WiFi.status()!=WL_CONNECTED && attempts<30) {
    Serial.print(".");
    delay(500);
    attempts++;
  }
  if(WiFi.status()==WL_CONNECTED){
    Serial.println("\nConnected to Wi-Fi!");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to Wi-Fi.");
  }
}

bool connectToServer() {
  Serial.print("Connecting to server: "); Serial.print(server); Serial.print(":"); Serial.println(port);
  if(!client.connect(server,port)){
    Serial.println("Error connecting to server");
    return false;
  }
  return true;
}

String readResponse() {
  char buffer[BUFSIZE]; 
  memset(buffer,0,BUFSIZE);
  client.readBytes(buffer,BUFSIZE);
  return String(buffer);
}

int getStatusCode(String &response) {
  if(response.length()<12) return 0;
  return response.substring(9,12).toInt();
}

String getResponseBody(String &response) {
  int split = response.indexOf("\r\n\r\n");
  if(split==-1) return "";
  String body = response.substring(split+4); 
  body.trim();
  return body;
}

// Returns: -1=error, -2=finished, >=0=next destination
int notifyArrival(int position) {
  if(!connectToServer()) return -1;
  String postBody="position="+String(position);
  client.println("POST /api/arrived/"+String(teamID)+" HTTP/1.1");
  client.println("Host: "+String(server));
  client.println("Content-Type: application/x-www-form-urlencoded");
  client.print("Content-Length: "); client.println(postBody.length());
  client.println(); // end headers
  client.println(postBody);
  String response=readResponse();
  int statusCode=getStatusCode(response);
  client.stop();
  if(statusCode!=200) return -1;
  String body=getResponseBody(response);
  if(body.equals("Finished")) return -2;
  return body.toInt();
}

