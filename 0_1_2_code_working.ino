#include <WiFi.h>

// ===============================
// ===== FORWARD DECLARATIONS =====
// ===============================
void connectToWiFi();
int  notifyArrival(int position);

void readSensors();
bool isMarker();
void followLine();
int  getLineError();
void setMotors(int leftSpeed, int rightSpeed);

void buildLeg(int fromNode, int toNode);
void handleMarkerHit();
void clearMarker();

// Startup guard
void clearStartupMarker();

void doStraight();
void doLeft90();
void doRight90();
void doUTurn();
void reacquireLineAfterTurn(bool keepTurningLeft, bool keepTurningRight);

// Server helpers
bool   connectToServer();
String readResponseWithTimeout(uint32_t timeoutMs);
int    getStatusCode(String &response);
String getResponseBody(String &response);

// ===============================
// ===== PINS & VARIABLES =====
// ===============================
int sensorPins[5] = {4, 5, 6, 7, 15};
int sensorValues[5];

// Motor Pins
int leftPWM    = 37;
int leftPhase  = 38;
int rightPWM   = 39;
int rightPhase = 20;

// ===============================
// ===== LINE LOGIC SETTINGS =====
// ===============================
// Track image: BLACK line on WHITE background.
// Typical reflectance sensors: BLACK => higher ADC value, WHITE => lower.
// If your readings are inverted, set this to false.
const bool LINE_IS_BLACK = false;

// Threshold: tune after printing raw ADC values.
int threshold = 250;

// Controller
int baseSpeed = 140;
int Kp = 80;
int turnSpeed = 130;

// Timing (tune these)
int CLEAR_MARKER_MS = 180;
int SPIN_90_MS      = 260;
int SPIN_180_MS     = 520;

// Startup: ignore marker detection briefly to stop startup "spin"
const unsigned long STARTUP_IGNORE_MS = 1500;
unsigned long ignoreMarkersUntil = 0;

// Wi-Fi / server
const char* ssid = "iot";
const char* password = "bosselation7isacoustic";
const char* server = "3.250.38.184";
const int   port   = 8000;
const char* teamID = "dtbv6902";
WiFiClient client;

// ===============================
// ===== MARKER MODEL =====
// ===============================
// Treat nodes and junctions as "markers" with integer IDs.
enum MarkerID : int8_t {
  M0 = 0,
  M1 = 1,
  M2 = 2,
  M3 = 3,
  M4 = 4,
  M5 = 5,
  MJ05 = 6,   // junction marker 0.5
  MJ35 = 7    // junction marker 3.5
};

const int NUM_MARKERS = 8;

// Helper: is this a real node (server destinations)?
bool isRealNode(MarkerID m) {
  return (m == M0 || m == M1 || m == M2 || m == M3 || m == M4 || m == M5);
}

// Helper: map node number from server (0..5) to marker ID
MarkerID nodeToMarker(int node) {
  switch (node) {
    case 0: return M0;
    case 1: return M1;
    case 2: return M2;
    case 3: return M3;
    case 4: return M4;
    case 5: return M5;
    default: return M0;
  }
}

// Inline helper: does this reading indicate line/ink under sensor?
inline bool seesLine(int v) {
  return LINE_IS_BLACK ? (v > threshold) : (v < threshold);
}

// ===============================
// ===== ACTIONS AT JUNCTIONS =====
// ===============================
enum Action : int8_t {
  ACT_UNKNOWN  = -1,
  ACT_STRAIGHT = 0,
  ACT_LEFT90   = 1,
  ACT_RIGHT90  = 2,
  ACT_UTURN    = 3
};

// 3D LUT: action depends on (prev, curr, next).
Action JUNC_LUT[NUM_MARKERS][NUM_MARKERS][NUM_MARKERS];

// Fill lookup rules
void initJunctionLUT() {
  for (int a=0; a<NUM_MARKERS; a++)
    for (int b=0; b<NUM_MARKERS; b++)
      for (int c=0; c<NUM_MARKERS; c++)
        JUNC_LUT[a][b][c] = ACT_UNKNOWN;

  // 0 -> 1 and 1 -> 0
  JUNC_LUT[M0][MJ05][M1] = ACT_LEFT90;
  JUNC_LUT[M1][MJ05][M0] = ACT_RIGHT90;

  // 0 -> 2 and 2 -> 0
  JUNC_LUT[M0][MJ05][M2] = ACT_STRAIGHT;
  JUNC_LUT[M2][MJ05][M0] = ACT_STRAIGHT;

  // 1 -> 2 and 2 -> 1
  JUNC_LUT[M1][MJ05][M2] = ACT_LEFT90;
  JUNC_LUT[M2][MJ05][M1] = ACT_RIGHT90;

  // 1 -> 3 and 3 -> 1
  JUNC_LUT[M1][MJ35][M3] = ACT_RIGHT90;
  JUNC_LUT[M3][MJ35][M1] = ACT_LEFT90;

  // 1 -> 4 and 4 -> 1
  JUNC_LUT[M1][MJ35][M4] = ACT_LEFT90;
  JUNC_LUT[M4][MJ35][M1] = ACT_RIGHT90;

  // 3 -> 4 and 4 -> 3
  JUNC_LUT[M3][MJ35][M4] = ACT_STRAIGHT;
  JUNC_LUT[M4][MJ35][M3] = ACT_STRAIGHT;
}

// ===============================
// ===== LEG / PATH EXPANSION =====
// ===============================
MarkerID legPath[10];
int legLen = 0;
int legIdx = 0;

void buildLeg(int fromNode, int toNode) {
  legLen = 0;
  legIdx = 0;

  MarkerID from = nodeToMarker(fromNode);
  MarkerID to   = nodeToMarker(toNode);

  legPath[legLen++] = from;

  if ((from == M0 || from == M2) && to == M1) {
    legPath[legLen++] = MJ05;
    legPath[legLen++] = M1;
  }
  else if (from == M1 && (to == M0 || to == M2)) {
    legPath[legLen++] = MJ05;
    legPath[legLen++] = to;
  }
  else if ((from == M0 && to == M2) || (from == M2 && to == M0)) {
    legPath[legLen++] = MJ05;
    legPath[legLen++] = to;
  }
  else if (from == M1 && to == M3) {
    legPath[legLen++] = MJ35;
    legPath[legLen++] = M3;
  }
  else if (from == M3 && to == M1) {
    legPath[legLen++] = MJ35;
    legPath[legLen++] = M1;
  }
  else if (from == M1 && to == M5) {
    legPath[legLen++] = MJ35;
    legPath[legLen++] = M5;
  }
  else if (from == M5 && to == M1) {
    legPath[legLen++] = MJ35;
    legPath[legLen++] = M1;
  }
  else {
    legPath[legLen++] = to;
  }

  Serial.print("Leg path: ");
  for (int i=0; i<legLen; i++) {
    Serial.print((int)legPath[i]);
    if (i < legLen-1) Serial.print(" -> ");
  }
  Serial.println();
}

// ===============================
// ===== STATE =====
// ===============================
int currentNode = 0;
int targetNode  = -1;
bool finished   = false;

MarkerID prevMarker = M0;
static bool markerTriggered = false;

// ===============================
// ===== SETUP =====
// ===============================
void setup() {
  Serial.begin(115200);
  delay(800);

  pinMode(leftPWM, OUTPUT);   pinMode(leftPhase, OUTPUT);
  pinMode(rightPWM, OUTPUT);  pinMode(rightPhase, OUTPUT);

  initJunctionLUT();
  connectToWiFi();

  targetNode = notifyArrival(currentNode);
  if (targetNode == -2) { finished = true; return; }
  if (targetNode < 0) { targetNode = 1; }

  buildLeg(currentNode, targetNode);
  prevMarker = nodeToMarker(currentNode);

  // Clear any start marker (thick bar) and ignore markers briefly
  readSensors();
  clearStartupMarker();
  ignoreMarkersUntil = millis() + STARTUP_IGNORE_MS;
}

// ===============================
// ===== LOOP =====
// ===============================
void loop() {
  if (finished) { setMotors(0, 0); return; }

  readSensors();

  // Hard ignore window prevents startup marker-trigger spin
  if (millis() < ignoreMarkersUntil) {
    followLine();
    delay(5);
    return;
  }

  if (isMarker()) {
    if (!markerTriggered) {
      markerTriggered = true;
      handleMarkerHit();
      // small cooldown after handling a marker
      ignoreMarkersUntil = millis() + 250;
    }
  } else {
    markerTriggered = false;
    followLine();
  }

  delay(5);
}

// ===============================
// ===== STARTUP MARKER CLEAR =====
// ===============================
void clearStartupMarker() {
  unsigned long t0 = millis();
  while (isMarker() && (millis() - t0 < 1500)) {
    setMotors(120, 120);
    readSensors();
    delay(10);
  }
  setMotors(0, 0);
  delay(100);
}

// ===============================
// ===== MARKER HANDLER =====
// ===============================
void handleMarkerHit() {
  setMotors(0, 0);
  delay(120);

  // ✅ SAVE where we came from BEFORE we change anything
  MarkerID cameFrom = prevMarker;

  if (legIdx < legLen - 1) legIdx++;
  MarkerID curr = legPath[legIdx];

  MarkerID next = curr;
  if (legIdx < legLen - 1) next = legPath[legIdx + 1];

  Serial.print("Marker hit. prev="); Serial.print((int)prevMarker);
  Serial.print(" curr="); Serial.print((int)curr);
  Serial.print(" next="); Serial.println((int)next);

  // ---- Junction marker behavior (unchanged) ----
  if (curr == MJ05 || curr == MJ35) {
    Action a = JUNC_LUT[prevMarker][curr][next];
    if (a == ACT_UNKNOWN) a = ACT_STRAIGHT;

    if (a == ACT_STRAIGHT) doStraight();
    else if (a == ACT_LEFT90) doLeft90();
    else if (a == ACT_RIGHT90) doRight90();
    else if (a == ACT_UTURN) doUTurn();

    clearMarker();
    prevMarker = curr;
    return;
  }

  // ---- Real node behavior (FIXED) ----
  if (isRealNode(curr)) {
    int arrivedNode = (int)curr;
    Serial.print("Arrived at node "); Serial.println(arrivedNode);

    if (arrivedNode == targetNode) {
      currentNode = targetNode;
      int nextDest = notifyArrival(currentNode);

      if (nextDest == -2) {
        finished = true;
        setMotors(0, 0);
        Serial.println("Server says Finished.");
        return;
      }
      if (nextDest < 0) {
        Serial.println("Server error. Stopping.");
        finished = true;
        return;
      }

      targetNode = nextDest;
      buildLeg(currentNode, targetNode);

      // ✅ KEY FIX:
      // If next leg starts by going back to the marker we came from,
      // we must do a U-turn at this node before leaving it.
      if (legLen >= 2 && legPath[1] == cameFrom) {
        Serial.println("Next leg goes back the way we came -> U-turn at node");
        doUTurn();
      }

      clearMarker();     // drive off node marker in correct direction
      prevMarker = curr; // keep state consistent for next junction
      return;
    }
  }

  clearMarker();
  prevMarker = curr;
}

// ===============================
// ===== LINE FOLLOW =====
// ===============================
void readSensors() {
  for (int i=0; i<5; i++) sensorValues[i] = analogRead(sensorPins[i]);
}

bool isMarker() {
  // Marker = thick bar covers all sensors with line/ink
  for (int i=0; i<5; i++) {
    if (!seesLine(sensorValues[i])) return false;
  }
  return true;
}

int getLineError() {
  int weights[5] = {-2, -1, 0, 1, 2};
  int sum = 0, count = 0;

  for (int i=0; i<5; i++) {
    if (seesLine(sensorValues[i])) {
      sum += weights[i];
      count++;
    }
  }
  if (count == 0) return 100;
  return sum / count;
}

void followLine() {
  int error = getLineError();

  if (error == 100) {
    setMotors(120, 120);
    return;
  }

  int correction = Kp * error;
  int L = constrain(baseSpeed - correction, 0, 200);
  int R = constrain(baseSpeed + correction, 0, 200);
  setMotors(L, R);
}

// ===============================
// ===== MOTORS =====
// ===============================
void setMotors(int leftSpeed, int rightSpeed) {
  digitalWrite(leftPhase,  leftSpeed  >= 0 ? LOW  : HIGH);
  digitalWrite(rightPhase, rightSpeed >= 0 ? HIGH : LOW);
  analogWrite(leftPWM, abs(leftSpeed));
  analogWrite(rightPWM, abs(rightSpeed));
}

// ===============================
// ===== BASIC MANEUVERS =====
// ===============================
void clearMarker() {
  setMotors(160, 160);
  delay(CLEAR_MARKER_MS);
  setMotors(0, 0);
  delay(50);
}

void doStraight() { }

void doLeft90() {
  setMotors(turnSpeed, -turnSpeed);
  delay(SPIN_90_MS);
  reacquireLineAfterTurn(true, false);
}

void doRight90() {
  setMotors(-turnSpeed, turnSpeed);
  delay(SPIN_90_MS);
  reacquireLineAfterTurn(false, true);
}

void doUTurn() {
  setMotors(turnSpeed, -turnSpeed);
  delay(SPIN_180_MS);
  reacquireLineAfterTurn(false, true);
}

void reacquireLineAfterTurn(bool keepTurningLeft, bool keepTurningRight) {
  unsigned long t0 = millis();
  while (!seesLine(analogRead(sensorPins[2]))) {
    if (keepTurningLeft)  setMotors(-110, 110);
    if (keepTurningRight) setMotors(110, -110);

    if (millis() - t0 > 2000) break;
    delay(5);
  }
  setMotors(0, 0);
  delay(80);
}

// ===============================
// ===== WIFI / SERVER =====
// ===============================
void connectToWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }

  Serial.println("\nWiFi CONNECTED");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

bool connectToServer() {
  return client.connect(server, port);
}

String readResponseWithTimeout(uint32_t timeoutMs) {
  String resp;
  unsigned long deadline = millis() + timeoutMs;

  while (millis() < deadline) {
    while (client.available()) resp += (char)client.read();
    if (!client.connected()) break;
    delay(5);
  }
  return resp;
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

int notifyArrival(int position) {
  if (!connectToServer()) return -1;

  String postBody = "position=" + String(position);

  client.println("POST /api/arrived/" + String(teamID) + " HTTP/1.1");
  client.println("Host: " + String(server));
  client.println("Content-Type: application/x-www-form-urlencoded");
  client.print("Content-Length: ");
  client.println(postBody.length());
  client.println();
  client.println(postBody);

  String response = readResponseWithTimeout(2500);
  int statusCode = getStatusCode(response);
  client.stop();

  if (statusCode != 200) return -1;

  String body = getResponseBody(response);
  if (body.equalsIgnoreCase("Finished")) return -2;

  return body.toInt();
}
