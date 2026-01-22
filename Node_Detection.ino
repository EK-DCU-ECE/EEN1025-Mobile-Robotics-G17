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

// Line tuning
int threshold = 500;   // white < threshold
int baseSpeed = 160;
int Kp = 60;
int turnSpeed = 130;

// Timing (tune these)
int CLEAR_MARKER_MS = 180;
int SPIN_90_MS      = 260;
int SPIN_180_MS     = 520;

// Wi-Fi / server
const char* ssid = "iPhone";
const char* password = "12345678";
const char* server = "3.250.38.184";
const int   port   = 8000;
const char* teamID = "dtbv6902";
WiFiClient client;

// ===============================
// ===== MARKER MODEL =====
// ===============================
// We treat nodes and junctions as "markers" with integer IDs.
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
// Default everything to ACT_UNKNOWN; we’ll set the important junction rules.
Action JUNC_LUT[NUM_MARKERS][NUM_MARKERS][NUM_MARKERS];

// Fill lookup rules
void initJunctionLUT() {
  // Default
  for (int a=0; a<NUM_MARKERS; a++)
    for (int b=0; b<NUM_MARKERS; b++)
      for (int c=0; c<NUM_MARKERS; c++)
        JUNC_LUT[a][b][c] = ACT_UNKNOWN;

  // ---------------------------
  // Junction 0.5 (MJ05)
  // ---------------------------
  // From node 0 up the right side to node 2, you pass MJ05.
  // If next is node 1, take the branch (your example: 0->1 => at 0.5 turn LEFT)
  JUNC_LUT[M0][MJ05][M1] = ACT_LEFT90;
  JUNC_LUT[M2][MJ05][M1] = ACT_LEFT90;

  // If staying on the outer loop (0->2 or 2->0), go straight past MJ05
  JUNC_LUT[M0][MJ05][M2] = ACT_STRAIGHT;
  JUNC_LUT[M2][MJ05][M0] = ACT_STRAIGHT;

  // Coming from node 1 to junction 0.5 (heading east), then choose up/down on outer loop:
  // To go to node 2: turn LEFT
  // To go to node 0: turn RIGHT
  JUNC_LUT[M1][MJ05][M2] = ACT_LEFT90;
  JUNC_LUT[M1][MJ05][M0] = ACT_RIGHT90;

  // ---------------------------
  // Junction 3.5 (MJ35)
  // ---------------------------
  // This depends on your physical track behaviour.
  // A sensible starting assumption:
  // - From node 1 to node 5: at 3.5 turn LEFT into the inner loop
  // - From node 5 back to node 1: at 3.5 turn RIGHT to rejoin the middle line
  // If node 5 is not on that inner loop, you can ignore these.
  JUNC_LUT[M1][MJ35][M5] = ACT_LEFT90;
  JUNC_LUT[M5][MJ35][M1] = ACT_RIGHT90;

  // If you are simply passing through (e.g., 1->? where path uses MJ35), default straight:
  JUNC_LUT[M1][MJ35][M1] = ACT_STRAIGHT;
}

// ===============================
// ===== LEG / PATH EXPANSION =====
// ===============================
// The server gives nodes only (0..5). We expand into marker steps including junctions.
// Current design supports the known junction insertions:
// - Any leg that uses the middle branch passes MJ05 and/or MJ35 accordingly.
MarkerID legPath[10];
int legLen = 0;
int legIdx = 0;

// Build expanded marker path for one leg: fromNode -> toNode
// IMPORTANT: this is where we insert 0.5 and 3.5 when needed.
void buildLeg(int fromNode, int toNode) {
  legLen = 0;
  legIdx = 0;

  MarkerID from = nodeToMarker(fromNode);
  MarkerID to   = nodeToMarker(toNode);

  // Default: direct (no junction markers)
  legPath[legLen++] = from;

  // --- Rules for your map ---
  // To reach node 1 from outer loop nodes (0 or 2), you must go via junction 0.5.
  // 0 -> 1 : 0, 0.5, 1
  // 2 -> 1 : 2, 0.5, 1
  if ((from == M0 || from == M2) && to == M1) {
    legPath[legLen++] = MJ05;
    legPath[legLen++] = M1;
  }
  // To leave node 1 to outer loop node 0 or 2: 1, 0.5, 0 or 2
  else if (from == M1 && (to == M0 || to == M2)) {
    legPath[legLen++] = MJ05;
    legPath[legLen++] = to;
  }
  // Moving between 0 and 2 along the right side passes the junction marker 0.5 but you go straight.
  else if ((from == M0 && to == M2) || (from == M2 && to == M0)) {
    legPath[legLen++] = MJ05;
    legPath[legLen++] = to;
  }
  // Example inner-loop access via 3.5 if node 5 is inside:
  else if (from == M1 && to == M5) {
    legPath[legLen++] = MJ35;
    legPath[legLen++] = M5;
  }
  else if (from == M5 && to == M1) {
    legPath[legLen++] = MJ35;
    legPath[legLen++] = M1;
  }
  // Otherwise: assume direct (no junction markers)
  else {
    legPath[legLen++] = to;
  }

  // Debug print
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
int currentNode = 0;      // starts at node 0
int targetNode  = -1;     // next node from server
bool finished   = false;

MarkerID prevMarker = M0;  // last marker we confirmed
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

  // Ask server for first destination at node 0
  targetNode = notifyArrival(currentNode);
  if (targetNode == -2) { finished = true; return; }
  if (targetNode < 0) { targetNode = 1; } // fallback

  buildLeg(currentNode, targetNode);

  // initialise prev marker to start marker
  prevMarker = nodeToMarker(currentNode);
}

// ===============================
// ===== LOOP =====
// ===============================
void loop() {
  if (finished) { setMotors(0, 0); return; }

  readSensors();

  if (isMarker()) {
    if (!markerTriggered) {
      markerTriggered = true;
      handleMarkerHit();
    }
  } else {
    markerTriggered = false;
    followLine();
  }

  delay(5);
}

// ===============================
// ===== MARKER HANDLER =====
// ===============================
void handleMarkerHit() {
  setMotors(0, 0);
  delay(120);

  // We expect to see markers in the order of legPath[].
  // legPath[0] is the starting node marker; the first "new" marker is legPath[1].
  if (legIdx < legLen - 1) legIdx++;
  MarkerID curr = legPath[legIdx];

  // Determine next marker (if any)
  MarkerID next = curr;
  if (legIdx < legLen - 1) next = legPath[legIdx + 1];

  Serial.print("Marker hit. prev=");
  Serial.print((int)prevMarker);
  Serial.print(" curr=");
  Serial.print((int)curr);
  Serial.print(" next=");
  Serial.println((int)next);

  // If this is a junction marker, use junction LUT to decide turn
  if (curr == MJ05 || curr == MJ35) {
    Action a = JUNC_LUT[prevMarker][curr][next];

    // If unknown, safest is STRAIGHT
    if (a == ACT_UNKNOWN) a = ACT_STRAIGHT;

    Serial.print("Junction action: ");
    Serial.println((int)a);

    // execute
    if (a == ACT_STRAIGHT) doStraight();
    else if (a == ACT_LEFT90) doLeft90();
    else if (a == ACT_RIGHT90) doRight90();
    else if (a == ACT_UTURN) doUTurn();

    clearMarker();
    prevMarker = curr;
    return;
  }

  // Otherwise it’s a real node marker
  if (isRealNode(curr)) {
    int arrivedNode = (int)curr;  // M0..M5 match 0..5
    Serial.print("Arrived at node ");
    Serial.println(arrivedNode);

    // Only notify server when this is the destination node for the leg
    if (arrivedNode == targetNode) {
      currentNode = targetNode;
      int nextDest = notifyArrival(currentNode);

      clearMarker();
      prevMarker = curr;

      if (nextDest == -2) {
        finished = true;
        setMotors(0, 0);
        Serial.println("Server says Finished.");
        return;
      }
      if (nextDest < 0) {
        Serial.println("Server error. Holding position.");
        finished = true;
        return;
      }

      targetNode = nextDest;
      buildLeg(currentNode, targetNode);
      return;
    }
  }

  // If it was a node marker but not our destination, just clear and continue
  clearMarker();
  prevMarker = curr;
}

// ===============================
// ===== LINE FOLLOW =====
// ===============================
void readSensors() {
  for (int i=0; i<5; i++) sensorValues[i] = analogRead(sensorPins[i]);
}

// Marker detection: thick bar / junction looks like "all sensors see white"
bool isMarker() {
  for (int i=0; i<5; i++) {
    if (sensorValues[i] >= threshold) return false;
  }
  return true;
}

int getLineError() {
  int weights[5] = {-2, -1, 0, 1, 2};
  int sum = 0, count = 0;

  for (int i=0; i<5; i++) {
    if (sensorValues[i] < threshold) {
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

void doStraight() {
  // nothing special; just keep going after clearMarker()
}

void doLeft90() {
  setMotors(-turnSpeed, turnSpeed);
  delay(SPIN_90_MS);
  reacquireLineAfterTurn(true, false);
}

void doRight90() {
  setMotors(turnSpeed, -turnSpeed);
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
  while (analogRead(sensorPins[2]) >= threshold) {
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

// Returns: -1=error, -2=finished, >=0=next destination
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
