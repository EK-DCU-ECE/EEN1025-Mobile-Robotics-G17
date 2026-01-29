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

// Startup guard
void clearStartupMarker();

void doStraight();
void doLeft90();
void doRight90();
void doUTurn(); // sensor-based robust U-turn
void reacquireLineAfterTurn(bool keepTurningLeft, bool keepTurningRight);

// marker helpers
int  countLineSensors();
bool goodLineLock();
void resetMarkerDebounce();
void clearMarkerSmart(bool afterTurn);

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
const bool LINE_IS_BLACK = false;
int threshold = 250;

// Controller
int baseSpeed = 140;
int Kp = 80;
int turnSpeed = 130;

// Timing (tune these)
int SPIN_90_MS      = 320;
int CLEAR_FWD_SPEED = 155;

// Startup: ignore marker detection briefly
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
enum MarkerID : int8_t {
  M0 = 0,
  M1 = 1,
  M2 = 2,
  M3 = 3,
  M4 = 4,
  M5 = 5,
  MJ05 = 6,   // junction marker 0.5 (NOT a node)
  MJ35 = 7    // junction marker 3.5 (NOT a node)
};

const int NUM_MARKERS = 8;

bool isRealNode(MarkerID m) {
  return (m == M0 || m == M1 || m == M2 || m == M3 || m == M4 || m == M5);
}

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

Action JUNC_LUT[NUM_MARKERS][NUM_MARKERS][NUM_MARKERS];

void initJunctionLUT() {
  for (int a=0; a<NUM_MARKERS; a++)
    for (int b=0; b<NUM_MARKERS; b++)
      for (int c=0; c<NUM_MARKERS; c++)
        JUNC_LUT[a][b][c] = ACT_UNKNOWN;

  // MJ05 (0,1,2)
  JUNC_LUT[M0][MJ05][M1] = ACT_LEFT90;
  JUNC_LUT[M1][MJ05][M0] = ACT_RIGHT90;

  JUNC_LUT[M0][MJ05][M2] = ACT_STRAIGHT;
  JUNC_LUT[M2][MJ05][M0] = ACT_STRAIGHT;

  JUNC_LUT[M1][MJ05][M2] = ACT_LEFT90;
  JUNC_LUT[M2][MJ05][M1] = ACT_RIGHT90;

  // MJ35 (1,3,4)
  JUNC_LUT[M1][MJ35][M3] = ACT_RIGHT90;
  JUNC_LUT[M3][MJ35][M1] = ACT_LEFT90;

  JUNC_LUT[M1][MJ35][M4] = ACT_LEFT90;
  JUNC_LUT[M4][MJ35][M1] = ACT_RIGHT90;

  JUNC_LUT[M3][MJ35][M4] = ACT_STRAIGHT;
  JUNC_LUT[M4][MJ35][M3] = ACT_STRAIGHT;

  // Optional node 5 via MJ35
  JUNC_LUT[M1][MJ35][M5] = ACT_STRAIGHT;
  JUNC_LUT[M5][MJ35][M1] = ACT_STRAIGHT;
}

// ===============================
// ===== LEG / PATH (WEIGHTED NODES) =====
// ===============================
MarkerID legPath[20];
int legLen = 0;
int legIdx = 0;

const int NODES = 6;
const int MAX_DEG = 5;

// Node adjacency (ONLY nodes 0..5). Includes direct 0<->4 and 2<->3.
int adj[NODES][MAX_DEG] = {
  {1, 2, 4, -1, -1},      // 0
  {0, 2, 3, 4, 5},        // 1
  {0, 1, 3, -1, -1},      // 2
  {2, 1, 4, -1, -1},      // 3
  {0, 1, 3, -1, -1},      // 4
  {1, -1, -1, -1, -1}     // 5
};

// Weights follow your spec:
// MJ35<->1 = 1, 0<->4 and 2<->3 = 2, 3<->MJ35 and 4<->MJ35 = 3, 3<->4 = 4.
// For MJ05 cluster edges (0/1/2) you didn't specify: set to 3 so it loses to the fast direct edges.
int w[NODES][MAX_DEG] = {
  {3, 3, 2, -1, -1},      // 0->1=3 (via MJ05), 0->2=3 (via MJ05), 0->4=2 (direct)
  {3, 3, 4, 4, 4},        // 1->0=3, 1->2=3, 1->3=4 (1+3 via MJ35), 1->4=4 (1+3), 1->5=4
  {3, 3, 2, -1, -1},      // 2->0=3, 2->1=3, 2->3=2 (direct)
  {2, 4, 4, -1, -1},      // 3->2=2, 3->1=4, 3->4=4 (direct slowest)
  {2, 4, 4, -1, -1},      // 4->0=2, 4->1=4, 4->3=4
  {4, -1, -1, -1, -1}     // 5->1=4
};

// Insert junction markers ONLY when needed (junctions are NOT nodes)
bool usesMJ05(int a, int b) {
  return (a <= 2 && b <= 2 && a != b);
}
bool usesMJ35(int a, int b) {
  bool inA = (a == 1 || a == 3 || a == 4 || a == 5);
  bool inB = (b == 1 || b == 3 || b == 4 || b == 5);
  if (!inA || !inB) return false;
  if ((a == 3 && b == 4) || (a == 4 && b == 3)) return false; // direct edge, no junction
  return (a != b);
}

// Weighted shortest path (Dijkstra) over nodes, then expand into marker list with junctions.
void buildLeg(int fromNode, int toNode) {
  legLen = 0;
  legIdx = 0;

  const int INF = 1000000000;
  int dist[NODES];
  int parent[NODES];
  bool used[NODES];

  for (int i=0; i<NODES; i++) {
    dist[i] = INF;
    parent[i] = -1;
    used[i] = false;
  }
  dist[fromNode] = 0;

  for (int it=0; it<NODES; it++) {
    int u = -1;
    int best = INF;
    for (int i=0; i<NODES; i++) {
      if (!used[i] && dist[i] < best) { best = dist[i]; u = i; }
    }
    if (u == -1) break;
    used[u] = true;
    if (u == toNode) break;

    for (int k=0; k<MAX_DEG; k++) {
      int v = adj[u][k];
      if (v < 0) break;
      int ww = w[u][k];
      if (ww < 0) continue;

      if (dist[u] + ww < dist[v]) {
        dist[v] = dist[u] + ww;
        parent[v] = u;
      }
    }
  }

  int nodePath[12];
  int nodeLen = 0;
  int cur = toNode;
  while (cur != -1 && nodeLen < 12) {
    nodePath[nodeLen++] = cur;
    if (cur == fromNode) break;
    cur = parent[cur];
  }

  if (nodeLen == 0 || nodePath[nodeLen-1] != fromNode) {
    nodePath[0] = fromNode;
    nodePath[1] = toNode;
    nodeLen = 2;
  } else {
    for (int i=0; i<nodeLen/2; i++) {
      int tmp = nodePath[i];
      nodePath[i] = nodePath[nodeLen-1-i];
      nodePath[nodeLen-1-i] = tmp;
    }
  }

  legPath[legLen++] = nodeToMarker(nodePath[0]);

  for (int i=0; i<nodeLen-1; i++) {
    int a = nodePath[i];
    int b = nodePath[i+1];

    if (usesMJ05(a,b))      legPath[legLen++] = MJ05;
    else if (usesMJ35(a,b)) legPath[legLen++] = MJ35;

    legPath[legLen++] = nodeToMarker(b);
    if (legLen >= 19) break;
  }

  Serial.print("Leg path: ");
  for (int i=0; i<legLen; i++) {
    Serial.print((int)legPath[i]);
    if (i < legLen-1) Serial.print(" -> ");
  }
  Serial.print("   cost=");
  Serial.println(dist[toNode]);
}

// ===============================
// ===== STATE =====
// ===============================
int currentNode = 0;
int targetNode  = -1;
bool finished   = false;

MarkerID prevMarker = M0;

// Debounced marker state
int markerOnCount = 0;
int markerOffCount = 0;
const int MARKER_ON_CONFIRM  = 4;
const int MARKER_OFF_CONFIRM = 4;
bool markerLatched = false;

// Turning guard
volatile bool isTurning = false;

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

  if (isTurning) {
    delay(5);
    return;
  }

  if (millis() < ignoreMarkersUntil) {
    followLine();
    delay(5);
    return;
  }

  bool m = isMarker();

  if (m) {
    markerOnCount++;
    markerOffCount = 0;
  } else {
    markerOffCount++;
    markerOnCount = 0;
  }

  if (markerOnCount >= MARKER_ON_CONFIRM && !markerLatched) {
    markerLatched = true;
    handleMarkerHit();
  }

  if (markerLatched && markerOffCount >= MARKER_OFF_CONFIRM) {
    markerLatched = false;
  }

  if (!markerLatched && millis() >= ignoreMarkersUntil) {
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

  MarkerID cameFrom = prevMarker;

  if (legIdx < legLen - 1) legIdx++;
  MarkerID curr = legPath[legIdx];

  MarkerID next = curr;
  if (legIdx < legLen - 1) next = legPath[legIdx + 1];

  Serial.print("Marker hit. prev="); Serial.print((int)prevMarker);
  Serial.print(" curr="); Serial.print((int)curr);
  Serial.print(" next="); Serial.println((int)next);

  // Junction marker
  if (curr == MJ05 || curr == MJ35) {
    Action a = JUNC_LUT[prevMarker][curr][next];
    if (a == ACT_UNKNOWN) a = ACT_STRAIGHT;

    if (a == ACT_STRAIGHT) doStraight();
    else if (a == ACT_LEFT90) doLeft90();
    else if (a == ACT_RIGHT90) doRight90();
    else if (a == ACT_UTURN) doUTurn();

    clearMarkerSmart(true);
    prevMarker = curr;
    return;
  }

  // Node marker
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

      // IMMEDIATE OPTIMIZATION RULE:
      // If next leg is 0 -> 4, do a U-turn at node 0 immediately (don't drive away first)
      if (currentNode == 0 && targetNode == 4) {
        Serial.println("Optimization: at node 0 heading to 4 -> immediate U-turn");
        doUTurn();
      } else {
        // Otherwise, keep your existing "go back where we came from" rule
        if (legLen >= 2 && legPath[1] == cameFrom) {
          Serial.println("Next leg goes back the way we came -> U-turn at node");
          doUTurn();
        }
      }

      clearMarkerSmart(false);
      prevMarker = curr;
      return;
    }
  }

  clearMarkerSmart(false);
  prevMarker = curr;
}

// ===============================
// ===== LINE FOLLOW =====
// ===============================
void readSensors() {
  for (int i=0; i<5; i++) sensorValues[i] = analogRead(sensorPins[i]);
}

bool isMarker() {
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
// ===== SENSOR HELPERS =====
// ===============================
int countLineSensors() {
  int c = 0;
  for (int i = 0; i < 5; i++) if (seesLine(sensorValues[i])) c++;
  return c;
}

bool goodLineLock() {
  int c = countLineSensors();
  if (c == 5) return false;                 // marker bar
  if (!seesLine(sensorValues[2])) return false;
  if (c < 1 || c > 3) return false;         // narrow-ish line
  return true;
}

void resetMarkerDebounce() {
  markerOnCount = 0;
  markerOffCount = MARKER_OFF_CONFIRM;
  markerLatched = false;
}

// ===============================
// ===== SMART CLEAR MARKER =====
// ===============================
void clearMarkerSmart(bool afterTurn) {
  unsigned long t0 = millis();
  int offCount = 0;

  setMotors(CLEAR_FWD_SPEED, CLEAR_FWD_SPEED);

  while (millis() - t0 < 900) {
    readSensors();
    if (!isMarker()) offCount++;
    else offCount = 0;

    if (offCount >= 6) break;
    delay(8);
  }

  setMotors(0, 0);
  delay(40);

  resetMarkerDebounce();
  ignoreMarkersUntil = millis() + (afterTurn ? 220 : 320);
}

// ===============================
// ===== BASIC MANEUVERS =====
// ===============================
void doStraight() { }

void doLeft90() {
  isTurning = true;
  setMotors(turnSpeed, -turnSpeed);
  delay(SPIN_90_MS);
  reacquireLineAfterTurn(true, false);
  isTurning = false;
}

void doRight90() {
  isTurning = true;
  setMotors(-turnSpeed, turnSpeed);
  delay(SPIN_90_MS);
  reacquireLineAfterTurn(false, true);
  isTurning = false;
}

// SENSOR-BASED LEFT U-TURN
void doUTurn() {
  isTurning = true;

  // move forward to leave the bar
  setMotors(140, 140);
  delay(160);

  unsigned long t0 = millis();

  const int UTURN_MIN_MS = 560;
  const int UTURN_MAX_MS = 2300;
  const int STABLE_NEED  = 6;

  setMotors(turnSpeed, -turnSpeed); // left
  delay(UTURN_MIN_MS);

  bool leftMarkerOnce = false;
  int stable = 0;

  while (millis() - t0 < (unsigned long)UTURN_MAX_MS) {
    readSensors();

    bool onBar = isMarker();
    if (!onBar) leftMarkerOnce = true;

    if (leftMarkerOnce && goodLineLock()) stable++;
    else stable = 0;

    if (stable >= STABLE_NEED) break;

    setMotors(turnSpeed, -turnSpeed);
    delay(6);
  }

  setMotors(0, 0);
  delay(80);

  resetMarkerDebounce();
  ignoreMarkersUntil = millis() + 280;

  isTurning = false;
}

void reacquireLineAfterTurn(bool keepTurningLeft, bool keepTurningRight) {
  isTurning = true;

  unsigned long t0 = millis();

  setMotors(120, 120);
  delay(120);

  while (true) {
    readSensors();

    bool centerOnLine = seesLine(sensorValues[2]);
    bool onMarkerNow  = isMarker();

    if (centerOnLine && !onMarkerNow) break;

    if (keepTurningLeft)  setMotors(-110, 110);
    if (keepTurningRight) setMotors(110, -110);

    if (millis() - t0 > 2500) break;
    delay(5);
  }

  setMotors(0, 0);
  delay(80);

  ignoreMarkersUntil = millis() + 250;
  isTurning = false;
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
