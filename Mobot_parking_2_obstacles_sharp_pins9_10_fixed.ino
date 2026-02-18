#include <WiFi.h>


// ===============================
// ===== BLOCKED SEGMENT STATE =====
// We block MARKER->MARKER segments (NOT node->node) so obstacles can be anywhere
// on the track (including between a junction bar and a node).
// Declared after NUM_MARKERS.
// ===============================


// ===============================
// ===== FORWARD DECLARATIONS =====
// ===============================
// Forward-declare the marker enum so it can be used in prototypes below.
// The full enum definition appears later in this file.
enum MarkerID : int8_t;

void connectToWiFi();
int  notifyArrival(int position);

void readSensors();
bool isMarker();
void followLine();
int  getLineError();
void setMotors(int leftSpeed, int rightSpeed);

void buildLeg(int fromNode, int toNode);
void buildLegFromMarker(MarkerID fromM, int toNode);
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

// ===== ULTRASONIC (ADDED) =====
int readUltrasonicCmFast();

// ===== SHARP IR (ADDED) =====
void readSharpSensors();
bool sharpObstacleDetected();

// ===== WALL AS NODE 5 (ADDED) =====
void arriveAtWallNode5();

// ===== OBSTACLE RETURN (ADDED) =====
void startReturnToPreviousNode();
int  findNextNodeInLeg(int startIdx);
void clearBlockedSeg();


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

// ===== ULTRASONIC (ADDED) =====
const int US_TRIG = 17;      // trig pin
const int US_ECHO = 16;      // echo pin (must be 3.3V safe on ESP32!)
const int STOP_CM  = 5;      // stop at 5cm
const int CLEAR_CM = 8;      // resume at 8cm (hysteresis)
const unsigned long US_PERIOD_MS = 60;

int usCm = -1;
unsigned long lastUSms = 0;
bool obstacleBlocked = false;

// ===== SHARP IR (ADDED) =====
// Two Sharp analog IR sensors for bend/side obstacle detection.
// Pins confirmed working by your test: GPIO9 and GPIO10.
const int SHARP_LEFT_PIN  = 9;
const int SHARP_RIGHT_PIN = 10;

// Recommended: use RAW-threshold detection (more robust than cm mapping).
// Tune these in Serial Monitor if needed.
const int SHARP_RAW_NEAR_L = 1800;
const int SHARP_RAW_NEAR_R = 1800;

const unsigned long SHARP_PERIOD_MS = 60;
const int SHARP_SAMPLES = 8;

int sharpRawL = 0;
int sharpRawR = 0;
bool sharpNearL = false;
bool sharpNearR = false;
unsigned long lastSharpMs = 0;

// ===============================
// ===== LINE LOGIC SETTINGS =====
// ===============================
const bool LINE_IS_BLACK = false;
int threshold = 250;

// Controller
int baseSpeed = 140;
int Kp = 80;
int turnSpeed = 130;

// ===============================
// Line-follow disable after MJ35 -> M5 (2s after leaving junction)
// ===============================
bool lineFollowEnabled = true;
bool lfDisableArmed = false;
uint32_t lfDisableAtMs = 0;
const uint32_t LF_DISABLE_DELAY_MS = 2000;


// Timing (tune these)
int SPIN_90_MS      = 320;
int CLEAR_FWD_SPEED = 155;

// Startup: ignore marker detection briefly
const unsigned long STARTUP_IGNORE_MS = 1500;
unsigned long ignoreMarkersUntil = 0;

// Wi-Fi / server
const char* ssid = "iot";
const char* password = "unwrinkleable66abrogative";
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

// =====================================================
// ===== MULTI-OBSTACLE SUPPORT (UP TO 2 EDGES) =====
// We still use a fast blockedSeg[][] matrix for O(1) checks in Dijkstra,
// but we also keep a small list so we can cap the number of simultaneously
// remembered obstacles and clear them individually.
// =====================================================
bool blockedSeg[NUM_MARKERS][NUM_MARKERS];

struct BlockedEdge {
  int u;
  int v;
  bool active;
  uint32_t stampedMs;
};

static const int MAX_BLOCKED_EDGES = 2;
BlockedEdge blockedEdges[MAX_BLOCKED_EDGES];
int blockedEdgeCount = 0;
int blockedEdgeReplaceIdx = 0;   // round-robin replacement if a 3rd obstacle appears
int lastBlockedEdgeIdx = -1;     // the edge that triggered the current RETURN/WAIT

int findBlockedEdgeIdx(int u, int v) {
  for (int i = 0; i < MAX_BLOCKED_EDGES; i++) {
    if (!blockedEdges[i].active) continue;
    int a = blockedEdges[i].u;
    int b = blockedEdges[i].v;
    if ((a == u && b == v) || (a == v && b == u)) return i;
  }
  return -1;
}

void clearBlockedEdgeIdx(int idx) {
  if (idx < 0 || idx >= MAX_BLOCKED_EDGES) return;
  if (!blockedEdges[idx].active) return;
  int u = blockedEdges[idx].u;
  int v = blockedEdges[idx].v;
  blockedSeg[u][v] = false;
  blockedSeg[v][u] = false;
  blockedEdges[idx].active = false;
  if (blockedEdgeCount > 0) blockedEdgeCount--;
  if (lastBlockedEdgeIdx == idx) lastBlockedEdgeIdx = -1;
}


// NOTE (USER REQUIREMENT): Blocked track sections NEVER clear during a run.
// So once an edge is blocked, the planner must avoid it forever (until reboot/reset).
// Keep this function as a no-op so older call sites don't need to change.
void clearBlockedIfTraversed(int /*u*/, int /*v*/) {
  // no-op
}

// Add a blocked edge (undirected). Returns the slot index used.
int addBlockedEdge(int u, int v) {
  int existing = findBlockedEdgeIdx(u, v);
  if (existing >= 0) {
    blockedEdges[existing].stampedMs = millis();
    blockedSeg[u][v] = true;
    blockedSeg[v][u] = true;
    return existing;
  }

  int slot = -1;
  // use a free slot if available
  for (int i = 0; i < MAX_BLOCKED_EDGES; i++) {
    if (!blockedEdges[i].active) { slot = i; break; }
  }

  // otherwise replace oldest via round-robin
  if (slot < 0) {
    slot = blockedEdgeReplaceIdx;
    blockedEdgeReplaceIdx = (blockedEdgeReplaceIdx + 1) % MAX_BLOCKED_EDGES;
    // clear the edge we're replacing from the matrix
    if (blockedEdges[slot].active) {
      int ou = blockedEdges[slot].u;
      int ov = blockedEdges[slot].v;
      blockedSeg[ou][ov] = false;
      blockedSeg[ov][ou] = false;
    }
  } else {
    blockedEdgeCount++;
  }

  blockedEdges[slot].u = u;
  blockedEdges[slot].v = v;
  blockedEdges[slot].active = true;
  blockedEdges[slot].stampedMs = millis();
  blockedSeg[u][v] = true;
  blockedSeg[v][u] = true;
  return slot;
}


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
  JUNC_LUT[M4][MJ35][M5] = ACT_LEFT90;
  JUNC_LUT[M3][MJ35][M5] = ACT_RIGHT90;
}

// ===============================
// ===== LEG / PATH (WEIGHTED MARKERS) =====
// We plan directly over markers: M0..M5 plus junctions MJ05/MJ35.
// This makes obstacle-blocking robust because we can block the exact segment
// we were travelling on: legPath[legIdx] -> legPath[legIdx+1].
// ===============================
MarkerID legPath[24];
int legLen = 0;
int legIdx = 0;

const int MAX_MDEG = 4;

// Marker adjacency (undirected; encode both directions explicitly)
int madj[NUM_MARKERS][MAX_MDEG] = {
  /* M0  */ {MJ05, M4,   -1,  -1},
  /* M1  */ {MJ05, MJ35, -1,  -1},
  /* M2  */ {MJ05, M3,   -1,  -1},
  /* M3  */ {M2,   MJ35, -1,  -1},
  /* M4  */ {M0,   MJ35, -1,  -1},
  /* M5  */ {MJ35, -1,   -1,  -1},
  /* MJ05*/ {M0,   M1,   M2,  -1},
  /* MJ35*/ {M1,   M3,   M4,  M5}
};

// Segment weights (tuned to preserve your old preferences as closely as possible)
// - Direct bottom/top segments stay cheap (2): M0<->M4, M2<->M3
// - MJ35 hub: M1<->MJ35 is very cheap (1), spokes to M3/M4/M5 are 3 (so 1->3 costs 4 like before)
// - MJ05 hub: set to 2 per spoke (so 0<->1 costs 4, slightly less attractive than before; stable + consistent)
int mw[NUM_MARKERS][MAX_MDEG] = {
  /* M0  */ {1, 2,  -1, -1},
  /* M1  */ {1, 1,  -1, -1},
  /* M2  */ {1, 2,  -1, -1},
  /* M3  */ {2, 3,  -1, -1},
  /* M4  */ {2, 3,  -1, -1},
  /* M5  */ {3, -1, -1, -1},
  /* MJ05*/ {1, 1,  1,  -1},
  /* MJ35*/ {1, 3,  3,  3}
};

void clearBlockedSeg() {
  for (int i=0; i<NUM_MARKERS; i++)
    for (int j=0; j<NUM_MARKERS; j++)
      blockedSeg[i][j] = false;

  for (int k=0; k<MAX_BLOCKED_EDGES; k++) {
    blockedEdges[k].active = false;
    blockedEdges[k].u = -1;
    blockedEdges[k].v = -1;
    blockedEdges[k].stampedMs = 0;
  }
  blockedEdgeCount = 0;
  blockedEdgeReplaceIdx = 0;
  lastBlockedEdgeIdx = -1;
}

// Dijkstra over marker graph (M0..M5 + MJ05/MJ35)
void buildLeg(int fromNode, int toNode) {
  legLen = 0;
  legIdx = 0;

  MarkerID fromM = nodeToMarker(fromNode);
  MarkerID toM   = nodeToMarker(toNode);

  const int INF = 1000000000;
  int  dist[NUM_MARKERS];
  int  parent[NUM_MARKERS];
  bool used[NUM_MARKERS];

  for (int i=0; i<NUM_MARKERS; i++) {
    dist[i] = INF;
    parent[i] = -1;
    used[i] = false;
  }
  dist[(int)fromM] = 0;

  for (int it=0; it<NUM_MARKERS; it++) {
    int u = -1;
    int best = INF;
    for (int i=0; i<NUM_MARKERS; i++) {
      if (!used[i] && dist[i] < best) { best = dist[i]; u = i; }
    }
    if (u == -1) break;
    used[u] = true;
    if (u == (int)toM) break;

    for (int k=0; k<MAX_MDEG; k++) {
      int v = madj[u][k];
      if (v < 0) break;
      int ww = mw[u][k];
      if (ww < 0) continue;

      // Skip blocked segment (treat as undirected segment)
      if (blockedSeg[u][v] || blockedSeg[v][u]) continue;

      if (dist[u] + ww < dist[v]) {
        dist[v] = dist[u] + ww;
        parent[v] = u;
      }
    }
  }

  // Reconstruct marker path
  int mp[32];
  int mlen = 0;
  int cur = (int)toM;
  while (cur != -1 && mlen < 32) {
    mp[mlen++] = cur;
    if (cur == (int)fromM) break;
    cur = parent[cur];
  }

  if (mlen == 0 || mp[mlen-1] != (int)fromM) {
    // No path under current blocked edges.
    // IMPORTANT: we do NOT clear blocks here because we may have 2 real obstacles.
    Serial.println("ERROR: No route found under current blocked edges.");
    legPath[0] = fromM;
    legLen = 1;
  } else {
    // reverse into legPath
    for (int i=0; i<mlen/2; i++) {
      int tmp = mp[i];
      mp[i] = mp[mlen-1-i];
      mp[mlen-1-i] = tmp;
    }

    legLen = 0;
    for (int i=0; i<mlen && legLen < 23; i++) {
      legPath[legLen++] = (MarkerID)mp[i];
    }
  }

  Serial.print("Leg path (markers): ");
  for (int i=0; i<legLen; i++) {
    Serial.print((int)legPath[i]);
    if (i < legLen-1) Serial.print(" -> ");
  }
  Serial.print("   cost=");
  Serial.println(dist[(int)toM]);
}

// Dijkstra over marker graph starting from an arbitrary MARKER (node or junction)
// to a NODE destination.
void buildLegFromMarker(MarkerID fromM, int toNode) {
  legLen = 0;
  legIdx = 0;

  MarkerID toM   = nodeToMarker(toNode);

  const int INF = 1000000000;
  int  dist[NUM_MARKERS];
  int  parent[NUM_MARKERS];
  bool used[NUM_MARKERS];

  for (int i=0; i<NUM_MARKERS; i++) {
    dist[i] = INF;
    parent[i] = -1;
    used[i] = false;
  }
  dist[(int)fromM] = 0;

  for (int it=0; it<NUM_MARKERS; it++) {
    int u = -1;
    int best = INF;
    for (int i=0; i<NUM_MARKERS; i++) {
      if (!used[i] && dist[i] < best) { best = dist[i]; u = i; }
    }
    if (u == -1) break;
    used[u] = true;
    if (u == (int)toM) break;

    for (int k=0; k<MAX_MDEG; k++) {
      int v = madj[u][k];
      if (v < 0) break;
      int ww = mw[u][k];
      if (ww < 0) continue;

      if (blockedSeg[u][v] || blockedSeg[v][u]) continue;

      if (dist[u] + ww < dist[v]) {
        dist[v] = dist[u] + ww;
        parent[v] = u;
      }
    }
  }

  int mp[32];
  int mlen = 0;
  int cur = (int)toM;
  while (cur != -1 && mlen < 32) {
    mp[mlen++] = cur;
    if (cur == (int)fromM) break;
    cur = parent[cur];
  }

  if (mlen == 0 || mp[mlen-1] != (int)fromM) {
    Serial.println("ERROR: No route found under current blocked edges.\n");
    legPath[0] = fromM;
    legLen = 1;
  } else {
    for (int i=0; i<mlen/2; i++) {
      int tmp = mp[i];
      mp[i] = mp[mlen-1-i];
      mp[mlen-1-i] = tmp;
    }
    legLen = 0;
    for (int i=0; i<mlen && legLen < 23; i++) {
      legPath[legLen++] = (MarkerID)mp[i];
    }
  }

  Serial.print("Leg path (markers): ");
  for (int i=0; i<legLen; i++) {
    Serial.print((int)legPath[i]);
    if (i < legLen-1) Serial.print(" -> ");
  }
  Serial.print("   cost=");
  Serial.println(dist[(int)toM]);
}


// ===============================
// ===== STATE =====
// ===============================
int currentNode = 0;
int targetNode  = -1;
bool finished   = false;

MarkerID prevMarker = M0;



// ===== Junction U-turn "came-from" override (ADDED) =====
// After an obstacle-triggered U-turn, the first marker we re-hit can be the same
// junction marker (MJ05/MJ35). In that moment, prevMarker is ambiguous and the
// junction LUT can fall back to STRAIGHT -> outer loop. These flags force a valid
// "came from" marker for the first marker-hit after the U-turn.
// Obstacle/junction return context: after a U-turn, we may re-hit the same junction
// without ever confirming the far-side marker. Store which junction we will re-hit and
// which marker side we attempted, so the LUT can choose the correct turn.
bool     obstOverrideActive = false;
MarkerID obstJuncMarker     = MJ05; // valid only when obstOverrideActive=true
MarkerID obstOtherMarker    = M0;   // valid only when obstOverrideActive=true
uint32_t obstOverrideArmedAt = 0; // millis() when override armed (expiry safety)
const uint32_t OBST_OVERRIDE_TTL_MS = 5000;
// Debounced marker state
int markerOnCount = 0;
int markerOffCount = 0;
const int MARKER_ON_CONFIRM  = 3;
const int MARKER_OFF_CONFIRM = 4;
bool markerLatched = false;

// Turning guard
volatile bool isTurning = false;

// ===============================
// ===== OBSTACLE RETURN STATE (ADDED) =====
// ===============================
bool returningToPrev = false;     // reversing along legPath
bool atPrevNodeWaiting = false;   // stopped at previous node waiting for obstacle clear
int  savedTargetNode = -1;        // optional: remembers where we were heading
bool returnFirstMarker = false;   // first marker hit after obstacle U-turn is usually the SAME marker we last confirmed
bool replanAtFirstAnchor = false; // after U-turn, replan as soon as we reach the first anchor marker (node or junction)

// ===== GENERIC U-TURN CONTEXT (ADDED) =====
// When an obstacle is detected while travelling from legPath[legIdx] -> legPath[legIdx+1],
// we store BOTH markers so the first RETURN hit can use a correct "prev" for the LUT.
bool     uTurnCtxValid    = false;
MarkerID uTurnFromMarker  = M0;   // marker we were at when obstacle was seen
MarkerID uTurnTriedMarker = M0;   // marker we were trying to reach when obstacle was seen
bool     uTurnPrevOverrideActive = false; // ONLY for the first RETURN marker hit

// Blocked edge (ADDED): avoid the segment where obstacle was detected
// ===============================
// ===== SETUP =====
// ===============================
void setup() {
  Serial.begin(115200);
  delay(800);

  // ===== ADC SETUP (ADDED) =====
  analogReadResolution(12);        // 0..4095
  analogSetAttenuation(ADC_11db);  // better scaling up to ~3.3V

  // Sharp pins (analog inputs)
  pinMode(SHARP_LEFT_PIN, INPUT);
  pinMode(SHARP_RIGHT_PIN, INPUT);

  pinMode(leftPWM, OUTPUT);   pinMode(leftPhase, OUTPUT);
  pinMode(rightPWM, OUTPUT);  pinMode(rightPhase, OUTPUT);

  // ===== ULTRASONIC SETUP (ADDED) =====
  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);
  digitalWrite(US_TRIG, LOW);

  initJunctionLUT();
  clearBlockedSeg();
  connectToWiFi();

  targetNode = notifyArrival(currentNode);
  if (targetNode == -2) { finished = true; return; }
  if (targetNode < 0) { targetNode = 1; }

  buildLeg(currentNode, targetNode);
  prevMarker = nodeToMarker(currentNode);

  readSensors();

  // ==========================================================
  // FIX: If the FIRST leg is 0 -> 4, do the U-turn IMMEDIATELY
  // BEFORE any forward "startup clear" motion.
  // This prevents accidentally hitting MJ05 first and desyncing
  // the legPath [M0, M4].
  // ==========================================================
  if (currentNode == 0 && targetNode == 4) {
    Serial.println("Startup optimization: target is 4 -> immediate U-turn at node 0");
    doUTurn();
    clearMarkerSmart(true); // drive off the node bar AFTER the turn
  } else {
    clearStartupMarker();
  }

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

  // ===== ULTRASONIC STOP (ADDED) =====
  // update distance every ~60ms (doesn't slow navigation)
  if (millis() - lastUSms >= US_PERIOD_MS) {
    lastUSms = millis();
    usCm = readUltrasonicCmFast();
  }


  // ===== SHARP IR UPDATE (ADDED) =====
  if (millis() - lastSharpMs >= SHARP_PERIOD_MS) {
    lastSharpMs = millis();
    readSharpSensors();
  }

  // Don't interfere while sitting on a marker bar
  bool onBar = isMarker();

  // ===== OBSTACLE BEHAVIOUR: RETURN TO PREVIOUS NODE =====
  if (!onBar) {

    // If we already returned and we're waiting on the node, just hold position
    if (atPrevNodeWaiting) {
      // Legacy safety mode. With permanent blocks enabled, we should never
      // enter this state. If we do, stop to avoid unpredictable motion.
      setMotors(0, 0);
      delay(5);
      return;
    }

    // If we detect obstacle, trigger the return routine
    bool ultrasonicHit = (usCm > 0 && usCm <= STOP_CM);
    bool sharpHit = sharpObstacleDetected();

    if (!returningToPrev && (ultrasonicHit || sharpHit)) {
      // SPECIAL CASE: treat the wall as "arrival at node 5" when heading to node 5.
      // This is your parking behaviour: stop and tell the server we're at node 5.
      if (targetNode == 5 && ultrasonicHit) {
        arriveAtWallNode5();
        return;
      }

      setMotors(0, 0);
      delay(60);
      startReturnToPreviousNode();
      delay(5);
      return;
    }
  }

  // ===== LINE FOLLOW DISABLE TIMER (MJ35 -> M5) =====
  if (lfDisableArmed && (int32_t)(millis() - lfDisableAtMs) >= 0) {
    lfDisableArmed = false;
    lineFollowEnabled = false;
    Serial.println("Line following DISABLED (2s after leaving MJ35 toward M5).");
  }

  if (millis() < ignoreMarkersUntil) {
    if (lineFollowEnabled) followLine();
    else setMotors(baseSpeed, baseSpeed);
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
    if (lineFollowEnabled) followLine();
    else setMotors(baseSpeed, baseSpeed);
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

  

// (obstacle junction override handled at junction lookup)


// =========================================
  // RETURN MODE: walk legPath backwards
  // =========================================
  if (returningToPrev) {
    // IMPORTANT:
    // On an obstacle-triggered U-turn, the first marker we hit after turning
    // is usually the SAME marker we last confirmed (legPath[legIdx]).
    // If we decrement legIdx immediately, the software and robot desync:
    // it *thinks* it is at the previous marker (often a real node), and it
    // will take the wrong junction action and/or report the wrong node.
    MarkerID curr;
    if (returnFirstMarker) {
      curr = legPath[legIdx];
      returnFirstMarker = false;
    } else {
      if (legIdx > 0) legIdx--;
      curr = legPath[legIdx];
    }

    // If we just traversed a previously-blocked segment, clear it.
    clearBlockedIfTraversed((int)cameFrom, (int)curr);

    // In reverse, "next" is the marker behind us (lower index)
    MarkerID next = curr;
    if (legIdx > 0) next = legPath[legIdx - 1];

    MarkerID prevEffective = prevMarker;
    if (uTurnCtxValid && uTurnPrevOverrideActive) {
      prevEffective = uTurnTriedMarker;   // pretend we came from the side we were trying to reach
      uTurnPrevOverrideActive = false;    // only for the FIRST return marker hit
    }

    Serial.print("RETURN marker hit. prev="); Serial.print((int)prevEffective);
    Serial.print(" curr="); Serial.print((int)curr);
    Serial.print(" next="); Serial.println((int)next);

    // Replan immediately from the first anchor marker after the U-turn.
    // Anchor = a marker we can confidently localize on: node OR junction.
    if (replanAtFirstAnchor && savedTargetNode >= 0 && (isRealNode(curr) || curr == MJ05 || curr == MJ35)) {
      replanAtFirstAnchor = false;

      obstOverrideActive = false;
      uTurnCtxValid = false;
      uTurnPrevOverrideActive = false;

      Serial.print("Returned to anchor marker "); Serial.print((int)curr);
      Serial.println(" after U-turn. Replanning to original target...");

      targetNode = savedTargetNode;
      buildLegFromMarker(curr, targetNode);
      legIdx = 0;

      returningToPrev = false;
      atPrevNodeWaiting = false;
      returnFirstMarker = false;

      prevMarker = curr;
      resetMarkerDebounce();
      ignoreMarkersUntil = millis() + 300;
      clearMarkerSmart(true);

      // If we are sitting on a junction marker, we must immediately take the
      // newly planned exit, otherwise the robot remains aligned for the RETURN
      // direction (the "next" printed earlier) and will drift the wrong way.
      if (curr == MJ05 || curr == MJ35) {
        if (legLen < 2) {
          Serial.println("Replan produced no onward step; stopping.");
          setMotors(0,0);
          delay(400);
          return;
        }
        MarkerID nextPlanned = legPath[1];
        // Use the approach-side context (prevEffective) for the very first
        // junction decision after replanning.
        MarkerID prevForLut = prevEffective;
        Action a2 = JUNC_LUT[prevForLut][curr][nextPlanned];
        Serial.print("Post-replan junction: prev="); Serial.print((int)prevForLut);
        Serial.print(" curr="); Serial.print((int)curr);
        Serial.print(" next="); Serial.println((int)nextPlanned);
        if (a2 == ACT_UNKNOWN) {
          Serial.println("LUT UNKNOWN for post-replan junction; stopping.");
          setMotors(0,0);
          delay(400);
          return;
        }
        if (a2 == ACT_STRAIGHT) doStraight();
        else if (a2 == ACT_LEFT90) doLeft90();
        else if (a2 == ACT_RIGHT90) doRight90();
        else if (a2 == ACT_UTURN) doUTurn();
        return;
      }
      return;
    }

    // Junction marker
    if (curr == MJ05 || curr == MJ35) {

// Expire any stale override (safety)
if (obstOverrideActive && (millis() - obstOverrideArmedAt) > OBST_OVERRIDE_TTL_MS) {
  Serial.println("Obstacle override expired (TTL).");
  obstOverrideActive = false;
}
      MarkerID prevForLut = prevEffective;
      if (obstOverrideActive && curr == obstJuncMarker) {
        prevForLut = obstOtherMarker;
        obstOverrideActive = false;
        Serial.print("Using obstacle override prevForLut="); Serial.println((int)prevForLut);
      }
      Action a = JUNC_LUT[prevForLut][curr][next];
      if (a == ACT_UNKNOWN) {
        Serial.print("LUT UNKNOWN at junction. prev="); Serial.print((int)prevForLut);
        Serial.print(" curr="); Serial.print((int)curr);
        Serial.print(" next="); Serial.println((int)next);
        // Safer than driving straight off-track
        setMotors(0,0);
        delay(400);
        return;
      }

      if (a == ACT_STRAIGHT) doStraight();
      else if (a == ACT_LEFT90) doLeft90();
      else if (a == ACT_RIGHT90) doRight90();
      else if (a == ACT_UTURN) doUTurn();

      clearMarkerSmart(true);

    // If we just left MJ35 heading to node 5, disable line following after 2 seconds
    if (curr == MJ35 && next == M5) {
      lineFollowEnabled = true; // keep following as we exit the junction
      lfDisableArmed = true;
      lfDisableAtMs = millis() + LF_DISABLE_DELAY_MS;
      Serial.println("Armed line-follow disable timer for MJ35 -> M5 (2s). ");
    }

    prevMarker = curr;
      return;
    }

    // If we reached the previous node (start of this leg), DO NOT wait.
    // User requirement: blocked sections never clear, so we immediately replan
    // to the original target using the permanent blocked-edge set.
    if (isRealNode(curr) && (int)curr == currentNode) {
      Serial.println("Returned to previous node. Permanent block: replanning now...");

      obstOverrideActive = false;
      uTurnCtxValid = false;
      uTurnPrevOverrideActive = false;

      returningToPrev = false;
      atPrevNodeWaiting = false;
      returnFirstMarker = false;

      // Replan from THIS node to the saved/original target.
      if (savedTargetNode >= 0) {
        targetNode = savedTargetNode;
        buildLegFromMarker(curr, targetNode);
        legIdx = 0;
      }

      setMotors(0, 0);
      resetMarkerDebounce();
      ignoreMarkersUntil = millis() + 300;
      prevMarker = curr;
      clearMarkerSmart(true);
      return;
    }

    clearMarkerSmart(false);
    prevMarker = curr;
    return;
  }

  // =========================================
  // NORMAL MODE
  // =========================================
  if (legIdx < legLen - 1) legIdx++;
  MarkerID curr = legPath[legIdx];

  // If we just traversed a previously-blocked segment, clear it.
  clearBlockedIfTraversed((int)prevMarker, (int)curr);

  MarkerID next = curr;
  if (legIdx < legLen - 1) next = legPath[legIdx + 1];

  Serial.print("Marker hit. prev="); Serial.print((int)prevMarker);
  Serial.print(" curr="); Serial.print((int)curr);
  Serial.print(" next="); Serial.println((int)next);

  // Junction marker
  if (curr == MJ05 || curr == MJ35) {

// Expire any stale override (safety)
if (obstOverrideActive && (millis() - obstOverrideArmedAt) > OBST_OVERRIDE_TTL_MS) {
  Serial.println("Obstacle override expired (TTL).");
  obstOverrideActive = false;
}
    MarkerID prevForLut = prevMarker;
      if (obstOverrideActive && curr == obstJuncMarker) {
        prevForLut = obstOtherMarker;
        obstOverrideActive = false;
        Serial.print("Using obstacle override prevForLut="); Serial.println((int)prevForLut);
      }
      Action a = JUNC_LUT[prevForLut][curr][next];
    if (a == ACT_UNKNOWN) {
        Serial.print("LUT UNKNOWN at junction. prev="); Serial.print((int)prevForLut);
        Serial.print(" curr="); Serial.print((int)curr);
        Serial.print(" next="); Serial.println((int)next);
        // Safer than driving straight off-track
        setMotors(0,0);
        delay(400);
        return;
      }

    if (a == ACT_STRAIGHT) doStraight();
    else if (a == ACT_LEFT90) doLeft90();
    else if (a == ACT_RIGHT90) doRight90();
    else if (a == ACT_UTURN) doUTurn();

    clearMarkerSmart(true);

    // If we just left MJ35 heading to node 5, disable line following after 2 seconds
    if (curr == MJ35 && next == M5) {
      lineFollowEnabled = true; // keep following as we exit the junction
      lfDisableArmed = true;
      lfDisableAtMs = millis() + LF_DISABLE_DELAY_MS;
      Serial.println("Armed line-follow disable timer for MJ35 -> M5 (2s). ");
    }

    prevMarker = curr;
    return;
  }

  // Node marker
  if (isRealNode(curr)) {
    int arrivedNode = (int)curr;

    // Re-enable line following when we reach any real node
    lineFollowEnabled = true;
    lfDisableArmed = false;

    Serial.print("Arrived at node "); Serial.println(arrivedNode);

    // Any special junction-approach override is no longer valid once we hit a real node
    obstOverrideActive = false;

    if (arrivedNode == targetNode) {
      currentNode = targetNode;
      // Permanent blocks: do NOT clear any blocked edges when arriving.
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
  if (c == 4) return false;                 // marker bar
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
// ===== START RETURN ROUTINE (ADDED) =====
// ===============================
void startReturnToPreviousNode() {
  if (returningToPrev || atPrevNodeWaiting) return;

  Serial.println("OBSTACLE: U-turn and return to previous node (and replan)...");

  // Remember where we were going
  savedTargetNode = targetNode;

  // Mark the *marker segment* we're currently traveling as blocked.
// Robust: blocks the real segment (node<->junction or node<->node) currently in use.
if (legIdx >= 0 && (legIdx + 1) < legLen) {
  int u = (int)legPath[legIdx];
  int v = (int)legPath[legIdx + 1];

  // Capture generic U-turn context
  uTurnFromMarker  = legPath[legIdx];
  uTurnTriedMarker = legPath[legIdx + 1];
  uTurnCtxValid = true;
  uTurnPrevOverrideActive = true;

  // Ensure prevMarker is sane at the moment we enter RETURN mode
  prevMarker = uTurnFromMarker;

  // Remember (up to) 2 blocked edges
  lastBlockedEdgeIdx = addBlockedEdge(u, v);
  Serial.print("Blocking segment "); Serial.print(u);
  Serial.print(" <-> "); Serial.print(v);
  Serial.print("  (slot "); Serial.print(lastBlockedEdgeIdx); Serial.println(")");
} else {
  Serial.println("WARN: Can't block segment (legIdx out of range).");
  uTurnCtxValid = false;
  uTurnPrevOverrideActive = false;
}

// Flip around
  


// Capture obstacle/junction context for robust junction turning after U-turn.
// The obstacle happened on segment (u <-> v). If either end is a junction (MJ05/MJ35),
// we will likely re-hit that same junction first after the U-turn, but prevMarker may still
// be the junction. Store which junction and which "other side" we attempted so the LUT
// can use a meaningful prev marker.
obstOverrideActive = false;
if (legIdx >= 0 && (legIdx + 1) < legLen) {
  MarkerID uM = legPath[legIdx];
  MarkerID vM = legPath[legIdx + 1];
  if (uM == MJ05 || uM == MJ35) {
    obstJuncMarker = uM;
    obstOtherMarker = vM;
    obstOverrideActive = true;
    obstOverrideArmedAt = millis();
  } else if (vM == MJ05 || vM == MJ35) {
    obstJuncMarker = vM;
    obstOtherMarker = uM;
    obstOverrideActive = true;
    obstOverrideArmedAt = millis();
  }
  if (obstOverrideActive) {
    Serial.print("Obstacle context: will re-hit junction "); Serial.print((int)obstJuncMarker);
    Serial.print(" coming from side "); Serial.println((int)obstOtherMarker);
  }
}

  doUTurn();
  // ensure we don't double-trigger a marker immediately after turning
  resetMarkerDebounce();
  ignoreMarkersUntil = millis() + 250;

  returningToPrev = true;
  // The first marker we will hit after the U-turn is almost always the same
  // marker we last confirmed (legPath[legIdx]). If we immediately decrement
  // legIdx, the code will *think* we're at the previous marker and will turn/
  // report the wrong node. This flag keeps the first return hit aligned.
  returnFirstMarker = true;
  replanAtFirstAnchor = true;
  obstacleBlocked = false;
}

// ===============================
// ===== LEG HELPERS (ADDED) =====
// ===============================
// Find the next REAL node marker ahead in legPath, starting at index startIdx.
// Returns node number 0..5, or -1 if none found.
int findNextNodeInLeg(int startIdx) {
  for (int i = startIdx; i < legLen; i++) {
    if (isRealNode(legPath[i])) return (int)legPath[i];
  }
  return -1;
}

// ===============================
// ===== WALL AS NODE 5 (ADDED) =====
// If we're heading to node 5 and we see the wall (ultrasonic <= STOP_CM),
// stop immediately and report "arrived at node 5" to the server.
// ===============================
void arriveAtWallNode5() {
  Serial.println("Wall detected while heading to node 5 -> treating as ARRIVAL at node 5.");

  // Hard stop
  setMotors(0, 0);
  delay(80);

  // Cancel any obstacle-return behaviour (we're done parking)
  returningToPrev = false;
  atPrevNodeWaiting = false;
  savedTargetNode = -1;

  // Cancel the MJ35->M5 line-follow disable timer (we're stopped anyway)
  lfDisableArmed = false;
  lineFollowEnabled = true;

  // Snap state to node 5
  currentNode = 5;
  targetNode = -1;
  prevMarker = M5;
  legLen = 0;
  legIdx = 0;

  // Tell server we've arrived at node 5
  int nextDest = notifyArrival(5);
  if (nextDest == -2) {
    finished = true;
    return;
  }

  // If server gives another target, continue from node 5
  if (nextDest >= 0) {
    targetNode = nextDest;
    buildLeg(currentNode, targetNode);
    prevMarker = nodeToMarker(currentNode);
    clearMarkerSmart(false);
  } else {
    // If server returns invalid/negative (other than -2), just hold.
    finished = true;
  }
}

// ===============================
// ===== SMART CLEAR MARKER =====
// ===============================
void clearMarkerSmart(bool afterTurn) {
  unsigned long t0 = millis();
  int offCount = 0;
  int lockCount = 0;

  // Push forward off the bar and wait until we're back on a "normal" line pattern.
  setMotors(CLEAR_FWD_SPEED, CLEAR_FWD_SPEED);

  while (millis() - t0 < 1200) {
    readSensors();

    if (!isMarker()) offCount++;
    else offCount = 0;

    // Require a few consecutive "good line" reads so we don't re-trigger the same bar
    if (!isMarker() && goodLineLock()) lockCount++;
    else lockCount = 0;

    if (offCount >= 8 && lockCount >= 4) break;

    delay(8);
  }

  setMotors(0, 0);
  delay(50);

  resetMarkerDebounce();

  // Stronger cooldown after turns/junctions to prevent legIdx skipping on the same bar
  ignoreMarkersUntil = millis() + (afterTurn ? 650 : 450);
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


// ===============================
// ===== ULTRASONIC FUNCTION (ADDED) =====
// Short timeout to avoid breaking navigation timing.
// Returns cm, or -1 if no echo.
// ===============================
int readUltrasonicCmFast() {
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);

  // 8000us timeout keeps loop responsive (we only care about close objects)
  unsigned long us = pulseIn(US_ECHO, HIGH, 8000UL);
  if (us == 0) return -1;

  return (int)(us / 58.0); // microseconds -> cm
}

// ============================================================
// ===== SHARP IR SENSOR SUPPORT (GPIO9 + GPIO10) =============
// ============================================================

static int readSharpAvg(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(250);
  }
  return (int)(sum / samples);
}

void readSharpSensors() {
  unsigned long now = millis();
  if (now - lastSharpMs < SHARP_PERIOD_MS) return;
  lastSharpMs = now;

  sharpRawL = readSharpAvg(SHARP_LEFT_PIN, SHARP_SAMPLES);
  sharpRawR = readSharpAvg(SHARP_RIGHT_PIN, SHARP_SAMPLES);

  sharpNearL = (sharpRawL >= SHARP_RAW_NEAR_L);
  sharpNearR = (sharpRawR >= SHARP_RAW_NEAR_R);
}

bool sharpObstacleDetected() {
  // Update states first (non-blocking due to SHARP_PERIOD_MS guard)
  readSharpSensors();
  return (sharpNearL || sharpNearR);
}
