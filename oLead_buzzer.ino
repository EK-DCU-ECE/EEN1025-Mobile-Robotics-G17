#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED Display Settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1 
#define SCREEN_ADDRESS 0x3C // Standard I2C address for SSD1306
#define I2C_SDA 13          // Defined in Mobile Robotics Guide
#define I2C_SCL 14          // Defined in Mobile Robotics Guide

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ===== BUZZER PIN =====
const int BUZZER_PIN = 8; 

// ===============================
// ===== BLOCKED EDGE STATE ======
// (Added for obstacle re-plan logic; must be declared BEFORE buildLeg())
// ===============================
bool haveBlockedEdge = false;
int  blockedU = -1;
int  blockedV = -1;


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

// ===== ULTRASONIC (ADDED) =====
int readUltrasonicCmFast();

// ===== WALL AS NODE 5 (ADDED) =====
void arriveAtWallNode5();

// ===== OBSTACLE RETURN (ADDED) =====
void startReturnToPreviousNode();
int  findNextNodeInLeg(int startIdx);
void clearBlockedEdge();


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

// Version 2
void initJunctionLUT() {
  // 1. Reset all to UNKNOWN
  for(int a=0; a<NUM_MARKERS; a++)
    for(int b=0; b<NUM_MARKERS; b++)
      for(int c=0; c<NUM_MARKERS; c++)
        JUNC_LUT[a][b][c] = ACT_UNKNOWN;

  // ==========================================
  // ===== JUNCTION MJ05 (Top T-Junction) =====
  // ==========================================
  // Layout: Node 1 (South), Node 0 (East), Node 2 (West) [Based on visual correction]
  
  // Entering from Node 1 (Northbound towards junction)
  JUNC_LUT[M1][MJ05][M0] = ACT_LEFT90;    // Turn Left for Node 0
  JUNC_LUT[M1][MJ05][M2] = ACT_RIGHT90;   // Turn Right for Node 2

  // Entering from Node 0 (Westbound towards junction)
  JUNC_LUT[M0][MJ05][M2] = ACT_STRAIGHT;  // Straight for Node 2
  JUNC_LUT[M0][MJ05][M1] = ACT_RIGHT90;   // Turn Right for Node 1

  // Entering from Node 2 (Eastbound towards junction)
  JUNC_LUT[M2][MJ05][M0] = ACT_STRAIGHT;  // Straight for Node 0
  JUNC_LUT[M2][MJ05][M1] = ACT_LEFT90;    // Turn Left for Node 1

  // ==========================================
  // ===== JUNCTION MJ35 (Bottom 4-Way) =====
  // ==========================================
  // Layout: Node 1 (North), Node 5 (South), Node 3 (West), Node 4 (East)

  // Entering from Node 1 (Southbound into junction)
  JUNC_LUT[M1][MJ35][M5] = ACT_STRAIGHT;  // Straight to Parking
  JUNC_LUT[M1][MJ35][M3] = ACT_RIGHT90;   // Turn Right to Node 3
  JUNC_LUT[M1][MJ35][M4] = ACT_LEFT90;    // Turn Left to Node 4

  // Entering from Node 5 (Northbound into junction - e.g. leaving parking)
  JUNC_LUT[M5][MJ35][M1] = ACT_STRAIGHT;  // Straight to Node 1
  JUNC_LUT[M5][MJ35][M3] = ACT_LEFT90;    // Turn Left to Node 3
  JUNC_LUT[M5][MJ35][M4] = ACT_RIGHT90;   // Turn Right to Node 4

  // Entering from Node 3 (Eastbound into junction)
  JUNC_LUT[M3][MJ35][M4] = ACT_STRAIGHT;  // Straight to Node 4
  JUNC_LUT[M3][MJ35][M1] = ACT_LEFT90;    // Turn Left to Node 1
  JUNC_LUT[M3][MJ35][M5] = ACT_RIGHT90;   // Turn Right to Parking

  // Entering from Node 4 (Westbound into junction)
  JUNC_LUT[M4][MJ35][M3] = ACT_STRAIGHT;  // Straight to Node 3
  JUNC_LUT[M4][MJ35][M1] = ACT_RIGHT90;   // Turn Right to Node 1
  JUNC_LUT[M4][MJ35][M5] = ACT_LEFT90;    // Turn Left to Parking
}

/*
// Version 1: Not quite!
void initJunctionLUT() {
  // Initialize all to UNKNOWN
  for(int a=0; a<NUM_MARKERS; a++)
    for(int b=0; b<NUM_MARKERS; b++)
      for(int c=0; c<NUM_MARKERS; c++)
        JUNC_LUT[a][b][c] = ACT_UNKNOWN;

  // ===============================
  // ===== JUNCTION MJ05 (Fixed) ===
  // ===============================
  // Deduced Topology: 
  // Node 0 and Node 2 are Straight from each other.
  // Node 1 is to the Left of Node 0.

  // From Node 0
  JUNC_LUT[M0][MJ05][M1] = ACT_LEFT90;    // Turn Left to go to 1
  JUNC_LUT[M0][MJ05][M2] = ACT_STRAIGHT;  // Straight to go to 2

  // From Node 1
  JUNC_LUT[M1][MJ05][M0] = ACT_LEFT90;    // Turn Left to go to 0
  JUNC_LUT[M1][MJ05][M2] = ACT_RIGHT90;   // Turn Right to go to 2

  // From Node 2
  JUNC_LUT[M2][MJ05][M0] = ACT_STRAIGHT;  // Straight to go to 0
  JUNC_LUT[M2][MJ05][M1] = ACT_LEFT90;    // Turn Left to go to 1 (from 2 facing South, 1 is East/Left)


  // ===============================
  // ===== JUNCTION MJ35 (Keep) ====
  // ===============================
  // Presumed 4-Way: 1(N), 5(S), 3(W), 4(E)
  
  // From Node 1 (Heading South)
  JUNC_LUT[M1][MJ35][M5] = ACT_STRAIGHT; 
  JUNC_LUT[M1][MJ35][M3] = ACT_RIGHT90;
  JUNC_LUT[M1][MJ35][M4] = ACT_LEFT90;

  // From Node 3 (Heading East)
  JUNC_LUT[M3][MJ35][M4] = ACT_STRAIGHT;
  JUNC_LUT[M3][MJ35][M1] = ACT_LEFT90;
  JUNC_LUT[M3][MJ35][M5] = ACT_RIGHT90;

  // From Node 4 (Heading West)
  JUNC_LUT[M4][MJ35][M3] = ACT_STRAIGHT;
  JUNC_LUT[M4][MJ35][M1] = ACT_RIGHT90;
  JUNC_LUT[M4][MJ35][M5] = ACT_LEFT90;
}

// Original Version
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
*/
// ===============================
// ===== LEG / PATH (WEIGHTED NODES) =====
// ===============================
MarkerID legPath[20];
int legLen = 0;
int legIdx = 0;

const int NODES = 6;
const int MAX_DEG = 5;

// Version 3
int adj[NODES][MAX_DEG] = {
  {1, 2, 4, -1, -1},       // Node 0 connects to: 1, 2, 4
  {0, 2, 3, 4, 5},         // Node 1 connects to: 0, 2, 3, 4, 5
  {0, 1, 3, -1, -1},       // Node 2 connects to: 0, 1, 3
  {1, 2, 4, 5, -1},        // Node 3 connects to: 1, 2, 4, 5
  {1, 3, 5, -1, -1},       // Node 4 connects to: 1, 3, 5 (REMOVED 0)
  {1, 3, 4, -1, -1}        // Node 5 connects to: 1, 3, 4
};

// WEIGHTS (Cost of travel)
int w[NODES][MAX_DEG] = {
  {3, 2, 4, -1, -1},       // 0->1, 0->2, 0->4
  {3, 3, 3, 4, 3},         // 1->0, 1->2, 1->3, 1->4, 1->5
  {2, 3, 3, -1, -1},       // 2->0, 2->1, 2->3
  {3, 3, 2, 3, -1},        // 3->1, 3->2, 3->4, 3->5
  {4, 2, 3, -1, -1},       // 4->1, 4->3, 4->5
  {3, 3, 3, -1, -1}        // 5->1, 5->3, 5->4
};

/*
// Version 2
// FULL CONNECTIVITY MAP (Restored & Fixed)
// Added link between 2 and 3 so robot doesn't U-Turn at 2.
int adj[NODES][MAX_DEG] = {
  {1, 2, 4, -1, -1},       // Node 0 connects to: 1, 2, 4
  {0, 2, 3, 4, 5},         // Node 1 connects to: 0, 2, 3, 4, 5
  {0, 1, 3, -1, -1},       // Node 2 connects to: 0, 1, 3  <-- FIXED: Added 3
  {1, 2, 4, 5, -1},        // Node 3 connects to: 1, 2, 4, 5 <-- FIXED: Added 2
  {0, 1, 3, 5, -1},        // Node 4 connects to: 0, 1, 3, 5
  {1, 3, 4, -1, -1}        // Node 5 connects to: 1, 3, 4
};

// WEIGHTS (Cost of travel)
int w[NODES][MAX_DEG] = {
  {3, 2, 4, -1, -1},       // 0->1, 0->2, 0->4
  {3, 3, 3, 4, 3},         // 1->0, 1->2, 1->3, 1->4, 1->5
  {2, 3, 3, -1, -1},       // 2->0, 2->1, 2->3
  {3, 3, 2, 3, -1},        // 3->1, 3->2, 3->4, 3->5
  {4, 4, 2, 3, -1},        // 4->0, 4->1, 4->3, 4->5
  {3, 3, 3, -1, -1}        // 5->1, 5->3, 5->4
};
*/

/*
// Version 1
// Full Connectivity Map
// Node 1 connects to everything (0,2 via MJ05 and 3,4,5 via MJ35)
int adj[NODES][MAX_DEG] = {
  {1, 2, -1, -1, -1},       // Node 0 connects to: 1, 2
  {0, 2, 3, 4, 5},          // Node 1 connects to: 0, 2, 3, 4, 5 (The Hub)
  {0, 1, -1, -1, -1},       // Node 2 connects to: 0, 1
  {1, 4, 5, -1, -1},        // Node 3 connects to: 1, 4, 5
  {1, 3, 5, -1, -1},        // Node 4 connects to: 1, 3, 5
  {1, 3, 4, -1, -1}         // Node 5 connects to: 1, 3, 4
};

int w[NODES][MAX_DEG] = {
  {3, 2, -1, -1, -1},       // 0->1(3), 0->2(2)
  {3, 3, 3, 4, 3},          // 1->0(3), 1->2(3), 1->3(3), 1->4(4), 1->5(3)
  {2, 3, -1, -1, -1},       // 2->0(2), 2->1(3)
  {3, 2, 3, -1, -1},        // 3->1(3), 3->4(2), 3->5(3)
  {4, 2, 3, -1, -1},        // 4->1(4), 4->3(2), 4->5(3)
  {3, 3, 3, -1, -1}         // 5->1(3), 5->3(3), 5->4(3)
};
*/

/*
// Original version
// Node adjacency (ONLY nodes 0..5). Includes direct 0<->4 and 2<->3.
int adj[NODES][MAX_DEG] = {
  {1, 2, 4, -1, -1},      // 0
  {0, 2, 3, 4, 5},        // 1
  {0, 1, 3, -1, -1},      // 2
  {2, 4, 5, -1, -1},      // 3
  {0, 3, 5, -1, -1},      // 4
  {1, 3, 4, -1, -1}     // 5
};
*/

/*// Weights follow your spec:
// MJ35<->1 = 1, 0<->4 and 2<->3 = 2, 3<->MJ35 and 4<->MJ35 = 3, 3<->4 = 4.
// For MJ05 cluster edges (0/1/2) you didn't specify: set to 3 so it loses to the fast direct edges.
int w[NODES][MAX_DEG] = {
  {3, 3, 2, -1, -1},      // 0->1=3 (via MJ05), 0->2=3 (via MJ05), 0->4=2 (direct)
  {3, 3, 4, 4, 4},        // 1->0=3, 1->2=3, 1->3=4 (1+3 via MJ35), 1->4=4 (1+3), 1->5=4
  {3, 3, 2, -1, -1},      // 2->0=3, 2->1=3, 2->3=2 (direct)
  {2, 4, 4, 3, -1},      // 3->2=2, 3->1=4, 3->4=4 (direct slowest)
  {2, 4, 4, 3, -1},      // 4->0=2, 4->1=4, 4->3=4
  {4, 3, 3, -1, -1}     // 5->1=4
};
*/

// Insert junction markers ONLY when needed (junctions are NOT nodes)
bool usesMJ05(int a, int b) {
  return (a <= 2 && b <= 2 && a != b);
}

bool usesMJ35(int a, int b) {
  bool inA = (a == 1 || a == 3 || a == 4 || a == 5);
  bool inB = (b == 1 || b == 3 || b == 4 || b == 5);
  if (!inA || !inB) return false;

  // FIX: DO NOT exclude 3<->4 anymore.
  // On your real track, the 3.5 bar exists between 3 and 4,
  // so we must insert MJ35 to prevent it being mistaken for node 4/3.
  return (a != b);
}

// Weighted shortest path (Dijkstra) over nodes, then expand into marker list with junctions.
// Blocked edge (ADDED): avoid the segment where obstacle was detected
// (moved above buildLeg so it is in scope)
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

      // Skip blocked edge (both directions)
      if (haveBlockedEdge && ((u == blockedU && v == blockedV) || (u == blockedV && v == blockedU))) {
        continue;
      }

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
    // If blocking an edge made the graph disconnected, retry without blocking.
    if (haveBlockedEdge) {
      bool saved = haveBlockedEdge;
      int su = blockedU, sv = blockedV;
      clearBlockedEdge();
      buildLeg(fromNode, toNode);   // retry without the block
      // restore block for future replans (still useful), unless you want it cleared permanently.
      haveBlockedEdge = saved; blockedU = su; blockedV = sv;
      return;
    }

    // Fallback (should be rare): direct hop list
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



// ===== Junction U-turn "came-from" override (ADDED) =====
// After an obstacle-triggered U-turn, the first marker we re-hit can be the same
// junction marker (MJ05/MJ35). In that moment, prevMarker is ambiguous and the
// junction LUT can fall back to STRAIGHT -> outer loop. These flags force a valid
// "came from" marker for the first marker-hit after the U-turn.
bool     overridePrevMarker = false;
MarkerID forcedPrevMarker   = M0;

// Debounced marker state
int markerOnCount = 0;
int markerOffCount = 0;
const int MARKER_ON_CONFIRM  = 4;
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


// ===============================
// ===== DISPLAY UPDATE FUNCTION =====
// ===============================
// This function consolidates robot state (WiFi, Navigation, and Sensors) onto the Midas MDOB128064WV-YBI OLED display.
void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Line 1: WiFi Status
  display.setCursor(0, 0);
  if (WiFi.status() == WL_CONNECTED) {
    display.print("WiFi: ON  IP:..ok"); 
  } else {
    display.print("WiFi: Connecting...");
  }

  // Line 2: Navigation State
  display.setCursor(0, 10);
  display.print("Node: ");
  display.print(currentNode);
  display.print(" -> ");
  if (targetNode == -1) display.print("WAIT");
  else display.print(targetNode);

  // Line 3: Ultrasonic Sensor
  display.setCursor(0, 20);
  display.print("Dist: ");
  if (usCm == -1) display.print("---");
  else display.print(usCm);
  display.print(" cm");

  // Line 4: Status / Obstacles
  display.setCursor(0, 30);
  if (finished) {
    display.print("STATUS: FINISHED");
  } else if (returningToPrev) {
    display.print("OBSTACLE: RETURNING");
  } else if (atPrevNodeWaiting) {
    display.print("WAITING FOR CLEAR");
  } else {
    display.print("STATUS: RUNNING");
  }
  
  display.display();
}

// Blocked edge (ADDED): avoid the segment where obstacle was detected
// ===============================
// ===== SETUP =====
// ===============================
void setup() {
  Serial.begin(115200);

  // Initialize Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Added initialisation of the the I2C bus and the display hardware
  // Initialize I2C with specific pins for ESP32-S3 
  Wire.begin(I2C_SDA, I2C_SCL);

  // Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    // Infinite loop or error handling if display is critical
  }
  display.clearDisplay();
  display.display();
  


  delay(800);

  pinMode(leftPWM, OUTPUT);   pinMode(leftPhase, OUTPUT);
  pinMode(rightPWM, OUTPUT);  pinMode(rightPhase, OUTPUT);

  // ===== ULTRASONIC SETUP (ADDED) =====
  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);
  digitalWrite(US_TRIG, LOW);

  initJunctionLUT();
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

    // Update the OLED display
    // Placing here prevents I2C communication overhead from interfering
    // with the high-speed loop required for the followLine() logic
    updateOLED();
  }

  // Don't interfere while sitting on a marker bar
  bool onBar = isMarker();

  // ===== OBSTACLE BEHAVIOUR: RETURN TO PREVIOUS NODE =====
  if (!onBar) {

    // If we already returned and we're waiting on the node, just hold position
    if (atPrevNodeWaiting) {
      setMotors(0, 0);

      // resume only when obstacle clears
      if (usCm <= 0 || usCm >= CLEAR_CM) {
        Serial.println("Obstacle cleared at previous node. Replanning to original target...");

        atPrevNodeWaiting = false;

        // Replan to the original target (the one that was blocked), avoiding the blocked edge
        if (savedTargetNode >= 0) {
          targetNode = savedTargetNode;
          buildLeg(currentNode, targetNode);
        } else {
          // fallback if savedTargetNode somehow invalid
          int nextDest = notifyArrival(currentNode);
          if (nextDest == -2) { finished = true; return; }
          if (nextDest < 0)   { finished = true; return; }
          targetNode = nextDest;
          buildLeg(currentNode, targetNode);
        }

        prevMarker = nodeToMarker(currentNode);
        clearMarkerSmart(false);
      }

      delay(5);
      return;
    }

    // If we detect obstacle, trigger the return routine
    if (!returningToPrev && usCm > 0 && usCm <= STOP_CM) {
      // SPECIAL CASE: treat the wall as "arrival at node 5" when heading to node 5.
      // This is your parking behaviour: stop and tell the server we're at node 5.
      if (targetNode == 5) {
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
// Version 2
void handleMarkerHit() {
  setMotors(0, 0);
  delay(120);

  MarkerID cameFrom = prevMarker;
  if (overridePrevMarker) { prevMarker = forcedPrevMarker; overridePrevMarker = false; }

  // ==========================================
  // ===== OBSTACLE RETURN LOGIC START ======
  // ==========================================
  if (returningToPrev) {
    if (legIdx > 0) legIdx--;
    MarkerID curr = legPath[legIdx];
    MarkerID next = curr;
    if (legIdx > 0) next = legPath[legIdx - 1];

    if (curr == MJ05 || curr == MJ35) {
      Action a = JUNC_LUT[prevMarker][curr][next];
      if (a == ACT_UNKNOWN) a = ACT_STRAIGHT;
      if (a == ACT_STRAIGHT) doStraight();
      else if (a == ACT_LEFT90) doLeft90();
      else if (a == ACT_RIGHT90) doRight90();
      else if (a == ACT_UTURN) doUTurn();
      
      clearMarkerSmart(true);
      if (curr == MJ35 && next == M5) { lineFollowEnabled = true; lfDisableArmed = true; lfDisableAtMs = millis() + LF_DISABLE_DELAY_MS; }
      prevMarker = curr; 
      return;
    }

    if (isRealNode(curr) && (int)curr == currentNode) {
      returningToPrev = false; atPrevNodeWaiting = true;
      doUTurn(); // Spin around to face the network again
      setMotors(0, 0); resetMarkerDebounce(); ignoreMarkersUntil = millis() + 500;
      prevMarker = curr; return;
    }
    clearMarkerSmart(false); prevMarker = curr; return;
  }

  // ==========================================
  // ===== NORMAL NAVIGATION LOGIC ==========
  // ==========================================
  if (legIdx < legLen - 1) legIdx++;
  MarkerID curr = legPath[legIdx];
  MarkerID next = curr;
  if (legIdx < legLen - 1) next = legPath[legIdx + 1];

  if (curr == MJ05 || curr == MJ35) {
    Action a = JUNC_LUT[prevMarker][curr][next];
    if (a == ACT_UNKNOWN) a = ACT_STRAIGHT;
    
    if (a == ACT_STRAIGHT) doStraight();
    else if (a == ACT_LEFT90) doLeft90();
    else if (a == ACT_RIGHT90) doRight90();
    else if (a == ACT_UTURN) doUTurn();
    
    clearMarkerSmart(true);
    if (curr == MJ35 && next == M5) { lineFollowEnabled = true; lfDisableArmed = true; lfDisableAtMs = millis() + LF_DISABLE_DELAY_MS; }
    prevMarker = curr; return;
  }

  if (isRealNode(curr)) {
    int arrivedNode = (int)curr;
    lineFollowEnabled = true; lfDisableArmed = false;
    playBB8Random(); // Sound effect

    if (arrivedNode == targetNode) {
      currentNode = targetNode;
      clearBlockedEdge();
      
      int nextDest = notifyArrival(currentNode);
      
      // ====================================================
      // ===== FORCE FIX: OVERRIDE TARGET AT NODE 4 =========
      // ====================================================
      // If we are at Node 4, the only logical next step is 5.
      // We force this to prevent the robot going back to 1.
      if (currentNode == 4) {
        Serial.println("Arrived at 4. Forcing Target to 5.");
        nextDest = 5;
      }
      // ====================================================

      if (nextDest == -2) { finished = true; setMotors(0, 0); return; }
      if (nextDest < 0) { finished = true; return; }
      
      targetNode = nextDest;
      buildLeg(currentNode, targetNode);
      
      // U-Turn Logic
      if (currentNode == 0 && targetNode == 4) doUTurn();
      else if (legLen >= 2 && legPath[1] == cameFrom) doUTurn();
      
      clearMarkerSmart(false); prevMarker = curr; return;
    }
  }
  clearMarkerSmart(false); prevMarker = curr;
}

/*
// Version 1
void handleMarkerHit() {
  // 1. Immediate Stop & Basic Setup
  setMotors(0, 0);
  delay(120); // Short pause to stabilize

  MarkerID cameFrom = prevMarker;
  // Handle forced override (used when we detected an obstacle mid-leg)
  if (overridePrevMarker) { 
    prevMarker = forcedPrevMarker; 
    overridePrevMarker = false; 
  }

  // =========================================================
  // ===== MODE A: RETURNING TO PREVIOUS NODE (OBSTACLE) =====
  // =========================================================
  if (returningToPrev) {
    // We are backing up along the path we just took
    if (legIdx > 0) legIdx--; 
    MarkerID curr = legPath[legIdx];
    MarkerID next = curr;
    if (legIdx > 0) next = legPath[legIdx - 1]; // Look "backwards" in the array

    // --- Sub-case: Handling Junctions in Reverse ---
    if (curr == MJ05 || curr == MJ35) {
      // Look up the action to get from 'prevMarker' (where we are) 
      // to 'next' (where we need to go to get back to start)
      Action a = JUNC_LUT[prevMarker][curr][next]; 
      
      if (a == ACT_UNKNOWN) a = ACT_STRAIGHT; // Safety fallback
      
      if (a == ACT_STRAIGHT) doStraight();
      else if (a == ACT_LEFT90) doLeft90();
      else if (a == ACT_RIGHT90) doRight90();
      else if (a == ACT_UTURN) doUTurn();
      
      clearMarkerSmart(true);
      // Special case: If we reverse through MJ35 towards M5, re-enable line follow
      if (curr == MJ35 && next == M5) { 
        lineFollowEnabled = true; 
        lfDisableArmed = true; 
        lfDisableAtMs = millis() + LF_DISABLE_DELAY_MS; 
      }
      prevMarker = curr; 
      return;
    }

    // --- Sub-case: Arrived back at the Safe Node ---
    if (isRealNode(curr) && (int)curr == currentNode) {
      // We have successfully retreated to the start node.
      returningToPrev = false; 
      atPrevNodeWaiting = true; // Now we wait for the path to clear
      
      // *** CRITICAL FIX ***
      // We are currently facing INTO the node (staring at the wall/dead end).
      // We must spin 180 degrees to face the network so we can drive forward again.
      doUTurn(); 
      
      setMotors(0, 0); 
      resetMarkerDebounce(); 
      ignoreMarkersUntil = millis() + 500;
      prevMarker = curr; 
      return;
    }
    
    // Default: Just passed a marker while returning, keep going
    clearMarkerSmart(false); 
    prevMarker = curr; 
    return;
  }

  // =========================================================
  // ===== MODE B: NORMAL NAVIGATION (FORWARD) ===============
  // =========================================================
  
  // Advance along the path
  if (legIdx < legLen - 1) legIdx++;
  MarkerID curr = legPath[legIdx];
  MarkerID next = curr;
  if (legIdx < legLen - 1) next = legPath[legIdx + 1];

  // --- Sub-case: Junctions ---
  if (curr == MJ05 || curr == MJ35) {
    Action a = JUNC_LUT[prevMarker][curr][next];
    
    if (a == ACT_UNKNOWN) a = ACT_STRAIGHT; // Safety fallback

    if (a == ACT_STRAIGHT) doStraight();
    else if (a == ACT_LEFT90) doLeft90();
    else if (a == ACT_RIGHT90) doRight90();
    else if (a == ACT_UTURN) doUTurn();
    
    clearMarkerSmart(true);
    
    // Special handling for the tricky approach to Node 5 (Parking)
    if (curr == MJ35 && next == M5) { 
      lineFollowEnabled = true; 
      lfDisableArmed = true; 
      lfDisableAtMs = millis() + LF_DISABLE_DELAY_MS; 
    }
    prevMarker = curr; 
    return;
  }

  // --- Sub-case: Real Node Arrival ---
  if (isRealNode(curr)) {
    int arrivedNode = (int)curr;
    lineFollowEnabled = true; 
    lfDisableArmed = false;

    // Celebration Sound
    playBB8Random(); 

    // Check if this node is our actual target destination
    if (arrivedNode == targetNode) {
      currentNode = targetNode;
      
      // We successfully reached the target, so the edge wasn't blocked.
      clearBlockedEdge(); 
      
      // Ask server for next destination
      int nextDest = notifyArrival(currentNode);
      
      if (nextDest == -2) { 
        finished = true; 
        setMotors(0, 0); 
        return; 
      }
      
      if (nextDest < 0) { 
        // Error or unknown state, stop
        finished = true; 
        return; 
      }
      
      targetNode = nextDest;
      
      // Calculate path to new target
      buildLeg(currentNode, targetNode);
      
      // If we need to turn around immediately to start the new leg
      if (currentNode == 0 && targetNode == 4) doUTurn();
      else if (legLen >= 2 && legPath[1] == cameFrom) doUTurn();
      
      clearMarkerSmart(false); 
      prevMarker = curr; 
      return;
    }
  }

  // Default: Passing a node that isn't our destination (rare in this map, but good practice)
  clearMarkerSmart(false); 
  prevMarker = curr;
}
/*

// Original Version
/*
void handleMarkerHit() {
  setMotors(0, 0);
  delay(120);

  MarkerID cameFrom = prevMarker;

  

// Apply forced "came-from" marker once (used after obstacle U-turn near junctions)
if (overridePrevMarker) {
  prevMarker = forcedPrevMarker;
  overridePrevMarker = false;
  Serial.print("Applied forced prevMarker: ");
  Serial.println((int)prevMarker);
}

// =========================================
  // RETURN MODE: walk legPath backwards
  // =========================================
  if (returningToPrev) {
    // Step backwards along the already-built legPath
    if (legIdx > 0) legIdx--;
    MarkerID curr = legPath[legIdx];

    // In reverse, "next" is the marker behind us (lower index)
    MarkerID next = curr;
    if (legIdx > 0) next = legPath[legIdx - 1];

    Serial.print("RETURN marker hit. prev="); Serial.print((int)prevMarker);
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

    // If we reached the previous node (start of this leg), stop and wait
    if (isRealNode(curr) && (int)curr == currentNode) {
      Serial.println("Returned to previous node. Waiting for obstacle to clear...");

      returningToPrev = false;
      atPrevNodeWaiting = true;

      setMotors(0, 0);
      resetMarkerDebounce();
      ignoreMarkersUntil = millis() + 500;
      prevMarker = curr;
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


    // Trigger BB-8 sequence when arriving at a node.
    Serial.println("Node Reached: Playing BB-8 Sequence");
    playBB8Random();


    // Re-enable line following when we reach any real node
    lineFollowEnabled = true;
    lfDisableArmed = false;

    Serial.print("Arrived at node "); Serial.println(arrivedNode);

    if (arrivedNode == targetNode) {
      currentNode = targetNode;
      // We made it to the target; allow previously blocked edge again
      clearBlockedEdge();
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
*/

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
// ===== START RETURN ROUTINE (ADDED) =====
// ===============================
void startReturnToPreviousNode() {
  if (returningToPrev || atPrevNodeWaiting) return;

  Serial.println("OBSTACLE: U-turn and return to previous node (and replan)...");

  // Remember where we were going
  savedTargetNode = targetNode;

  // Mark the edge we're currently traveling as "blocked" for replanning.
  // We assume we're somewhere between legPath[legIdx] and legPath[legIdx+1].
  // The blocked edge is between currentNode (start of this leg) and the next REAL node ahead.
  int nextNode = findNextNodeInLeg(legIdx + 1);
  if (nextNode >= 0 && nextNode != currentNode) {
    haveBlockedEdge = true;
    blockedU = currentNode;
    blockedV = nextNode;
    Serial.print("Blocking edge "); Serial.print(blockedU);
    Serial.print(" <-> "); Serial.println(blockedV);
  }

  // Flip around
  

// If the last marker we hit was a junction (MJ05/MJ35), then after the U-turn
// the first marker we will hit again is typically that SAME junction. Using
// prevMarker==junction makes the LUT index [MJxx][MJxx][next] -> ACT_UNKNOWN,
// which defaults to STRAIGHT and sends the robot onto the outer loop.
// Force prevMarker to be the marker we were heading toward in the forward direction.
if (legIdx >= 0 && legIdx < legLen && (legPath[legIdx] == MJ05 || legPath[legIdx] == MJ35)) {
  if (legIdx + 1 < legLen) {
    forcedPrevMarker = legPath[legIdx + 1];  // forward-direction "came from"
    overridePrevMarker = true;
    Serial.print("Forcing prevMarker after U-turn to: ");
    Serial.println((int)forcedPrevMarker);
  }
}

  doUTurn();
  // ==========================================================
  // NEW: If the last confirmed marker was a junction (0.5 or 3.5),
  // STOP immediately after the 180° turn.
  // This is your requested safety/logic rule.
  // ==========================================================
  if (prevMarker == MJ05 || prevMarker == MJ35) {
    Serial.println("Previous marker was a junction (0.5/3.5) -> stopping after U-turn.");
    setMotors(0, 0);
    returningToPrev = false;
    atPrevNodeWaiting = true;   // reuse existing "waiting" state
    resetMarkerDebounce();
    ignoreMarkersUntil = millis() + 500;
    obstacleBlocked = false;
    return;
  }

  // ensure we don't double-trigger a marker immediately after turning
  resetMarkerDebounce();
  ignoreMarkersUntil = millis() + 250;

  returningToPrev = true;
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

void clearBlockedEdge() {
  haveBlockedEdge = false;
  blockedU = -1;
  blockedV = -1;
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
  // FIXED: Left Motor Back (-), Right Motor Fwd (+) -> Counter-Clockwise Turn
  setMotors(-turnSpeed, turnSpeed);
  // setMotors(turnSpeed, -turnSpeed);
  delay(SPIN_90_MS);
  reacquireLineAfterTurn(true, false);
  isTurning = false;
}

void doRight90() {
  isTurning = true;
  // FIXED: Left Motor Fwd (+), Right Motor Back (-) -> Clockwise Turn
  setMotors(turnSpeed, -turnSpeed);
  //setMotors(-turnSpeed, turnSpeed);
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

// ===============================
// ===== BB-8 SOUND EFFECTS =====
// ===============================
void playBB8Random() {
  int phrase = random(1, 5);
  if (phrase == 1) playHappy();
  else if (phrase == 2) playChatter();
  else if (phrase == 3) playWhistle();
  else playConfused();
}

void playBB8Rhythm() {
  // Mimics the rhythm of BB-8 (Fast, glitchy bursts)
  // We cannot change pitch on ABI-006-RC, so we modulate duration.
  
  // "Dit-dit-dweeee" pattern
  digitalWrite(BUZZER_PIN, HIGH); delay(40);  // Dit
  digitalWrite(BUZZER_PIN, LOW);  delay(30);
  digitalWrite(BUZZER_PIN, HIGH); delay(40);  // Dit
  digitalWrite(BUZZER_PIN, LOW);  delay(30);
  digitalWrite(BUZZER_PIN, HIGH); delay(150); // Dweeee (Longer)
  digitalWrite(BUZZER_PIN, LOW);  delay(50);
  
  // Quick chatter "dat-dat-dat"
  for(int i=0; i<3; i++) {
    digitalWrite(BUZZER_PIN, HIGH); delay(30);
    digitalWrite(BUZZER_PIN, LOW);  delay(30);
  }
  
  // Final confirmation chirp
  delay(50);
  digitalWrite(BUZZER_PIN, HIGH); delay(80);
  digitalWrite(BUZZER_PIN, LOW);
}

void playHappy() {
  slide(1000, 2000, 100); delay(50);
  slide(1500, 2500, 100);
  noTone(BUZZER_PIN);
}

void playChatter() {
  int count = random(5, 12);
  for(int i = 0; i < count; i++) {
    tone(BUZZER_PIN, random(500, 2500));
    delay(random(20, 80)); noTone(BUZZER_PIN); delay(random(20, 50));
  }
  noTone(BUZZER_PIN);
}

void playWhistle() {
  slide(800, 2200, 200);
  slide(2200, 800, 200);
  noTone(BUZZER_PIN);
}

void playConfused() {
  for(int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, random(300, 600)); delay(150);
    noTone(BUZZER_PIN); delay(50);
  }
}

void slide(int startFreq, int endFreq, int duration) {
  int steps = 50; int stepDelay = duration / steps;
  if (startFreq < endFreq) {
    for (int i = startFreq; i <= endFreq; i += (endFreq - startFreq) / steps) { delay(stepDelay); }
  } else {
    for (int i = startFreq; i >= endFreq; i -= (startFreq - endFreq) / steps) { delay(stepDelay); }
  }
}
