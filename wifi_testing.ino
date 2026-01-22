#include <WiFi.h>

const char* ssid = "iPhone";
const char* password = "*********";

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("Connecting to Wi-Fi...");

  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    Serial.print(".");
    delay(500);
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to Wi-Fi.");
  }
}

void loop() {}


