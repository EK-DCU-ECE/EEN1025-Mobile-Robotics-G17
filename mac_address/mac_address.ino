// Sample code to get MAC Address:

#ifdef ESP32
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif

void setup(){
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);
  while(!WiFi.STA.started()) { 
    delay(100); 
    }
  Serial.println();
  Serial.print("ESP Board MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void loop(){}

